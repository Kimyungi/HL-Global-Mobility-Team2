"""경로 엔진 — 웨이포인트 CSV → vehicle frame ref points 변환 (ROS 무의존).

stack_gps 노드의 로직 코어. CLAUDE.md §5.5의 정신에 따라 ROS 없이 단독
임포트·테스트할 수 있다 (test/test_path_engine.py 참조).

좌표계 규약:
  - ENU 로컬: CSV 첫 점이 원점. east/north [m]. 위경도→미터는 등장방형 근사
    (record_waypoints.py와 동일 상수 — 작업 반경 수백 m에서 오차 무시 가능).
  - vehicle frame: 차량 = (0,0,0), x 전방(+), y 좌측(+). CLAUDE.md §3.
  - 차량 헤딩은 IMU 없이 "최근접 경로 접선 = 차량 헤딩"으로 가정한다.
    경로 추종 중에는 성립하고, 이탈 시 오차는 매 주기 재생성되는 ref와
    MPC 재계획으로 흡수된다 (tools/waypoints/README.md 인수인계 결정).

수치 처리:
  - GGA 좌표는 2~3cm 노이즈가 있으므로 접선·곡률을 이웃 한 칸이 아니라
    약 1m 베이스라인(중심 차분)으로 계산해 각도 노이즈를 줄인다.
"""
import csv
import math
import time

M_PER_DEG_LAT = 111_320.0


def wrap_angle(a):
    """[-pi, pi) 로 정규화."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def load_waypoints_csv(path, log=None):
    """record_waypoints.py가 만든 CSV → [(lat, lon)] (십진도).

    east_m/north_m 열은 기록 세션의 기준점에 묶여 있어 쓰지 않고,
    lat/lon에서 다시 계산한다. 연속 중복점은 제거.

    quality 열이 있으면 RTK FIXED(4)가 아닌 행은 버린다 — FLOAT(5)는
    m급 오차로 트랙을 오염시킨다 (2026-08-01 course_1의 idx 0·84가
    이웃 대비 2.5~2.7m 튀어 시작 횡오차 4m대의 한 원인이었음).
    버린 수는 log 콜백으로 보고.
    """
    pts, dropped = [], 0
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            q = row.get("quality")
            if q is not None and int(q) != 4:
                dropped += 1
                continue
            lat, lon = float(row["lat"]), float(row["lon"])
            if pts and pts[-1] == (lat, lon):
                continue
            pts.append((lat, lon))
    if dropped and log is not None:
        log(f"비-FIXED 웨이포인트 {dropped}개 제외 (FLOAT 오염 방지): {path}")
    if len(pts) < 2:
        raise ValueError(f"웨이포인트가 {len(pts)}개뿐 — 유효한 트랙이 아님: {path}")
    return pts


class PathEngine:
    """웨이포인트 트랙 1개를 들고 매 fix마다 vehicle frame 경로창을 잘라 준다.

    zone_ranges: [(start_idx, end_idx)] 포함 구간 목록 (웨이포인트 인덱스 기준).
    """

    # ref 시작점이 "전방"으로 인정되는 최소 vehicle-frame x [m].
    # 0으로 두면 옆구리(x≈0) 점이 뽑혀 도달 곡률이 폭주한다(2026-08-05 위빙).
    MIN_FORWARD_M = 0.3
    # 재합류 lookahead 계산에 쓰는 최소 회전반경 [m] (WHEELTEC 실측 1.5m,
    # DRIVE_GUIDE의 S자 코스 기록 근거). 이탈이 클수록 lookahead를 늘려
    # 도달 곡률 κ=2y/L² 이 1/R_min 을 넘지 않게 한다 — 넘으면 풀조향 포화.
    MIN_TURN_RADIUS_M = 1.5
    # ref 첫 점의 최대 방위각 [rad]. dSPACE는 **첫 점만** 목표로 쓴다(2026-08-14
    # 손상민 확인). 차가 트랙에서 멀어지는 쪽을 보고 있으면 트랙은 전진하는 만큼
    # 옆으로도 벌어져 **트랙 위 어떤 점도** 조준 가능한 방위에 없다 —
    # run_0814_201650 실측: 20점 전부 방위 76~84°(거의 정옆), 명령 선회반경
    # 2.3m인데 실제 24m로 사실상 직진, 횡오차 5m까지 발산.
    # 그래서 이 각을 넘으면 트랙 점 대신 **트랙 쪽으로 꺾인 중간 목표점**을 낸다.
    # 차가 돌아 정렬될수록 방위가 줄어 자연히 원래 동작으로 수렴한다.
    MAX_TARGET_BEARING_RAD = 0.436   # 25°
    # 재합류 목표점의 **거리 상한** [m]. dSPACE의 조향 응답은 방위뿐 아니라
    # 목표까지의 거리에 강하게 의존한다 (2026-08-15 실측, 4런 통합):
    #   목표 0.8~1.6m → 명령 곡률의 ~55% 실현 | 2.5m 초과 → 10% 아래로 붕괴
    # 같은 방위 25°·같은 속도 0.5m/s인데 목표 거리만 달랐던 두 복귀가 갈렸다:
    #   run_0815_142817 d=1.28~1.62m → str −0.21, 2초에 24° 선회, cross 0.43→0.10 ✅
    #   run_0815_144142 d=2.9~5.4m   → str −0.014, 선회 없음, cross 1.88→5.04 ❌
    # 상한이 없으면 **이탈이 클수록 트랙 점이 멀어져 목표도 멀어지고 조향은
    # 약해지는 양의 되먹임**이 된다 — 실측에서 명령 선회반경이 이탈과 함께
    # 3.4m(cross 1.7m)에서 6.3m(cross 5.0m)로 되레 완만해졌다.
    REJOIN_TARGET_MAX_M = 1.8
    # 재합류 접근각 α 가 최대(MAX_TARGET_BEARING_RAD)가 되는 횡오차 [m].
    # 이보다 가까우면 비례해서 접근각을 줄여 트랙에 **접선으로** 붙는다 —
    # 직각으로 꽂히면 그대로 통과해 반대편으로 나간다(run_0816_194948:
    # 도달 시점 헤딩오차 +74.5°, 반대편 1.7m 까지 이탈).
    # α 가 최대가 되는 횡오차 [m]. 근접 이득을 트랙 점과 비슷하게 유지하려면
    # 작아야 한다 — 트랙 점의 실효 이득이 ~52°/m 인데 α_max/FULL 이 그보다 훨씬
    # 작으면 온트랙 추종이 굼떠진다(3.0 이면 8.3°/m 로 6배 약함).
    # 0.5 → α_max/FULL = 50°/m 로 트랙 점과 비슷하다.
    REJOIN_FULL_CROSS_M = 0.5
    # ψₑ 변화율 감쇠 시상수 [s].
    #
    # ★ 2026-08-17: 기본값 0.6 → **0.0 (끔)**. 이 항은 dSPACE 조향 지연을
    #   선행 보상하려고 넣은 것이다 — run_0816_200226 실측에서 명령이 t=76.0에
    #   뒤집혔는데 차는 약 1초 뒤에야 선회를 멈췄고 그 사이 ψₑ가 −19°→−49°로
    #   30° 더 넘어갔다. **그 지연을 dSPACE 조향 PI가 없앴다**(2026-08-17,
    #   손상민). 보상 대상이 사라진 미분항은 10Hz GPS 헤딩의 **잡음 증폭기**로만
    #   남는다 — 틱 잡음 이득이 T_d/dt = ×6, EMA 0.5로 ×3.
    #   run_0817_011045 실측(gps 구간 66s): ψₑ 자체 sd 7.4° 인데 **감쇠항 sd
    #   13.6°·스파이크 296°** 로 댐핑 대상보다 항이 2배 크다. 결과가 GPS 구간의
    #   명령 채터링이다 — 같은 run의 차선 구간과 비교하면
    #     ref[0] 1.29m ↔ 2.06m | 명령 |κ| 0.212 ↔ 0.068 1/m
    #     방위 부호반전 1.53 ↔ 0.41 회/s | 틱간 Δ방위 sd 6.41° ↔ 0.94°
    #     실제 요레이트 MAD 3.0 ↔ 1.0 °/s
    #   덤프 재구성 추정으로 이 항만 빼면 Δ방위 sd 6.2→4.1°, 부호반전
    #   1.20→0.68회/s.
    #   되살릴 일이 있으면(조향 지연이 다시 생기면) 파라미터로 올리면 된다.
    #     b = ψₑ + sign(e)·α + T_d · dψₑ/dt
    REJOIN_RATE_DAMP_S = 0.0
    REJOIN_RATE_EMA = 0.5
    # 재합류 목표점의 **거리 하한** [m] (2026-08-17 신설).
    #
    # 종전 하한은 기하 제약 하나뿐이었다 — 2·R_min·sin(25°) = 1.267m, "도달 곡률이
    # R_min 을 넘지 않는 최소 거리". 그런데 run_0817_020112 실측에서 GPS 구간
    # ref[0] 거리의 **81.7%가 이 하한에 붙어** 있었다. 사실상 상수 1.267m다.
    # κ = 2·sin(b)/d 이므로 거리가 짧을수록 같은 방위 잡음이 그대로 증폭된다 —
    # 차선(2.04m) 대비 1.61배. 이것이 "GPS만 ref가 튄다"의 증폭단이다.
    #
    # 이 하한은 dSPACE 조향이 명령의 10~55%만 실현하던 시절의 보상이었는데,
    # **조향 PI 도입(2026-08-17, 손상민) 후 서보 실현율이 80~107%로 올라갔다**
    # (같은 run: 실제δ/명령δ = 차선 79.9% · GPS 91.7% · 회피 107.4%).
    # 보상 대상이 사라졌으므로 CLAUDE.md §3 ③ 이 "응답이 살아 있다"고 기록한
    # 밴드 [1.27, 1.8] 안에서 **완만한 쪽 끝**으로 옮긴다.
    # ⚠ 실차 재검증 전까지는 잠정값 — RUNBOOK §5-2 의 단계별 시험 순서를 따를 것.
    REJOIN_TARGET_MIN_M = 1.8
    # 접근각 α 가 쓰는 횡오차 e 의 저역통과 시상수 [s] (2026-08-17 신설).
    #
    # run_0817_020112 갱신열 분석: Δψₑ 의 lag-1 자기상관은 +0.002 로 **백색 성분이
    # 없다**(= 진짜 요운동이니 필터링하면 실동작을 죽인다). 반면 Δα 는 −0.142 로
    # **54%가 백색잡음**이고 그 sd 는 0.628° = e 환산 1.26cm — RTK 위치 잡음이다.
    # 그래서 필터는 **ψₑ 가 아니라 e 에만** 건다.
    #   τ=0.15s → 10Hz 백색 성분 약 1/2, 지연 0.6m/s 에서 9cm.
    REJOIN_E_LPF_S = 0.15

    def __init__(self, latlon_pts, n_points=30,
                 accel_ranges=(), parking_ranges=(), tangent_baseline_m=1.0,
                 lookahead_m=0.0, rate_damp_s=None, full_cross_m=None,
                 target_max_m=None, target_min_m=None, e_lpf_s=None):
        """lookahead_m: ref 시작점을 최근접점이 아니라 이만큼 전방의 트랙
        점으로 민다. dSPACE는 첫 점만 목표로 쓰므로(stack_avoid 실측 주석)
        최근접점(차 옆구리, x≈0)을 주면 도달 곡률 κ=2y/(x²+y²)가 폭주해
        풀조향 위빙을 유발한다 — 회피(0.4m 호 lookahead)와 같은 원리로
        전방 점을 목표로 제공 (2026-08-05 위빙 원인 분석).

        rate_damp_s / full_cross_m / target_max_m: 재합류 기하의 세 이득.
        None이면 클래스 기본값(REJOIN_*)을 쓴다. **주행 중 조정 가능해야 하므로**
        인스턴스 속성이다 — stack_gps 노드가 ROS 파라미터로 노출하고 콜백에서
        직접 갈아끼운다 (2026-08-17). 셋 다 "dSPACE 조향이 느리던 시절"에 맞춰
        잡은 보상값이라 조향 응답 특성이 바뀌면 함께 재조정해야 한다."""
        self.n_points = n_points
        self.accel_ranges = list(accel_ranges)
        self.parking_ranges = list(parking_ranges)
        self.lookahead_m = float(lookahead_m)
        self.rate_damp_s = (self.REJOIN_RATE_DAMP_S if rate_damp_s is None
                            else float(rate_damp_s))
        self.full_cross_m = (self.REJOIN_FULL_CROSS_M if full_cross_m is None
                             else float(full_cross_m))
        self.target_max_m = (self.REJOIN_TARGET_MAX_M if target_max_m is None
                             else float(target_max_m))
        self.target_min_m = (self.REJOIN_TARGET_MIN_M if target_min_m is None
                             else float(target_min_m))
        self.e_lpf_s = (self.REJOIN_E_LPF_S if e_lpf_s is None
                        else float(e_lpf_s))
        self._prev_psi_e = None      # 재합류 감쇠항용 — 직전 헤딩오차와 시각
        self._prev_psi_t = None
        self._psi_rate = 0.0         # [rad/s] EMA 저역통과된 dψₑ/dt
        self._e_lpf = None           # [m] 저역통과된 부호 있는 횡오차
        self._prev_e_t = None

        lat0, lon0 = latlon_pts[0]
        self._lat0, self._lon0 = lat0, lon0
        self._m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(lat0))

        self.e = []
        self.n = []
        for lat, lon in latlon_pts:
            e, n = self.to_enu(lat, lon)
            self.e.append(e)
            self.n.append(n)
        m = len(self.e)

        # 점 간격 추정 → 접선/곡률용 이웃 스텝 k (약 tangent_baseline_m)
        seg = [math.hypot(self.e[i + 1] - self.e[i], self.n[i + 1] - self.n[i])
               for i in range(m - 1)]
        spacing = sorted(seg)[len(seg) // 2]
        k = max(1, round(tangent_baseline_m / max(spacing, 1e-6)))
        self._spacing = spacing          # set_lookahead() 재계산용
        self._la_pts = max(0, round(lookahead_m / max(spacing, 1e-6)))

        # 접선 yaw: i-k → i+k 중심 차분 (끝단은 클램프)
        self.yaw = []
        for i in range(m):
            a, b = max(0, i - k), min(m - 1, i + k)
            self.yaw.append(math.atan2(self.n[b] - self.n[a],
                                       self.e[b] - self.e[a]))

        # 곡률: 헤딩 변화율 Δyaw / 호길이 (좌회전 +, vehicle frame y좌측+와 일치)
        self.curvature = []
        for i in range(m):
            a, b = max(0, i - k), min(m - 1, i + k)
            arc = sum(seg[a:b])
            dyaw = wrap_angle(self.yaw[b] - self.yaw[a])
            self.curvature.append(dyaw / arc if arc > 1e-6 else 0.0)

    def set_lookahead(self, lookahead_m):
        """주행 중 lookahead 변경 (ROS 파라미터 콜백용) — 폴백 인덱스도 재계산."""
        self.lookahead_m = float(lookahead_m)
        self._la_pts = max(0, round(self.lookahead_m / max(self._spacing, 1e-6)))

    def to_enu(self, lat, lon):
        return ((lon - self._lon0) * self._m_per_deg_lon,
                (lat - self._lat0) * M_PER_DEG_LAT)

    def _nearest_idx(self, e, n):
        best, best_d2 = 0, float("inf")
        for i in range(len(self.e)):
            d2 = (self.e[i] - e) ** 2 + (self.n[i] - n) ** 2
            if d2 < best_d2:
                best, best_d2 = i, d2
        return best, math.sqrt(best_d2)

    def _foot_on_track(self, idx, e, n):
        """최근접 **꼭짓점** idx 주변 두 선분에 내린 수선의 발 → (fe, fn, 수직거리).

        ★ 2026-08-17 신설. 종전엔 `_nearest_idx` 의 **점-대-점 거리**를 그대로
        cross_track_m 으로 내보냈다. 트랙은 이산 웨이포인트라 그 거리는 차가
        트랙 위를 얌전히 따라가도 **웨이포인트 간격에 맞춰 톱니로 진동한다** —
        수선의 발이 두 점의 중간에 있을 때 최대, 꼭짓점 옆일 때 최소.
          h ↔ √(h² + (L/2)²),  L = 웨이포인트 간격
        run_0817_020112 실측(간격 0.342m, 0.6m/s): 주기 0.583s · 진폭 0.081m,
        이론값 0.083m 과 2mm 이내 일치. 갱신당 |Δcross| 3.43cm 중 69%가 이 톱니였다.
        실제 수직 횡오차가 0.104m 로 일정한 구간에서 0.104~0.17m 를 오갔다.

        이 값은 **MGM 의 waypoint→lane 전이 게이트**(§4 `lane_entry_max_cross_m`,
        기본 0.5m)의 입력이다 — 0.5m 문턱에 0.08m 짜리 가짜 진동이 실려 있었다.
        속도·간격이 커지면 진폭도 커지므로(L/2 에 비례) 그대로 둘 수 없다.

        수선의 발은 최근접 꼭짓점에 인접한 두 선분 중 하나에 있다(트랙에 극단적
        예각이 없는 한). 그래서 O(1) — 전체 선분을 다시 훑지 않는다.
        """
        best = None
        for a in (idx - 1, idx):
            if a < 0 or a + 1 >= len(self.e):
                continue
            ex, ey = self.e[a + 1] - self.e[a], self.n[a + 1] - self.n[a]
            L2 = ex * ex + ey * ey
            if L2 < 1e-12:
                continue
            # 선분 위로 클램프한 투영 파라미터 t∈[0,1]
            t = ((e - self.e[a]) * ex + (n - self.n[a]) * ey) / L2
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            fe, fn = self.e[a] + t * ex, self.n[a] + t * ey
            d2 = (e - fe) ** 2 + (n - fn) ** 2
            if best is None or d2 < best[2]:
                best = (fe, fn, d2)
        if best is None:      # 트랙 점이 1개뿐 = 선분 없음 → 꼭짓점으로 폴백
            return self.e[idx], self.n[idx], math.hypot(e - self.e[idx],
                                                        n - self.n[idx])
        return best[0], best[1], math.sqrt(best[2])

    @staticmethod
    def _in_ranges(idx, ranges):
        return any(a <= idx <= b for a, b in ranges)

    def _target_idx(self, idx, ev, nv, c, s, cross=0.0):
        """ref 시작점 = **차량 앞쪽**에 있으면서 lookahead_m 이상 떨어진 첫 트랙 점.

        구현은 "최근접점에서 트랙 호길이로 lookahead_m 앞"(idx + _la_pts)이었는데,
        그건 차가 트랙 위에 있을 때만 전방을 뜻한다. 횡오차가 커지면 트랙 따라
        1m 앞이 **차량 기준으로는 뒤**가 되어 ref[0].x가 음수로 나오고, 전진밖에
        못 하는 차는 그 목표에 도달할 수 없다 — 복귀가 구조적으로 불가능해진다.
        (2026-08-14 run_0814_195116: 회피 후 cross 2.7m·헤딩 30° 이탈 상태에서
        ref[0]=(-3.15,-0.10) → 재합류 실패, cross가 4.5m까지 단조 발산)

        그래서 기준을 **차량으로부터의 유클리드 거리 + 전방 조건**으로 바꾼다.
        트랙 위에 정렬돼 있을 때는 기존과 사실상 같은 점을 고른다.
        전방에 후보가 없으면(차가 트랙을 등짐) 구 동작으로 폴백한다 — 이 경우는
        복귀가 아니라 정지가 답이고, MGM의 역방향 가드(|ref[0].yaw|>120°)가 잡는다.
        """
        last = len(self.e) - 1
        if self.lookahead_m <= 0.0:
            return idx          # lookahead 끔 = 최근접점 그대로 (구동작 보존)
        la2 = self.lookahead_m * self.lookahead_m
        fallback = None
        for i in range(idx, last + 1):
            de, dn = self.e[i] - ev, self.n[i] - nv
            x = c * de + s * dn
            if x < self.MIN_FORWARD_M:
                continue
            d2 = de * de + dn * dn
            if d2 < la2:
                continue
            if fallback is None:
                fallback = i        # 전방·거리 조건은 만족 (곡률만 과함)
            # 도달 곡률 κ = 2y/L² 이 1/R_min 이하인 점을 고른다.
            # |2y|/d2 ≤ 1/R_min  ⟺  |2y|·R_min ≤ d2.
            # 이탈이 클수록 더 먼 점이 뽑혀 재합류가 완만해진다 — 가까운 점을
            # 주면 조향이 포화돼 트랙을 가로질렀다 되돌아오는 진동이 된다.
            y = -s * de + c * dn
            if abs(2.0 * y) * self.MIN_TURN_RADIUS_M <= d2:
                return i
        if fallback is not None:
            return fallback
        return min(idx + self._la_pts, last)

    def snapshot(self, lat, lon, heading=None, now=None):
        """현재 fix → dict(points, accel_zone, parking_zone, idx, cross_track_m).

        points: [(x, y, yaw, curvature)] vehicle frame, 최근접점부터 앞으로
        n_points개 (트랙 끝에서는 남은 만큼만).

        heading: 차량 헤딩(ENU rad, 예: RMC 이동방향). None이면 "최근접 경로
        접선 = 차량 헤딩" 가정으로 폴백 — 정지·출발 직후 등 헤딩을 모를 때만
        쓰고, 이때는 차가 트랙 위에 진행 방향으로 정렬돼 있어야 유효하다
        (2026-08-01 첫 주행에서 이 가정 위반으로 선회 발산 — COG 도입 계기).
        """
        ev, nv = self.to_enu(lat, lon)
        idx, _vertex_dist = self._nearest_idx(ev, nv)
        fe, fn, dist = self._foot_on_track(idx, ev, nv)   # 선분까지 수직거리
        psi = self.yaw[idx] if heading is None else heading
        c, s = math.cos(psi), math.sin(psi)

        start = self._target_idx(idx, ev, nv, c, s, dist)
        points = []
        for i in range(start, min(start + self.n_points, len(self.e))):
            de, dn = self.e[i] - ev, self.n[i] - nv
            points.append((c * de + s * dn,          # x 전방
                           -s * de + c * dn,         # y 좌측
                           wrap_angle(self.yaw[i] - psi),
                           self.curvature[i]))
        # 첫 점의 방위가 너무 크면(= 트랙이 옆에 있고 차는 멀어지는 방향) 트랙 점
        # 대신 재합류용 중간 목표점으로 대체한다 (MAX_TARGET_BEARING_RAD 주석 참조).
        # 거리는 **응답이 살아 있는 밴드로 클램프**한다 (2026-08-15):
        #   상한 REJOIN_TARGET_MAX_M — 멀면 dSPACE 조향이 죽는다(주석의 실측 근거)
        #   하한 2·R_min·sin β    — 도달 곡률 R = d/(2·sinβ) 이 R_min 이상
        # 하한을 뒤에 적용해 R_min 이 항상 이긴다. yaw/curvature는 트랙 값을 그대로
        # 둔다 — MGM 역방향 가드가 ref[0].yaw를 "트랙 대비 차 방향"으로 해석하기 때문.
        if points and self.lookahead_m > 0.0:   # lookahead 끔 = 구동작 완전 보존
            px, py, pyaw, pcurv = points[0]
            # ★ 조건 없이 **항상** 접근각 법칙으로 첫 점을 만든다 (2026-08-16).
            #   종전엔 "트랙 점 방위 > 25°" 일 때만 치환해서, 복귀 막바지에 두 제어법
            #   사이를 10Hz로 오가며 명령이 튀었다 — run_0816_200949 t=59.7~62.0:
            #     +12.7° → −24.3° → +12.6° → −18.2° → −3.3° → +17.5°
            #   경계를 넘나들 때마다 30° 씩 뒤집히니 차가 어느 쪽도 못 따라간다.
            #   ("복귀는 하는데 그 뒤로 이상해진다"의 정체)
            #
            #   접근각 법칙은 이탈량에 관계없이 정의되므로 분기가 필요 없다.
            #   정상 온트랙 주행(cross 0.03~0.17m)에 대입해 확인했다 — 트랙 점을
            #   그대로 쓸 때와 명령 차이가 **±1~2°**(최대 4.65°)로 무시할 수준이다.
            #   e→0 이면 α→0 이라 b→ψₑ 로 수렴해 트랙 방향 정렬만 남는다.
            # 부호 있는 횡오차 = **수선의 발**의 차량 좌표 y (좌 +).
            # 종전엔 최근접 꼭짓점을 썼는데, 그러면 종방향 성분이 헤딩오차만큼
            # y 로 새어 들어온다(누설 ≈ 종방향 offset × sin ψₑ). run_0817_020112
            # 실측: 웨이포인트 위상에 따라 e 가 0.016m 흔들려 α 에 0.82°의 가짜
            # 진폭을 만들었다. 수선의 발은 정의상 종방향 성분이 0이라 누설이 없다.
            de0, dn0 = fe - ev, fn - nv
            e_signed = -s * de0 + c * dn0
            if now is None:
                now = time.monotonic()
            # e 저역통과 — 백색잡음이 있는 채널은 여기뿐이다(클래스 주석 참조).
            if self.e_lpf_s > 0.0:
                dt_e = None if self._prev_e_t is None else now - self._prev_e_t
                if self._e_lpf is None or dt_e is None or not (0.0 < dt_e < 1.0):
                    self._e_lpf = e_signed       # 첫 호출·시간 점프 = 리셋
                else:
                    k = dt_e / (self.e_lpf_s + dt_e)
                    self._e_lpf += k * (e_signed - self._e_lpf)
                e_signed = self._e_lpf
            self._prev_e_t = now
            # ψₑ 변화율 (감쇠항) — dt 이상치는 무시하고 EMA 로 저역통과
            if self._prev_psi_t is not None:
                dt = now - self._prev_psi_t
                if 0.0 < dt < 1.0:
                    raw = wrap_angle(pyaw - self._prev_psi_e) / dt
                    self._psi_rate = (self.REJOIN_RATE_EMA * raw +
                                      (1.0 - self.REJOIN_RATE_EMA) * self._psi_rate)
            self._prev_psi_e, self._prev_psi_t = pyaw, now

            alpha = self.MAX_TARGET_BEARING_RAD * min(
                1.0, abs(e_signed) / max(self.full_cross_m, 1e-6))
            b = (pyaw + math.copysign(alpha, e_signed) +
                 self.rate_damp_s * self._psi_rate)
            b = max(-self.MAX_TARGET_BEARING_RAD,
                    min(self.MAX_TARGET_BEARING_RAD, b))
            d = min(math.hypot(px, py), self.target_max_m)
            # 하한 두 개를 뒤에 적용해 항상 이기게 한다:
            #   ① 기하 제약 2·R_min·sin β — 도달 곡률 R 이 R_min 이상 (안전 바닥)
            #   ② REJOIN_TARGET_MIN_M — 잡음 증폭 억제 (재조정 대상, 클래스 주석)
            # target_min 이 max 보다 크게 설정돼도 밴드를 벗어나지 않도록 묶는다.
            d = max(d, 2.0 * self.MIN_TURN_RADIUS_M *
                    math.sin(self.MAX_TARGET_BEARING_RAD),
                    min(self.target_min_m, self.target_max_m))
            points[0] = (d * math.cos(b), d * math.sin(b), pyaw, pcurv)

        return {
            "points": points,
            "accel_zone": self._in_ranges(idx, self.accel_ranges),
            "parking_zone": self._in_ranges(idx, self.parking_ranges),
            "idx": idx,
            "cross_track_m": dist,
            "at_end": idx >= len(self.e) - 2,
        }
