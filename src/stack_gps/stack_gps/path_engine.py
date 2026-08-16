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
    # 재합류 접근각이 최대(MAX_TARGET_BEARING_RAD)가 되는 횡오차 [m]. 이보다 가까우면
    # 비례해서 접근각을 줄여 트랙에 **접선으로** 붙는다 — 직각으로 꽂히면 그대로
    # 통과해 반대편으로 나간다(run_0816_194948: 도달 시점 헤딩오차 +74.5°,
    # 반대편 1.7m 까지 이탈).
    #
    # 1.0 → 3.0 (2026-08-16, run_0816_200226). 접근각 공식 자체는 설계대로
    # 동작했다 — t=76.0에 명령이 +14.7°→−8.4°로 **이미 뒤집혔다**. 문제는 차가
    # 못 따라온 것이다: 그 시점 선회율이 −23°/s 였고, 명령 반전 후에도
    # **약 1초간(t=77.25까지) 계속 돌아** ψₑ가 −19°→−49°로 30° 더 넘어갔다.
    # 조향 응답 지연(명령→str 0.18~0.44s) + 선회 관성이 합쳐진 값이다.
    #   ⇒ 되돌림을 그만큼 **일찍** 시작해야 한다. 같은 궤적 대입:
    #        FULL 1.0 → 되돌림 시작 t=76.0 (늦음, 실제로 30° 오버슈트)
    #        FULL 3.0 → 되돌림 시작 t=75.0 (1초 앞당김 = 지연분과 일치)
    # 큰 이탈에서 복귀력이 약해지지 않는다 — 3.0m 이상에서는 여전히 최대
    # 접근각(25°)이 그대로 나온다. 줄어드는 건 마지막 접근 구간뿐이다.
    # 그래도 넘치면 다음 후보는 비례 이득 축소(b = k·(ψₑ+α), k<1) 또는
    # ψₑ 변화율 기반 감쇠 항이다.
    REJOIN_FULL_CROSS_M = 3.0

    def __init__(self, latlon_pts, n_points=30,
                 accel_ranges=(), parking_ranges=(), tangent_baseline_m=1.0,
                 lookahead_m=0.0):
        """lookahead_m: ref 시작점을 최근접점이 아니라 이만큼 전방의 트랙
        점으로 민다. dSPACE는 첫 점만 목표로 쓰므로(stack_avoid 실측 주석)
        최근접점(차 옆구리, x≈0)을 주면 도달 곡률 κ=2y/(x²+y²)가 폭주해
        풀조향 위빙을 유발한다 — 회피(0.4m 호 lookahead)와 같은 원리로
        전방 점을 목표로 제공 (2026-08-05 위빙 원인 분석)."""
        self.n_points = n_points
        self.accel_ranges = list(accel_ranges)
        self.parking_ranges = list(parking_ranges)
        self.lookahead_m = float(lookahead_m)

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

    def snapshot(self, lat, lon, heading=None):
        """현재 fix → dict(points, accel_zone, parking_zone, idx, cross_track_m).

        points: [(x, y, yaw, curvature)] vehicle frame, 최근접점부터 앞으로
        n_points개 (트랙 끝에서는 남은 만큼만).

        heading: 차량 헤딩(ENU rad, 예: RMC 이동방향). None이면 "최근접 경로
        접선 = 차량 헤딩" 가정으로 폴백 — 정지·출발 직후 등 헤딩을 모를 때만
        쓰고, 이때는 차가 트랙 위에 진행 방향으로 정렬돼 있어야 유효하다
        (2026-08-01 첫 주행에서 이 가정 위반으로 선회 발산 — COG 도입 계기).
        """
        ev, nv = self.to_enu(lat, lon)
        idx, dist = self._nearest_idx(ev, nv)
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
            bearing = math.atan2(py, px)
            if abs(bearing) > self.MAX_TARGET_BEARING_RAD:
                # ── 방위는 "트랙까지"가 아니라 **"트랙에 접근하는 각"** 으로 낸다
                #    (2026-08-16). 종전엔 트랙 점 방위를 25°로 클램프만 해서, 차가
                #    이미 충분히 돌아섰는데도 횡오차가 남아 있으면 계속 "더 꺾어라"를
                #    냈다 — 적분기에 브레이크가 없는 꼴이라 트랙을 직각으로 가로질렀다.
                #    run_0816_194948: 회피 후 cross 2.87m 에서 복귀에 성공했으나
                #    t=59~68 **10초 내내 방위 −25.5°(포화)** 를 유지해 헤딩이
                #    −29°→+38° 로 67° 과회전했고, 트랙 도달 시점(t=71.5)에 헤딩오차가
                #    **+74.5°** 라 그대로 통과해 반대편 1.7m 까지 이탈했다.
                #      t=66  ψₑ=+19.0° → 필요 −6°  / 실제 −25.5° (4배 과다)
                #      t=68  ψₑ=+37.7° → 필요 +12.7° / 실제 −25.5° (**부호 반대**)
                #
                #    공식: 목표 접근각 α 를 횡오차로 정하고, 지금 헤딩오차 ψₑ 에서
                #    그 각까지 **남은 회전량**을 방위로 낸다.
                #      α    = MAX_TARGET_BEARING · min(1, |e| / REJOIN_FULL_CROSS_M)
                #      b    = ψₑ + sign(e)·α          (e = 트랙이 있는 쪽, 좌측 +)
                #    헤딩이 목표 접근각에 도달하면 b→0 이 되어 **더 안 꺾는다.**
                #    횡오차가 줄면 α 도 줄어 트랙에 접선으로 붙는다.
                #    ψₑ = points[0][2] = wrap(트랙 yaw − 차량 yaw) — 이미 계산돼 있다.
                de0, dn0 = self.e[idx] - ev, self.n[idx] - nv
                e_signed = -s * de0 + c * dn0        # 최근접 트랙점의 차량 좌표 y (좌 +)
                alpha = self.MAX_TARGET_BEARING_RAD * min(
                    1.0, abs(e_signed) / max(self.REJOIN_FULL_CROSS_M, 1e-6))
                b = pyaw + math.copysign(alpha, e_signed)
                b = max(-self.MAX_TARGET_BEARING_RAD,
                        min(self.MAX_TARGET_BEARING_RAD, b))   # 조준 가능 범위로 클램프
                d = min(math.hypot(px, py), self.REJOIN_TARGET_MAX_M)
                d = max(d, 2.0 * self.MIN_TURN_RADIUS_M *
                        math.sin(self.MAX_TARGET_BEARING_RAD))
                points[0] = (d * math.cos(b), d * math.sin(b), pyaw, pcurv)

        return {
            "points": points,
            "accel_zone": self._in_ranges(idx, self.accel_ranges),
            "parking_zone": self._in_ranges(idx, self.parking_ranges),
            "idx": idx,
            "cross_track_m": dist,
            "at_end": idx >= len(self.e) - 2,
        }
