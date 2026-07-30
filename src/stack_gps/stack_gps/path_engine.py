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


def load_waypoints_csv(path):
    """record_waypoints.py가 만든 CSV → [(lat, lon)] (십진도).

    east_m/north_m 열은 기록 세션의 기준점에 묶여 있어 쓰지 않고,
    lat/lon에서 다시 계산한다. 연속 중복점은 제거.
    """
    pts = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            lat, lon = float(row["lat"]), float(row["lon"])
            if pts and pts[-1] == (lat, lon):
                continue
            pts.append((lat, lon))
    if len(pts) < 2:
        raise ValueError(f"웨이포인트가 {len(pts)}개뿐 — 유효한 트랙이 아님: {path}")
    return pts


class PathEngine:
    """웨이포인트 트랙 1개를 들고 매 fix마다 vehicle frame 경로창을 잘라 준다.

    zone_ranges: [(start_idx, end_idx)] 포함 구간 목록 (웨이포인트 인덱스 기준).
    """

    def __init__(self, latlon_pts, n_points=30,
                 accel_ranges=(), parking_ranges=(), tangent_baseline_m=1.0):
        self.n_points = n_points
        self.accel_ranges = list(accel_ranges)
        self.parking_ranges = list(parking_ranges)

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

    def snapshot(self, lat, lon):
        """현재 fix → dict(points, accel_zone, parking_zone, idx, cross_track_m).

        points: [(x, y, yaw, curvature)] vehicle frame, 최근접점부터 앞으로
        n_points개 (트랙 끝에서는 남은 만큼만).
        """
        ev, nv = self.to_enu(lat, lon)
        idx, dist = self._nearest_idx(ev, nv)
        psi = self.yaw[idx]
        c, s = math.cos(psi), math.sin(psi)

        points = []
        for i in range(idx, min(idx + self.n_points, len(self.e))):
            de, dn = self.e[i] - ev, self.n[i] - nv
            points.append((c * de + s * dn,          # x 전방
                           -s * de + c * dn,         # y 좌측
                           wrap_angle(self.yaw[i] - psi),
                           self.curvature[i]))
        return {
            "points": points,
            "accel_zone": self._in_ranges(idx, self.accel_ranges),
            "parking_zone": self._in_ranges(idx, self.parking_ranges),
            "idx": idx,
            "cross_track_m": dist,
            "at_end": idx >= len(self.e) - 2,
        }
