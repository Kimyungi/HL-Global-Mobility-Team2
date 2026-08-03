"""IMU yaw × GPS COG 상보 융합 → ENU 절대 헤딩 (ROS 무의존).

문제: 단일 안테나 GPS의 COG는 이동 중에만 유효하다 — 정지·출발 직후에는
"경로 접선 = 차량 헤딩" 가정으로 폴백해야 했고, 이 가정이 깨지면 발산한다
(2026-08-01 첫 주행). IMU yaw는 항상 나오고 단기 안정(자이로)이지만
ENU 절대각이 아니다 — 장착 방위·전원 인가 시점·드리프트만큼 오프셋이 있다.

융합: 이동 중 offset = COG − yaw_imu 를 저역통과로 추정해 두면, 이후로는
    heading = yaw_imu + offset
이 정지·저속 포함 항상 유효한 ENU 절대 헤딩이 된다. 드리프트는 다음
직진 구간의 COG가 계속 보정한다.

offset 갱신 게이트 (호출자가 아니라 여기서 판정 — 융합의 내부 규칙):
  - IMU 샘플이 신선할 것 (COG와 같은 시점의 yaw와 비교해야 의미가 있음)
  - |gyro_z| < gyro_gate — 선회 중에는 COG가 실제 헤딩과 어긋나고(슬립·
    안테나 위치) COG 지연×선회율만큼 짝짓기 오차가 생기므로 직진에서만 갱신.
"""
from stack_gps.path_engine import wrap_angle


class HeadingFusion:

    def __init__(self, alpha=0.1, imu_timeout=0.5, sign=1.0,
                 gyro_gate=0.15):
        """alpha: offset 저역통과 이득 (COG 갱신 1회당).
        imu_timeout: IMU 샘플 신선도 한계 [s].
        sign: IMU yaw 부호 (+1 = CCW+, ENU와 동일 — HandsFree 기본).
        gyro_gate: offset 갱신 허용 선회율 [rad/s]."""
        self._alpha = float(alpha)
        self._imu_timeout = float(imu_timeout)
        self._sign = float(sign)
        self._gyro_gate = float(gyro_gate)
        self._imu = None           # (yaw_signed, t)
        self._gyro_z = 0.0         # 최신 선회율 — 게이트용 (없으면 0 = 통과)
        self._offset = None
        self.last_innovation = None  # 최근 COG−융합 잔차 [rad] — 진단용

    @property
    def offset(self):
        return self._offset

    @property
    def aligned(self):
        """offset 초기화 완료 여부 — False면 heading()은 항상 None."""
        return self._offset is not None

    def reset_alignment(self):
        """IMU 재연결(전원 재인가 가능성) 시 호출 — yaw 기준점이 바뀌었을 수
        있으므로 기존 offset을 폐기하고 다음 직진 COG로 재정렬한다.
        리셋 직후 heading()은 None → 호출자는 COG/접선 폴백으로 안전."""
        self._offset = None
        self.last_innovation = None

    def update_imu(self, yaw_rad, t, gyro_z=None):
        self._imu = (self._sign * yaw_rad, t)
        if gyro_z is not None:
            self._gyro_z = gyro_z

    def update_cog(self, cog_yaw, t):
        """이동 중 유효한 COG(ENU rad)로 offset 추정. 유효성(속도·나이)
        판정은 호출자 몫 — 여기서는 IMU 신선도·선회율 게이트만 본다."""
        if self._imu is None or t - self._imu[1] > self._imu_timeout:
            return
        if abs(self._gyro_z) > self._gyro_gate:
            return
        target = wrap_angle(cog_yaw - self._imu[0])
        if self._offset is None:
            self._offset = target
            self.last_innovation = 0.0
        else:
            inn = wrap_angle(target - self._offset)
            self._offset = wrap_angle(self._offset + self._alpha * inn)
            self.last_innovation = inn

    def heading(self, t):
        """융합 헤딩(ENU rad) 또는 None (IMU 부재·정렬 전)."""
        if self._imu is None or self._offset is None:
            return None
        if t - self._imu[1] > self._imu_timeout:
            return None
        return wrap_angle(self._imu[0] + self._offset)
