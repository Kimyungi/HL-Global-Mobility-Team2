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
import math

from stack_gps.path_engine import wrap_angle


class HeadingFusion:

    def __init__(self, alpha=0.1, imu_timeout=0.5, sign=1.0,
                 gyro_gate=0.15, inn_gate=math.radians(60.0),
                 seed_n=5, seed_width=1.0, seed_spread=math.radians(25.0),
                 reseed_after=30, min_turn_radius=3.0, turn_settle=0.5):
        """alpha: offset 저역통과 이득 (COG 갱신 1회당).
        imu_timeout: IMU 샘플 신선도 한계 [s].
        sign: IMU yaw 부호 (+1 = CCW+, ENU와 동일 — HandsFree 기본).
        gyro_gate: offset 갱신 허용 선회율 [rad/s].
        inn_gate: 잔차 게이트 [rad] — 정렬 후 이보다 큰 COG 잔차는 거부.
          후진·역주행 시 COG는 차머리 반대(≈180°)라 offset을 통째로
          끌어내린다 (2026-08-03 주행 말미 실사례: 124.7°→21.1° 오염).
          정상 드리프트 보정은 수 ° 수준이므로 60°면 넉넉한 상한.
        seed_n/seed_width/seed_spread: 최초 정렬 합의 조건 — seed_width[s]
          이상에 걸친 seed_n개 표본이 seed_spread 안에 모여야 offset을
          확정한다. 출발 전 차를 뒤로 밀거나 뒤로 구르면 순간 COG가
          차머리 반대를 보고해 첫 정렬이 180° 오염되던 것 방지
          (2026-08-03 저속 run 실사례: offset -57°로 오염 → 주행 불능).
        reseed_after: 잔차 거부가 이 횟수 연속되면(유효 COG 기준 ≈3초)
          정렬 자체가 오염된 것으로 보고 재시드 — 오염 잠금 해제 경로.
        min_turn_radius[m]: 회전 반경 게이트 — speed < R·|gyro_z| 이면
          반경 R 미만의 원호 운동이므로 COG ≠ 차머리로 보고 거부.
          제자리 선회 시 안테나(회전 중심에서 ~1m)가 원호를 그리며
          0.3m/s로 '이동'해 COG를 오염시키던 것 차단 (2026-08-04
          진단 캡처 실증: 선회 스텝에서 offset -14.7° 오염).
        turn_settle[s]: 선회 종료 후 이 시간 안에 측정된 COG는 거부.
          0.5 = RMC course 지연 상한. 1.0이었을 때 위빙 주행(조용한 창
          1~2초)에서 표본이 굶주려 정렬 자체가 안 되는 부작용 실증
          (2026-08-04 밤 주행: 전 구간 미정렬 → COG 끊김 시 접선 폴백
          발산). 피벗 꼬리 방어는 '측정 시각이 선회 중' 조건으로 유지됨.
          COG 표본은 나이(≤1s)가 있어 '지금 자이로'로 게이트하면 선회 중
          찍힌 낡은 원호 표본이 정지 직후 통과·반복 소비돼 offset을 끌고
          간다 (2026-08-04 3차 시험 실증: 선회 직후 자세마다 offset 수십°
          이동). 같은 표본 재소비도 측정 시각 dedupe로 차단."""
        self._alpha = float(alpha)
        self._imu_timeout = float(imu_timeout)
        self._sign = float(sign)
        self._gyro_gate = float(gyro_gate)
        self._inn_gate = float(inn_gate)
        self._seed_n = int(seed_n)
        self._seed_width = float(seed_width)
        self._seed_spread = float(seed_spread)
        self._reseed_after = int(reseed_after)
        self._min_turn_radius = float(min_turn_radius)
        self._turn_settle = float(turn_settle)
        self._last_turn_t = None   # 마지막으로 |gyro| > gate 였던 시각
        self._last_cog_t = None    # 마지막으로 소비한 COG 측정 시각 (dedupe)
        self._imu = None           # (yaw_signed, t)
        self._gyro_z = 0.0         # 최신 선회율 — 게이트용 (없으면 0 = 통과)
        self._offset = None
        self._seed_buf = []        # [(target, t)] — 정렬 전 COG 표본
        self._reject_streak = 0
        self.last_innovation = None  # 최근 COG−융합 잔차 [rad] — 진단용
        self.rejected = 0            # 잔차 게이트 거부 누계 — 진단용
        self.reseeds = 0             # 오염 판정 재정렬 횟수 — 진단용
        self.arc_blocked = 0         # 회전 반경 게이트 차단 누계 — 진단용

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
        self._seed_buf = []
        self._reject_streak = 0
        self.last_innovation = None

    def _try_seed(self, target, t):
        """정렬 전 COG 표본 합의 — seed_width 이상에 걸친 seed_n개가
        seed_spread 안에 모이면 순환 평균으로 offset 확정."""
        buf = self._seed_buf
        buf.append((target, t))
        buf[:] = [(a, ta) for a, ta in buf if t - ta <= 12.0]  # 위빙 주기(~12s)보다 길게
        if len(buf) < self._seed_n:
            return
        if self._seed_n > 1 and t - buf[0][1] < self._seed_width:
            return
        mean = math.atan2(sum(math.sin(a) for a, _ in buf),
                          sum(math.cos(a) for a, _ in buf))
        if max(abs(wrap_angle(a - mean)) for a, _ in buf) > self._seed_spread:
            return  # 표본이 흩어짐 (뒤로 밀림·저속 노이즈 혼재) — 대기
        self._offset = wrap_angle(mean)
        self.last_innovation = 0.0
        buf.clear()

    def update_imu(self, yaw_rad, t, gyro_z=None):
        self._imu = (self._sign * yaw_rad, t)
        if gyro_z is not None:
            self._gyro_z = gyro_z
            if abs(gyro_z) > self._gyro_gate:
                self._last_turn_t = t   # 선회 중 — turn_settle 기산점

    def update_cog(self, cog_yaw, t, speed=None):
        """이동 중 유효한 COG(ENU rad)로 offset 추정. 유효성(속도·나이)
        판정은 호출자 몫. t는 COG '측정 시각' — 같은 표본은 1회만 소비."""
        if self._last_cog_t is not None and abs(t - self._last_cog_t) < 1e-3:
            return                  # 같은 측정 재소비 금지 (50Hz 루프 × 낡은 표본)
        self._last_cog_t = t
        if self._imu is None or t - self._imu[1] > self._imu_timeout:
            return
        if abs(self._gyro_z) > self._gyro_gate:
            return
        if (self._last_turn_t is not None
                and t - self._last_turn_t < self._turn_settle):
            self.arc_blocked += 1   # 선회 중/직후 측정된 표본 — 원호 접선 의심
            return
        if (speed is not None
                and speed < self._min_turn_radius * abs(self._gyro_z)):
            self.arc_blocked += 1   # 소반경 원호(제자리 선회 등) — COG ≠ 차머리
            return
        target = wrap_angle(cog_yaw - self._imu[0])
        if self._offset is None:
            self._try_seed(target, t)
            return
        inn = wrap_angle(target - self._offset)
        self.last_innovation = inn
        if abs(inn) > self._inn_gate:
            self.rejected += 1       # 후진/이상 기동의 COG — 오염 방지 거부
            self._reject_streak += 1
            if self._reject_streak >= self._reseed_after:
                # 유효 COG가 지속적으로 정렬과 모순 → 정렬 오염 판정, 재시드
                self.reseeds += 1
                self._offset = None
                self._seed_buf = []
                self._reject_streak = 0
            return
        self._reject_streak = 0
        self._offset = wrap_angle(self._offset + self._alpha * inn)

    def heading(self, t):
        """융합 헤딩(ENU rad) 또는 None (IMU 부재·정렬 전)."""
        if self._imu is None or self._offset is None:
            return None
        if t - self._imu[1] > self._imu_timeout:
            return None
        return wrap_angle(self._imu[0] + self._offset)
