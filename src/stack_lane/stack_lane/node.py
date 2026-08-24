"""stack_lane 노드 — OAK-D Pro + YOLOPv2 차선 검출 → /perception/lane_path
담당: 이현준

파이프라인: OAK-D 라이브 프레임 -> YOLOPv2 추론(stack_lane.yolopv2_infer) ->
BEV 워프(stack_lane.bev) -> 슬라이딩 윈도우+중심선(stack_lane.lane_fit) ->
lookahead(기본 3m) 지점 {x,y,yaw,curvature} + confidence(stack_lane.lane_path).
전 과정 개발/검증 이력은 PROJECT_BRIEF.md §9~§14 참조.

호모그래피: `homography_path` 파라미터(기본값 = config/homography.json)가 있으면
그걸 쓰고, 없으면 카메라 스펙 기반 analytic placeholder를 자동 사용한다 —
캘리브레이션이 나중에 끝나면 그 파일만 놓으면 코드 변경 없이 자동 전환된다
(PROJECT_BRIEF.md §9 설계). placeholder 사용 중엔 confidence·형태는 정상이지만
x,y의 절대 거리 정확도는 보장되지 않으니 실측 캘리브레이션 전 실주행 투입 금지.

주의:
- 이 노드는 MGM 10ms 루프와 별도 프로세스다 (CLAUDE.md §5.2). 카메라 fps는
  기본 30(=~33ms)으로 REQUIREMENTS.md가 가정한 "카메라 100ms"보다 빠르지만,
  MGM은 항상 최신 스냅샷만 pull하므로 더 빠른 갱신은 문제 되지 않는다(자유도일 뿐).
  필요하면 `camera_fps` 파라미터로 100ms(=10fps)에 맞출 수 있음.
- 카메라 기동 직후 노출 적응 전 프레임은 오검출 위험이 있어 `warmup_frames`만큼
  버린다 (2026-08-07 실차 테스트에서 발견).
- v_ref·정지 판단·모드 판단은 하지 않는다 (REQUIREMENTS.md 금지사항).
- `/perception/stopline`은 이 노드에서 발행하지 않는다 — 정지선 검출은 팀 내
  다른 담당자에게 재배정됨 (PROJECT_BRIEF.md §6 참고, REQUIREMENTS.md와 실제
  배정이 다른 상태이니 팀과 확인 후 REQUIREMENTS.md 갱신 권장).

실행 예:
  ros2 run stack_lane stack_lane_node --ros-args \
      -p device:=0 -p lookahead_m:=3.0
"""
from __future__ import annotations

import time
from dataclasses import replace

import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image

from fma_interfaces.msg import LanePath, RefPoint

from stack_lane.bev import BevGrid, DEFAULT_HOMOGRAPHY_PATH, load_homography
from stack_lane.debug_draw import build_debug_frame
from stack_lane.lane_path import estimate_lane_path
from stack_lane.logging_utils import CsvFrameLogger
from stack_lane.yolopv2_infer import DEFAULT_WEIGHTS, infer, load_model, preprocess, resolve_device


class StackLaneNode(Node):

    def __init__(self):
        super().__init__('stack_lane_node')

        self.declare_parameter('weights', str(DEFAULT_WEIGHTS))
        self.declare_parameter('device', '0')  # 'cpu' 또는 cuda 인덱스
        self.declare_parameter('img_size', 640)
        self.declare_parameter('lookahead_m', 3.0)
        # 다점 출력 (2026-08-08 조향 진단 결과 반영 — lane_path.py 모듈 docstring 참조)
        self.declare_parameter('n_points', 20)
        self.declare_parameter('points_x_start', 2.5)
        self.declare_parameter('points_x_end', 6.0)
        # REF_POINT_00 근거리 치환 (2026-08-08 도입, 2026-08-16 동적화 — lane_path.py
        # estimate_lane_path()/_dynamic_ref_point0_lookahead() docstring 참조).
        # 기본값(0.0)은 비활성 = 기존 동작 그대로. 켜면 이 값은 "차가 차선 중심에
        # 잘 있을 때(c0 작을 때) 쓰는 가장 공격적인 기준 거리"가 된다 — 실제 매
        # 프레임 거리는 c0(드리프트)·c2(도로 곡률)에 따라 이 값~points_x_start
        # 사이에서 동적으로 정해짐. 1.15m = stack_avoid/avoid_to_ref.py가 GPS
        # candump 실측(run1_20260803/0806)으로 확인한 REF_POINT_00 규약 거리
        # 중앙값(0.25~0.97m) 근처, 카메라 최소 가시거리(2.5m) 제약 하에서
        # 시도해볼 첫 값 (2026-08-16 결정, 실차 미검증).
        self.declare_parameter('ref_point0_lookahead_m', 1.15)
        self.declare_parameter('ref_point0_extrap_mode', 'quadratic')  # 'linear' | 'quadratic'
        self.declare_parameter('ref_point0_min_confidence', 0.5)
        # c0(다항식 상수항 = x=0에서의 y) 기반 드리프트 판정 구간 — 이 이하면
        # base_lookahead_m 그대로(공격적), 이 이상이면 points_x_start까지 물러섬
        # (보수적), 사이는 선형보간. c2(도로 곡률)와 분리된 신호라 S자·급커브에서
        # 곡률 때문에 y가 커지는 것과 실제 이탈을 구분하기 위함.
        self.declare_parameter('ref_point0_c0_safe_m', 0.3)
        self.declare_parameter('ref_point0_c0_unsafe_m', 1.0)
        # 최소 회전반경 [m] — WHEELTEC 실측치, stack_gps PathEngine.MIN_TURN_RADIUS_M과
        # 동일 물리 상수. 실제로 고른 거리의 요구 곡률이 이걸 넘으면(|2y|·R_min>d²)
        # 이유 불문 더 먼 쪽으로 밀어낸다(하드 안전장치).
        self.declare_parameter('ref_point0_min_turn_radius_m', 1.5)
        # 프레임 간 연속성 체크 (2026-08-08, 편측 오검출 진단 — lane_path.py
        # estimate_lane_path()의 prev_y/max_y_jump_m 참조).
        self.declare_parameter('max_y_jump_m', 1.0)
        # HELD->SEARCH->LOST 점진적 신뢰도 감쇠 (2026-08-10, YOLOPv2_PC_LANE_CORRIDOR
        # 참조 프로젝트의 LaneTrajectoryTracker 상태머신 이식) — 기존 "y_jump_reset_after
        # 프레임 뒤 무조건 통과" 방식은 리셋 순간 검사 없이 큰 튐을 그대로 받아들이는
        # 부작용이 실측으로 확인됨(steering_smooth_204404.csv t=6.37s, 3.1m 튐).
        # 대신: 거부 시작 -> HELD(직전 값 유지, confidence만 매 프레임 hold_confidence_decay
        # 배 감쇠) -> hold_frames 초과 시 SEARCH(같은 값 유지하되 confidence 바닥,
        # 동시에 연속성 체크 허용폭을 서서히 넓힘) -> search_frames 초과 시 LOST(완전
        # 리셋, 검사 없이 다음 값 수용) — 급격한 단일 리셋 지점이 없어짐.
        self.declare_parameter('hold_frames', 12)            # ~0.6s @20Hz
        self.declare_parameter('hold_confidence_decay', 0.65)
        self.declare_parameter('search_frames', 30)           # ~1.5s @20Hz, 이후 LOST
        self.declare_parameter('search_min_confidence', 0.05)
        self.declare_parameter('search_max_jump_widen', 3.0)  # SEARCH 끝 시점 max_y_jump_m 배율
        # 다항식 계수 EMA 저역통과 필터 (2026-08-08, 오실레이션 완화 — lane_path.py
        # estimate_lane_path()의 prev_coeffs/coeff_smoothing_alpha 참조).
        # 1.0 = 비활성(기존 동작). 작을수록 부드럽지만 반응이 느려짐 — 실측 튜닝 필요.
        self.declare_parameter('coeff_smoothing_alpha', 1.0)
        self.declare_parameter('homography_path', str(DEFAULT_HOMOGRAPHY_PATH))
        # OAK-D MxID 핀닝 (CLAUDE.md §6 — 2대 운용 시 어느 노드가 어느 카메라를
        # 잡을지 비결정적이므로 필수). 기본값 = 차선용 OAK-D Pro 실측 MxID
        # (2026-08-11 확정, 팀장). 빈 문자열이면 첫 가용 장치 사용(단독 시험용).
        self.declare_parameter('camera_mxid', '14442C105157D3D200')
        self.declare_parameter('camera_fps', 10)
        # USB 링크 속도 상한. 'super' = 제한 없음(SuperSpeed 5Gbps로 열거).
        # 'high' = USB 2.0 강제 — SuperSpeed 신호 자체가 사라져 GNSS L1(1575MHz)
        # 방사 잡음의 주 원인이 제거된다 (2026-08-14 실측: 카메라 부팅 시 GPS
        # C/N0가 42→28dB, 근접 시 17dB까지 붕괴. camera_fps를 30→10으로 낮춰도
        # 무효였는데, 그건 페이로드일 뿐 링크는 계속 5Gbps로 돌기 때문).
        # ⚠ 'high'는 대역폭이 ~40MB/s라 1280x720 BGR 30fps(83MB/s)가 안 들어간다
        # — 반드시 camera_fps를 10 이하로 함께 낮출 것 (추론은 5.8Hz라 무손실).
        # ★ 기본값을 'high'/10 으로 뒤집었다 (2026-08-24, 인수인계).
        #   USB3 로 열거되면 GNSS L1 이 덮여 RTK 가 죽는다 — 그걸 피하려면
        #   매 launch 마다 인자를 손으로 붙여야 했는데, 한 번 잊으면 위성 수도
        #   HDOP 도 RTCM 도 정상으로 보이는 채 FIXED 만 안 잡혀 원인을 찾기 어렵다.
        #   안전한 쪽을 기본으로 두고, USB3 가 필요하면 그때 명시적으로 올린다.
        self.declare_parameter('usb_speed', 'high')
        self.declare_parameter('warmup_frames', 30)
        self.declare_parameter('poll_period_sec', 0.02)
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('log_csv', '')

        self.img_size = int(self.get_parameter('img_size').value)
        self.lookahead_m = float(self.get_parameter('lookahead_m').value)
        self.n_points = int(self.get_parameter('n_points').value)
        self.points_x_start = float(self.get_parameter('points_x_start').value)
        self.points_x_end = float(self.get_parameter('points_x_end').value)
        ref_point0_lookahead_m = float(self.get_parameter('ref_point0_lookahead_m').value)
        self.ref_point0_lookahead_m = ref_point0_lookahead_m if ref_point0_lookahead_m > 0.0 else None
        self.ref_point0_extrap_mode = str(self.get_parameter('ref_point0_extrap_mode').value)
        self.ref_point0_min_confidence = float(self.get_parameter('ref_point0_min_confidence').value)
        self.ref_point0_c0_safe_m = float(self.get_parameter('ref_point0_c0_safe_m').value)
        self.ref_point0_c0_unsafe_m = float(self.get_parameter('ref_point0_c0_unsafe_m').value)
        self.ref_point0_min_turn_radius_m = float(self.get_parameter('ref_point0_min_turn_radius_m').value)
        self.max_y_jump_m = float(self.get_parameter('max_y_jump_m').value)
        self.hold_frames = int(self.get_parameter('hold_frames').value)
        self.hold_confidence_decay = float(self.get_parameter('hold_confidence_decay').value)
        self.search_frames = int(self.get_parameter('search_frames').value)
        self.search_min_confidence = float(self.get_parameter('search_min_confidence').value)
        self.search_max_jump_widen = float(self.get_parameter('search_max_jump_widen').value)
        self.coeff_smoothing_alpha = float(self.get_parameter('coeff_smoothing_alpha').value)
        self._prev_y = None  # 연속성 체크용 — 마지막으로 채택(mode!=none)됐던 raw y
        self._prev_coeffs = None  # 스무딩용 — 직전 프레임의 smoothed 계수
        # 추적 상태머신: 'valid'(방금 새로 검출) | 'held' | 'search' | 'lost'
        self._track_status = 'lost'
        self._age_frames = 0        # 현재 상태(held/search)에서 경과 프레임 수
        self._held_estimate = None  # HELD/SEARCH 중 그대로 재발행할 마지막 정상 LaneEstimate
        self.warmup_frames = int(self.get_parameter('warmup_frames').value)
        self._frames_seen = 0
        self._cap_ts_warned = False      # 캡처 시각 경고 1회만
        self._pipeline_ms = []           # 캡처→발행 지연 표본 (주기 로깅용)
        self._frames_dropped = 0         # 큐에서 버린 낡은 프레임 누계

        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.debug_pub = None
        self.bridge = None
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, '/perception/lane_debug_image', 1)
            self.bridge = CvBridge()

        log_csv_path = str(self.get_parameter('log_csv').value)
        self.logger_csv = None

        device_arg = str(self.get_parameter('device').value)
        self.device, self.half = resolve_device(device_arg)
        weights = str(self.get_parameter('weights').value)
        self.model = load_model(weights, self.device, self.half)
        self._warmup_model()

        homography_path = str(self.get_parameter('homography_path').value) or None
        self.H, self.is_placeholder, meta = load_homography(homography_path)
        if self.is_placeholder:
            self.get_logger().warn(
                '실측 호모그래피 없음 — placeholder 사용 중 '
                f'(실좌표 정확도 보장 안 됨): {meta.get("placeholder_params")}')
        self.H_inv = np.linalg.inv(self.H)
        self.grid = BevGrid()

        if log_csv_path:
            self.logger_csv = CsvFrameLogger(log_csv_path, is_placeholder_homography=self.is_placeholder)
            self.get_logger().info(f'CSV 로깅: {log_csv_path}')

        self._setup_camera(int(self.get_parameter('camera_fps').value))

        self.pub = self.create_publisher(LanePath, '/perception/lane_path', 1)
        period = float(self.get_parameter('poll_period_sec').value)
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(
            f'stack_lane_node 준비됨 (ref_point0_base_lookahead_m={self.ref_point0_lookahead_m}, '
            f'c0_safe_m={self.ref_point0_c0_safe_m}, c0_unsafe_m={self.ref_point0_c0_unsafe_m}, '
            f'min_turn_radius_m={self.ref_point0_min_turn_radius_m}, '
            f'extrap_mode={self.ref_point0_extrap_mode}, '
            f'min_confidence={self.ref_point0_min_confidence}) — CSV의 ref_point0_applied/'
            f'ref_point0_x와 대조해 사후 분석할 것(ref_point0_x가 매 프레임 달라지면'
            f' 동적 로직이 작동 중인 것)')

    def _warmup_model(self) -> None:
        dummy = torch.zeros(1, 3, self.img_size, self.img_size, device=self.device)
        dummy = dummy.half() if self.half else dummy.float()
        with torch.no_grad():
            self.model(dummy)
        if self.device.type == 'cuda':
            torch.cuda.synchronize()
        elif self.device.type == 'xpu':
            torch.xpu.synchronize()

    def _setup_camera(self, fps: int) -> None:
        # depthai v2/v3 겸용 (2026-08-11): 산업용 PC는 depthai 3.x라 v2 API
        # (createColorCamera/XLinkOut)가 없어 기동이 즉사했다 — 첫 통합 run에서
        # 차선 주행이 통째로 빠진 원인. 해상도·소켓은 양쪽 동일(1280x720/CAM_A).
        import depthai as dai

        mxid = str(self.get_parameter('camera_mxid').value).strip()
        # USB 링크 속도 상한 (usb_speed 파라미터 주석 참조 — GPS 간섭 대책)
        speed_name = str(self.get_parameter('usb_speed').value).strip().upper()
        max_speed = None
        if speed_name and speed_name != 'SUPER':
            if hasattr(dai, 'UsbSpeed') and hasattr(dai.UsbSpeed, speed_name):
                max_speed = getattr(dai.UsbSpeed, speed_name)
                self.get_logger().warn(
                    f'USB 링크 속도 제한: {speed_name} — GPS 간섭 대책. '
                    'camera_fps가 10 이하인지 확인할 것 (대역폭 부족 시 프레임 유실)')
            else:
                self.get_logger().error(
                    f'usb_speed={speed_name} 인식 불가 — 제한 없이 진행')
        if mxid:
            self.get_logger().info(f'OAK-D MxID 핀닝: {mxid}')
        else:
            self.get_logger().warn(
                'camera_mxid 미지정 — 첫 가용 OAK-D 사용. 카메라 2대 연결 상태에선 '
                '부팅 순서에 따라 신호등용 카메라를 잡을 수 있음 (CLAUDE.md §6)')

        def open_device(factory):
            # 직전 프로세스 강제 종료 직후엔 카메라가 USB 리셋 중이라 첫 오픈이
            # 실패할 수 있다 (2026-08-11 실측: Couldn't open stream /
            # X_LINK_DEVICE_NOT_FOUND 후 15s 뒤 정상). 재시도로 흡수.
            import time
            for attempt in range(4):
                try:
                    return factory()
                except RuntimeError as e:
                    if attempt == 3:
                        raise
                    self.get_logger().warn(
                        f'카메라 오픈 실패({attempt + 1}/4): {e} — 8s 후 재시도')
                    time.sleep(8.0)

        # ⚠ dai.Pipeline() 을 '인스턴스로' 만들어 판별하면 안 된다 — depthai v3 에서는
        # 인자 없는 Pipeline() 이 **기본 장치를 암묵적으로 연다**(2026-08-25 실측:
        # getDefaultDevice() 가 Device 반환). MxID 핀닝을 무시하고 아무 카메라나
        # 잡아 버리고, 이어지는 dai.Device(DeviceInfo(mxid)) 가 점유 충돌로 실패해
        # open_device 의 8s×4 재시도(최대 24s)에 걸린다 — 기동이 멈춘 것처럼 보인다.
        # 클래스 속성으로 보면 하드웨어를 건드리지 않는다 (v2 True / v3 False 확인).
        if hasattr(dai.Pipeline, 'createColorCamera'):  # v2
            pipeline = dai.Pipeline()
            cam = pipeline.createColorCamera()
            cam.setPreviewSize(1280, 720)
            cam.setInterleaved(False)
            cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
            cam.setFps(fps)
            xout = pipeline.createXLinkOut()
            xout.setStreamName('rgb')
            cam.preview.link(xout.input)
            self._dai_device = open_device(
                lambda: dai.Device(pipeline, dai.DeviceInfo(mxid), max_speed)
                if mxid and max_speed is not None else
                dai.Device(pipeline, dai.DeviceInfo(mxid)) if mxid else
                dai.Device(pipeline, max_speed) if max_speed is not None else
                dai.Device(pipeline))
            self._queue = self._dai_device.getOutputQueue('rgb', maxSize=4, blocking=False)
            self._dai_pipeline = None
        else:  # v3
            self._dai_device = open_device(
                lambda: dai.Device(dai.DeviceInfo(mxid), max_speed)
                if mxid and max_speed is not None else
                dai.Device(dai.DeviceInfo(mxid)) if mxid else
                dai.Device(max_speed) if max_speed is not None else
                dai.Device())
            pipeline = dai.Pipeline(self._dai_device)
            cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            self._queue = cam.requestOutput(
                (1280, 720), dai.ImgFrame.Type.BGR888i, fps=float(fps)
            ).createOutputQueue(maxSize=4, blocking=False)
            pipeline.start()
            self._dai_pipeline = pipeline

    def _capture_monotonic(self, pkt) -> float | None:
        """프레임 캡처 시각 [s, CLOCK_MONOTONIC]. 못 얻으면 None.

        이걸로 header.stamp를 캡처 시각까지 되돌린다 — 종전에는 발행 시각을 찍어
        **카메라 파이프라인 지연(캡처→추론→발행)이 소비 측에서 보이지 않았다.**
        2026-08-15 차선 위빙 분석에서 필요해진 값: 이득을 낮춰도(ref[0] 2.5m)
        진폭이 15초 창마다 +1.12°씩 계속 자라 지연이 남은 용의자인데, 정작
        지연을 측정할 수가 없었다(수신−발행 = 0.000s로만 보임).

        주의: MGM의 신선도 watchdog(§5.7)은 **수신 시각**(monotonicNs)을 쓰므로
        이 변경의 영향을 받지 않는다. 확인 완료 — lane_path의 header.stamp를
        읽는 소비자는 현재 없다.
        """
        try:
            return pkt.getTimestamp().total_seconds()
        except Exception as exc:   # depthai 버전차·타임스탬프 미지원 대비
            if not self._cap_ts_warned:
                self._cap_ts_warned = True
                self.get_logger().warn(
                    f'프레임 캡처 시각을 못 읽음({exc}) — header.stamp를 발행 시각으로 '
                    '폴백한다. 파이프라인 지연 측정 불가.')
            return None

    def tick(self) -> None:
        pkt = self._queue.tryGet()
        if pkt is None:
            return  # 아직 새 프레임 없음

        # 큐에 쌓인 프레임은 버리고 **최신 것만** 쓴다 — 방어책 (2026-08-15).
        # createOutputQueue(maxSize=4)는 FIFO라 tryGet()이 **가장 오래된** 프레임을
        # 준다. 소비가 밀리면 그 뒤로 계속 낡은 프레임을 처리하고 최대 4프레임
        # (10fps에서 400ms)까지 지연이 눌어붙을 수 있다.
        # ★ 실측상 현재는 백로그가 없다 — 폐기 누계가 기동 시 3개뿐이고 지연도
        #   150~160ms로 변화 없었다. 즉 이 변경은 **지연을 줄이지 못했다.**
        #   주행 부하로 소비가 밀릴 때를 대비한 안전장치로만 남긴다.
        #   실제 지연의 정체는 전송이다: 추론 30ms + USB2 전송 69ms
        #   (720p BGR888i 2.76MB ÷ ~40MB/s) + 노출·ISP. 해상도나 전송 포맷을
        #   바꾸지 않으면 못 줄인다 (해상도 하향·USB3는 검출 품질·GPS 간섭 때문에
        #   선택지가 아니다 — 팀장 확정).
        while True:
            newer = self._queue.tryGet()
            if newer is None:
                break
            pkt = newer
            self._frames_dropped += 1

        self._frames_seen += 1
        if self._frames_seen <= self.warmup_frames:
            return  # 노출 적응 대기 중 — 오검출 위험 있는 콜드스타트 프레임 스킵

        # 프레임 **캡처 시각**을 붙잡아 둔다 (아래 header.stamp 용).
        # depthai의 getTimestamp()는 호스트 steady_clock(=CLOCK_MONOTONIC) 기준이라
        # time.monotonic()과 기준선이 같다 — 두 값의 **차이**만 쓰므로 ROS 시각과의
        # 에폭 오프셋을 알 필요가 없다.
        cap_mono = self._capture_monotonic(pkt)

        frame = pkt.getCvFrame()
        canvas, tensor = preprocess(frame, self.img_size, self.device, self.half)
        t0 = self.get_clock().now()
        _da_mask, ll_mask = infer(self.model, tensor)
        infer_ms = (self.get_clock().now() - t0).nanoseconds / 1e6
        # SEARCH 중엔 연속성 체크 허용폭을 서서히 넓힘 — 급격한 단일 리셋 지점 없이
        # 재검출이 자연스럽게 받아들여지게 함(§7 docstring 참조).
        if self._track_status == 'search':
            age_in_search = max(0, self._age_frames - self.hold_frames)
            widen = 1.0 + min(1.0, age_in_search / max(1, self.search_frames)) * (self.search_max_jump_widen - 1.0)
            effective_max_jump = self.max_y_jump_m * widen
        else:
            effective_max_jump = self.max_y_jump_m

        prev_y = self._prev_y if self._track_status != 'lost' else None
        prev_coeffs = self._prev_coeffs if self._track_status != 'lost' else None
        estimate, debug = estimate_lane_path(
            ll_mask.astype(np.uint8), self.H, self.grid, lookahead_m=self.lookahead_m,
            n_points=self.n_points, points_x_start=self.points_x_start, points_x_end=self.points_x_end,
            ref_point0_lookahead_m=self.ref_point0_lookahead_m,
            ref_point0_extrap_mode=self.ref_point0_extrap_mode,
            ref_point0_min_confidence=self.ref_point0_min_confidence,
            ref_point0_c0_safe_m=self.ref_point0_c0_safe_m,
            ref_point0_c0_unsafe_m=self.ref_point0_c0_unsafe_m,
            ref_point0_min_turn_radius_m=self.ref_point0_min_turn_radius_m,
            prev_y=prev_y, max_y_jump_m=effective_max_jump,
            prev_coeffs=prev_coeffs, coeff_smoothing_alpha=self.coeff_smoothing_alpha)

        if estimate.mode != 'none':
            # 정상 검출 -> VALID. 다음 HELD/SEARCH에 쓸 수 있게 이 값을 저장해둠.
            self._track_status = 'valid'
            self._age_frames = 0
            self._prev_y = debug.get('raw_y', estimate.y)  # 연속성 체크는 항상 raw 기준
            self._prev_coeffs = debug.get('smoothed_coeffs')
            self._held_estimate = estimate
            final_estimate = estimate
        else:
            self._age_frames += 1
            if self._track_status == 'valid':
                self._track_status = 'held' if self._held_estimate is not None else 'lost'
            if self._track_status == 'held' and self._age_frames > self.hold_frames:
                self._track_status = 'search'
            if self._track_status == 'search' and self._age_frames > self.hold_frames + self.search_frames:
                self._track_status = 'lost'

            if self._track_status == 'held' and self._held_estimate is not None:
                decay = self.hold_confidence_decay ** self._age_frames
                final_estimate = replace(self._held_estimate,
                                          confidence=self._held_estimate.confidence * decay,
                                          reject_reason='held')
            elif self._track_status == 'search' and self._held_estimate is not None:
                final_estimate = replace(self._held_estimate,
                                          confidence=self.search_min_confidence,
                                          reject_reason='search')
            else:  # lost — 완전 리셋, 다음 프레임은 검사 없이 수용
                self._prev_y = None
                self._prev_coeffs = None
                self._held_estimate = None
                final_estimate = estimate

        if self.logger_csv is not None:
            self.logger_csv.log(infer_ms=infer_ms, estimate=final_estimate, fit_result=debug['fit'],
                                 raw_y=debug.get('raw_y'), reject_streak=self._age_frames)

        if self.debug_pub is not None:
            frame_vis = build_debug_frame(
                canvas, ll_mask, final_estimate, debug, self.grid, self.lookahead_m,
                self.H_inv, infer_ms, self.is_placeholder)
            img_msg = self.bridge.cv2_to_imgmsg(frame_vis, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.debug_pub.publish(img_msg)

        # header.stamp = **프레임 캡처 시각** (발행 시각이 아니다).
        # 지금(now) − 캡처(cap_mono) 를 같은 monotonic 기준에서 뺀 뒤 ROS now에서
        # 되돌린다. 추론이 끝난 뒤 계산하므로 캡처→추론→발행 전 구간이 들어간다.
        now = self.get_clock().now()
        pipeline_s = 0.0
        if cap_mono is not None:
            age = time.monotonic() - cap_mono
            if 0.0 <= age < 1.0:          # 이상치(시계 점프·재연결)는 무시하고 발행 시각 사용
                pipeline_s = age
            elif not self._cap_ts_warned:
                self._cap_ts_warned = True
                self.get_logger().warn(
                    f'프레임 지연이 비정상({age:.3f}s) — header.stamp를 발행 시각으로 폴백')
        msg = LanePath()
        msg.header.stamp = (now - Duration(seconds=pipeline_s)).to_msg()
        msg.header.frame_id = 'base_link'
        msg.confidence = float(final_estimate.confidence)
        points = []
        for p in final_estimate.points:
            rp = RefPoint()
            rp.x = float(p.x)
            rp.y = float(p.y)
            rp.yaw = float(p.yaw)
            rp.curvature = float(p.curvature)
            points.append(rp)
        msg.points = points
        self.pub.publish(msg)

        # 캡처→발행 지연 주기 로깅 (5초). 차선 추종 루프의 위상 여유를 좌우하는
        # 값이라 현장에서 바로 보여야 한다 — 2026-08-15 위빙 분석에서 이 값이
        # 없어 지연 가설을 확정 못 했다.
        if pipeline_s > 0.0:
            self._pipeline_ms.append(pipeline_s * 1e3)
            if len(self._pipeline_ms) >= 50:
                v = sorted(self._pipeline_ms)
                self.get_logger().info(
                    '카메라 파이프라인 지연[ms] 중앙값 %.0f  p90 %.0f  최대 %.0f '
                    ' (추론 %.0f, 큐 폐기 누계 %d)'
                    % (v[len(v) // 2], v[int(0.9 * len(v))], v[-1],
                       infer_ms, self._frames_dropped))
                self._pipeline_ms.clear()

    def destroy_node(self) -> None:
        if self.logger_csv is not None:
            self.logger_csv.close()
        if getattr(self, '_dai_pipeline', None) is not None:  # v3
            self._dai_pipeline.stop()
        if hasattr(self, '_dai_device'):
            self._dai_device.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StackLaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
