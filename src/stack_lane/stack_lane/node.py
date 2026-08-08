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

import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
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
        # REF_POINT_00 근거리 치환 실험 (2026-08-08, 조향 게인 진단 — lane_path.py
        # estimate_lane_path() docstring 참조). 기본값(0.0)은 비활성 = 기존 동작 그대로.
        self.declare_parameter('ref_point0_lookahead_m', 0.0)
        self.declare_parameter('ref_point0_extrap_mode', 'quadratic')  # 'linear' | 'quadratic'
        self.declare_parameter('ref_point0_min_confidence', 0.5)
        # 프레임 간 연속성 체크 (2026-08-08, 편측 오검출 진단 — lane_path.py
        # estimate_lane_path()의 prev_y/max_y_jump_m 참조).
        self.declare_parameter('max_y_jump_m', 1.0)
        self.declare_parameter('y_jump_reset_after', 15)  # 연속 이만큼 거부되면 리셋(고착 방지)
        # 다항식 계수 EMA 저역통과 필터 (2026-08-08, 오실레이션 완화 — lane_path.py
        # estimate_lane_path()의 prev_coeffs/coeff_smoothing_alpha 참조).
        # 1.0 = 비활성(기존 동작). 작을수록 부드럽지만 반응이 느려짐 — 실측 튜닝 필요.
        self.declare_parameter('coeff_smoothing_alpha', 1.0)
        self.declare_parameter('homography_path', str(DEFAULT_HOMOGRAPHY_PATH))
        self.declare_parameter('camera_fps', 30)
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
        self.max_y_jump_m = float(self.get_parameter('max_y_jump_m').value)
        self.y_jump_reset_after = int(self.get_parameter('y_jump_reset_after').value)
        self.coeff_smoothing_alpha = float(self.get_parameter('coeff_smoothing_alpha').value)
        self._prev_y = None  # 연속성 체크용 — 마지막으로 채택(mode!=none)됐던 raw y
        self._prev_coeffs = None  # 스무딩용 — 직전 프레임의 smoothed 계수
        self._reject_streak = 0
        self.warmup_frames = int(self.get_parameter('warmup_frames').value)
        self._frames_seen = 0

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
            f'stack_lane_node 준비됨 (ref_point0_lookahead_m={self.ref_point0_lookahead_m}, '
            f'extrap_mode={self.ref_point0_extrap_mode}, '
            f'min_confidence={self.ref_point0_min_confidence}) — CSV의 ref_point0_applied/'
            f'ref_point0_x와 대조해 사후 분석할 것')

    def _warmup_model(self) -> None:
        dummy = torch.zeros(1, 3, self.img_size, self.img_size, device=self.device)
        dummy = dummy.half() if self.half else dummy.float()
        with torch.no_grad():
            self.model(dummy)
        if self.device.type == 'cuda':
            torch.cuda.synchronize()

    def _setup_camera(self, fps: int) -> None:
        import depthai as dai

        pipeline = dai.Pipeline()
        cam = pipeline.createColorCamera()
        cam.setPreviewSize(1280, 720)
        cam.setInterleaved(False)
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setFps(fps)
        xout = pipeline.createXLinkOut()
        xout.setStreamName('rgb')
        cam.preview.link(xout.input)

        self._dai_device = dai.Device(pipeline)
        self._queue = self._dai_device.getOutputQueue('rgb', maxSize=4, blocking=False)

    def tick(self) -> None:
        pkt = self._queue.tryGet()
        if pkt is None:
            return  # 아직 새 프레임 없음

        self._frames_seen += 1
        if self._frames_seen <= self.warmup_frames:
            return  # 노출 적응 대기 중 — 오검출 위험 있는 콜드스타트 프레임 스킵

        frame = pkt.getCvFrame()
        canvas, tensor = preprocess(frame, self.img_size, self.device, self.half)
        t0 = self.get_clock().now()
        _da_mask, ll_mask = infer(self.model, tensor)
        infer_ms = (self.get_clock().now() - t0).nanoseconds / 1e6
        # 연속 거부가 너무 오래 지속되면(오검출이 아니라 실제로 차가 이동해서
        # 직전 기준이 낡은 것일 수 있음) 리셋 — 영구 고착 방지. 스무딩 기준도 같이 리셋
        # (낡은 계수에 새 값을 섞으면 안 되므로).
        reset = self._reject_streak >= self.y_jump_reset_after
        prev_y = None if reset else self._prev_y
        prev_coeffs = None if reset else self._prev_coeffs
        estimate, debug = estimate_lane_path(
            ll_mask.astype(np.uint8), self.H, self.grid, lookahead_m=self.lookahead_m,
            n_points=self.n_points, points_x_start=self.points_x_start, points_x_end=self.points_x_end,
            ref_point0_lookahead_m=self.ref_point0_lookahead_m,
            ref_point0_extrap_mode=self.ref_point0_extrap_mode,
            ref_point0_min_confidence=self.ref_point0_min_confidence,
            prev_y=prev_y, max_y_jump_m=self.max_y_jump_m,
            prev_coeffs=prev_coeffs, coeff_smoothing_alpha=self.coeff_smoothing_alpha)

        if estimate.mode == 'none':
            self._reject_streak += 1
        else:
            self._reject_streak = 0
            self._prev_y = debug.get('raw_y', estimate.y)  # 연속성 체크는 항상 raw 기준
            self._prev_coeffs = debug.get('smoothed_coeffs')

        if self.logger_csv is not None:
            self.logger_csv.log(infer_ms=infer_ms, estimate=estimate, fit_result=debug['fit'],
                                 raw_y=debug.get('raw_y'))

        if self.debug_pub is not None:
            frame_vis = build_debug_frame(
                canvas, ll_mask, estimate, debug, self.grid, self.lookahead_m,
                self.H_inv, infer_ms, self.is_placeholder)
            img_msg = self.bridge.cv2_to_imgmsg(frame_vis, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.debug_pub.publish(img_msg)

        msg = LanePath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.confidence = float(estimate.confidence)
        points = []
        for p in estimate.points:
            rp = RefPoint()
            rp.x = float(p.x)
            rp.y = float(p.y)
            rp.yaw = float(p.yaw)
            rp.curvature = float(p.curvature)
            points.append(rp)
        msg.points = points
        self.pub.publish(msg)

    def destroy_node(self) -> None:
        if self.logger_csv is not None:
            self.logger_csv.close()
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
