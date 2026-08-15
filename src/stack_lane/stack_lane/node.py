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
  차량 기본 10(=100ms)이다. USB 링크는 `usb_speed=high`로 USB2를 강제하며,
  요청/실제 링크가 HIGH인지와 지정 MxID가 일치하는지 확인한 뒤에만 시작한다.
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

from dataclasses import replace

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
from stack_lane.oak_camera import (
    LANE_CAMERA_HEIGHT,
    LANE_CAMERA_WIDTH,
    OakRgbCamera,
    normalize_usb_speed,
    validate_camera_profile,
)
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
        # 차량 기본은 USB2(HIGH)다. high/super 이외 값이나 실제 HIGH 불일치는
        # 카메라를 열어 둔 채 계속하지 않고 즉시 실패한다.
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
        self.camera_mxid = str(
            self.get_parameter('camera_mxid').value
        ).strip()
        self.camera_fps = int(self.get_parameter('camera_fps').value)
        self.camera_usb_speed = normalize_usb_speed(
            self.get_parameter('usb_speed').value
        )
        validate_camera_profile(
            width=LANE_CAMERA_WIDTH,
            height=LANE_CAMERA_HEIGHT,
            fps=self.camera_fps,
            usb_speed=self.camera_usb_speed,
        )

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

        self._setup_camera()

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
        elif self.device.type == 'xpu':
            torch.xpu.synchronize()

    def _setup_camera(self) -> None:
        if self.camera_mxid:
            self.get_logger().info(
                f'OAK-D MxID 핀닝: {self.camera_mxid}'
            )
        else:
            self.get_logger().warn(
                'camera_mxid 미지정 — OAK가 정확히 한 대일 때만 허용합니다.'
            )

        self._oak_camera = OakRgbCamera(
            width=LANE_CAMERA_WIDTH,
            height=LANE_CAMERA_HEIGHT,
            fps=self.camera_fps,
            mxid=self.camera_mxid,
            usb_speed=self.camera_usb_speed,
            warn=self.get_logger().warn,
        )
        self._queue = self._oak_camera.queue
        self.get_logger().info(
            'OAK-D lane camera ready: '
            f'api=v{self._oak_camera.api_major} '
            f'mxid={self._oak_camera.mxid} '
            f'{LANE_CAMERA_WIDTH}x{LANE_CAMERA_HEIGHT}@'
            f'{self.camera_fps} '
            f'usb_requested={self.camera_usb_speed.upper()} '
            f'usb_actual={self._oak_camera.usb_speed}'
        )

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

        msg = LanePath()
        msg.header.stamp = self.get_clock().now().to_msg()
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

    def destroy_node(self) -> None:
        if self.logger_csv is not None:
            self.logger_csv.close()
        oak_camera = getattr(self, '_oak_camera', None)
        if oak_camera is not None:
            oak_camera.release()
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
