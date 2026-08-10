#!/usr/bin/env python3
"""실차 측정용 스텝 주입기 — 측방 목표 오프셋을 계단으로 주고 응답을 재게 한다.  이기돈

stage2 실측 ①②의 입력 발생기. 회피 로직을 거치지 않고 /adas/target_ref를 직접 내되,
**avoid_to_ref와 같은 코드로 ref point를 만든다** — 공용 `ref_points` 모듈을 두 노드가
함께 import 한다. 그래야 여기서 잰 지연·이동곡선이 실제 회피 기동에 그대로 적용된다.
(2026-08-10 이전에는 각자 구현이었고, step 쪽이 이미 삭제된 구방식을 복제하고 있었다.)

  ① 조향 응답 시간   : 스탠드(바퀴 듦)에서 실행 → ref y 스텝 → VehicleVector.str 응답
  ② 측방 이동 곡선   : 지상에서 v=0.3/0.5로 실행 → 스텝 후 |y| 0.30·0.46m 도달까지 전진거리

★ v_ref=0으로는 측정할 수 없다. dSPACE MPC 지평 = 0.2 × v_ref 이므로 v_ref=0이면
  지평이 0으로 붕괴해 조향이 반응하지 않는다. 스탠드 시험도 v_ref를 줘서(바퀴가 돌지만
  차는 안 움직임) 해야 한다 — ①을 스탠드에서 하는 이유가 바로 이것.

시퀀스: [정렬(y=0, settle_s)] → [오프셋 유지(hold_s)] → ... offsets 순회, repeats회 반복.
각 전이마다 /test/event(String)를 발행해 bag에서 구간을 잘라낼 수 있게 한다.
시퀀스가 끝나면 v_ref=0으로 자동 정지한다.

  ros2 run stack_avoid step_injector --ros-args \
      -p v_ref:=0.3 -p offsets:="[0.46, -0.46, 0.30, -0.30]" -p hold_s:=3.0 -p repeats:=3

★ estop 게이트 상시 적용 (estop_gate.EstopGate) — 실차를 움직이므로 예외 없음.
★ 안전: 스탠드 먼저, 물리 비상정지 준비, 지상 주행은 통제된 직선 구간에서.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fma_interfaces.msg import RefPoint, TargetRef

from stack_avoid.estop_gate import EstopGate
from stack_avoid.ref_points import lateral_step_point


class StepInjector(Node):
    """측방 오프셋 계단 입력 발생기 + 구간 이벤트 발행."""

    def __init__(self):
        """파라미터를 읽고 시퀀스를 조립한 뒤 주기 발행을 시작한다."""
        super().__init__('step_injector')
        self.v_ref = float(self.declare_parameter('v_ref', 0.3).value)
        # 시험할 측방 오프셋 [m]. 0.46 = 통로 반폭(차폭/2+여유), 0.30 = estop 통로 반폭.
        self.offsets = [float(v) for v in
                        self.declare_parameter('offsets', [0.46, -0.46, 0.30, -0.30]).value]
        self.hold_s = float(self.declare_parameter('hold_s', 3.0).value)
        self.settle_s = float(self.declare_parameter('settle_s', 3.0).value)
        self.repeats = int(self.declare_parameter('repeats', 3).value)
        # 가상 회피 목표점의 전방거리 [m] — avoid_to_ref가 쓰는 obs_x 자리.
        self.target_x = float(self.declare_parameter('target_x_m', 1.5).value)
        # ★ 아래 3개는 avoid_to_ref 와 **같은 값**이어야 여기서 잰 응답이 실제 회피에
        #   그대로 이전된다 (팀장 리뷰 2026-08-10 ⑦). 기본값을 avoid_to_ref 의
        #   ray_pull 규약에 맞춘다: ref_lookahead_m 0.90 / min_turn_radius 1.15 /
        #   ref_x_min 0.8. 예전 lookahead 기본 0.4 는 이미 삭제된 구방식의 값이라,
        #   그대로 두면 스텝 시험이 실제 회피와 다른 점을 보내게 된다.
        self.lookahead = float(self.declare_parameter('lookahead_m', 0.90).value)
        self.min_turn_radius = float(self.declare_parameter('min_turn_radius_m', 1.15).value)
        self.ref_x_min = float(self.declare_parameter('ref_x_min_m', 0.8).value)
        self.curv_gain = float(self.declare_parameter('curvature_gain', 1.0).value)
        self.period = float(self.declare_parameter('period_ms', 10).value) / 1000.0
        # 폭주 방지 상한 — 시퀀스 계산치와 무관하게 이 시간이 지나면 정지.
        self.max_run_s = float(self.declare_parameter('max_run_s', 600.0).value)

        self.gate = EstopGate(
            self,
            enabled=bool(self.declare_parameter('estop_gate', True).value),
            stale_s=float(self.declare_parameter('estop_stale_s', 0.25).value))

        # 시퀀스: (오프셋, 유지시간, 이벤트라벨) 목록. 정렬 구간을 매 스텝 앞에 둔다.
        self.seq = []
        for rep in range(self.repeats):
            for off in self.offsets:
                self.seq.append((0.0, self.settle_s, f'settle r{rep + 1}'))
                self.seq.append((off, self.hold_s, f'step {off:+.2f} r{rep + 1}'))
        self.seq.append((0.0, self.settle_s, 'settle final'))

        self.idx = 0
        self.phase_start = None
        self.t_start = None
        self.done = False

        self.pub = self.create_publisher(TargetRef, '/adas/target_ref', 1)
        self.event_pub = self.create_publisher(String, '/test/event', 10)
        self.timer = self.create_timer(self.period, self.tick)

        total = sum(d for _, d, _ in self.seq)
        self.get_logger().warn(
            f'step_injector: 측방 스텝 {self.offsets} × {self.repeats}회, '
            f'유지 {self.hold_s}s / 정렬 {self.settle_s}s, 총 {total:.0f}s | '
            f'v_ref={self.v_ref}m/s | {self.gate.banner()}')
        if self.v_ref <= 1e-3:
            self.get_logger().error(
                'v_ref=0 — MPC 지평(0.2×v_ref)이 0이라 조향이 반응하지 않는다. 측정 불가.')

    def _event(self, label):
        m = String()
        m.data = label
        self.event_pub.publish(m)
        self.get_logger().info(f'▶ {label}')

    def _rp(self, x, y=0.0, yaw=0.0, curv=0.0):
        p = RefPoint()
        p.x, p.y, p.yaw, p.curvature = float(x), float(y), float(yaw), float(curv)
        return p

    def _lookahead_point(self, ty):
        """목표 (target_x, ty) 를 **회피와 같은 생성기**로 ref 점 하나로.

        ★ 2026-08-10 (팀장 리뷰 ⑦). 예전에는 "avoid_to_ref 와 같은 계산"이라며 목표점
          까지의 등곡률 호 위 lookahead 점을 직접 구현했는데, 그 방식은 avoid_to_ref
          에서 이미 삭제된 구방식이었다. 현재 기본은 ray_pull — **방향 보존 반직선**
          이고 yaw 는 0 이다. 두 계산은 같은 목표에 대해 다른 점을 낸다.
          그 상태로 잰 ①(조향 응답)·②(측방 이동 곡선)은 실제 회피에 이전되지 않는다.
          → 공용 `ref_points.lateral_step_point` 하나만 부른다. 이제 스텝 시험이 내는
            ref 는 실제 회피가 내는 ref 와 **같은 코드**를 거친다.
        """
        pts = lateral_step_point(
            ty, target_x_m=self.target_x,
            lookahead_m=self.lookahead, n_points=1,
            min_turn_radius_m=self.min_turn_radius,
            ref_x_min_m=self.ref_x_min, curv_gain=self.curv_gain)
        return self._rp(*pts[0])

    def tick(self):
        """현재 시퀀스 단계의 오프셋을 ref로 내보낸다. estop이면 v_ref=0."""
        now = self.get_clock().now()
        if self.t_start is None:
            # ★ 첫 라벨은 /test/event 구독자(bag 기록기)가 붙은 뒤에만 낸다 — 매칭 전에
            # 내면 첫 구간 라벨이 통째로 유실된다 (8/7 실차 스윕에서 실제 발생, gain_sweep
            # 과 동일 처리. PR #27 리뷰 반영). 구독자가 없으면 스텝 시작도 미룬다.
            if self.event_pub.get_subscription_count() == 0:
                self.get_logger().warning(
                    '/test/event 구독자 대기 중 — bag 기록이 떠야 스텝을 시작한다 '
                    '(라벨 유실 방지). 구독자가 없으면 10s 후 자동 진행.', once=True)
                # ★ if/elif 로 쓰면 첫 틱에 elif 가 평가되지 않아 그대로 아래로
                #   떨어진다 — 가드가 no-op 이 되고 10s 분기는 도달 불가 데드코드가
                #   된다(팀장 리뷰 2026-08-10 ①). 대기 여부를 한 조건으로 판정한다.
                if not hasattr(self, '_wait_since'):
                    self._wait_since = now
                if (now - self._wait_since).nanoseconds * 1e-9 < 10.0:
                    return                      # 구독자 붙을 때까지 시작 자체를 미룬다
                self.get_logger().warning(
                    '/test/event 구독자 없이 10s 경과 — 라벨 유실을 감수하고 진행한다')
            self.t_start = now
            self.phase_start = now
            self._event(f'RUN start v_ref={self.v_ref}')
            self._event(self.seq[0][2])

        elapsed = (now - self.t_start).nanoseconds * 1e-9
        if not self.done and elapsed > self.max_run_s:
            self.done = True
            self._event('RUN abort (max_run_s)')
            self.get_logger().error('max_run_s 초과 — 정지')

        m = TargetRef()
        m.header.stamp = now.to_msg()
        m.header.frame_id = 'base_link'
        m.state = TargetRef.STATE_AVOID

        if self.done:
            m.v_ref = 0.0
            m.ref_points = [self._rp(self.target_x)]
            self.pub.publish(m)
            return

        offset, hold, _ = self.seq[self.idx]
        if (now - self.phase_start).nanoseconds * 1e-9 >= hold:
            self.idx += 1
            self.phase_start = now
            if self.idx >= len(self.seq):
                self.done = True
                self._event('RUN done')
                self.get_logger().warn('시퀀스 완료 — v_ref=0 정지. Ctrl+C로 종료.')
                m.v_ref = 0.0
                m.ref_points = [self._rp(self.target_x)]
                self.pub.publish(m)
                return
            offset, hold, label = self.seq[self.idx]
            self._event(label)

        m.v_ref = self.v_ref
        m.ref_points = [self._lookahead_point(offset)]

        # estop 게이트 — 최상위 우선권. ref_points는 유지(§3 급조향 금지).
        blocked, why = self.gate.block()
        if blocked:
            m.v_ref = 0.0
        self.gate.log_reason(why)

        self.pub.publish(m)


def main(args=None):
    """노드 진입점."""
    rclpy.init(args=args)
    node = StepInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
