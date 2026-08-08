#!/usr/bin/env python3
"""테스트 전용 더미 estop 클리어 퍼블리셔 — 실제 장애물 감지 로직 없음.

stack_estop(라이다 기반, 박찬미)이 아직 연결 안 된 상태에서 stack_lane 단독
통합 테스트(카메라만으로 ref_point 추종 확인)를 하기 위한 임시 도구.
/perception/estop에 estop=false를 주기적으로 발행해 adas_mgm의 estop 신선도
watchdog(mgm_node.cpp: 미수신 시 estop=true 강제)만 만족시킨다.

⚠️ 이 스크립트는 실제 장애물을 전혀 감지하지 않는다. 물리 E-stop 버튼이
항상 즉시 사용 가능한 상태에서만, 초저속 통제된 테스트에만 사용할 것.
실차 배포에는 stack_estop_node(실제 라이다)로 반드시 교체해야 한다.
"""
import rclpy
from rclpy.node import Node

from fma_interfaces.msg import EstopRequest


class DummyEstopClear(Node):

    def __init__(self):
        super().__init__('dummy_estop_clear')
        self.pub = self.create_publisher(EstopRequest, '/perception/estop', 1)
        self.timer = self.create_timer(0.05, self.tick)  # stack_estop 실제 주기(50ms)와 동일
        self.get_logger().warn(
            '더미 estop 클리어 발행 중 — 실제 장애물 감지 없음. '
            '물리 E-stop 대기 상태에서만 사용할 것.')

    def tick(self):
        msg = EstopRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.estop = False
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyEstopClear()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
