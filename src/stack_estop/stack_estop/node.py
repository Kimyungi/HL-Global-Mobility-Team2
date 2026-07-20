"""stack_estop 스켈레톤 — 돌발 장애물 감지 → 긴급 정지 요구
담당: 박찬미

지금은 중립 placeholder를 퍼블리시한다 (MGM 파이프라인 관통용).
실제 인지 로직으로 교체할 것. 출력 계약은 REQUIREMENTS.md 참조.

주의: 이 노드는 MGM 10ms 루프와 별도 프로세스다 (CLAUDE.md §5.2).
무거운 처리(YOLO 등)가 이 노드 안에서 얼마나 걸리든 MGM은 최신 스냅샷을 pull한다.
"""
import rclpy
from rclpy.node import Node

from fma_interfaces.msg import EstopRequest


class StackEstopNode(Node):

    def __init__(self):
        super().__init__('stack_estop_node')
        self.pub = self.create_publisher(EstopRequest, '/perception/estop', 1)
        self.timer = self.create_timer(0.05, self.tick)

    def tick(self):
        msg = EstopRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.estop = False  # TODO: 돌발 장애물 (전방 근접 급출현)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StackEstopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
