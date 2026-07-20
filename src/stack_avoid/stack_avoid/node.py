"""stack_avoid 스켈레톤 — 장애물 인지, 회피 가능 판정 재료(TTC·측방), 회피 경로
담당: 이기돈

지금은 중립 placeholder를 퍼블리시한다 (MGM 파이프라인 관통용).
실제 인지 로직으로 교체할 것. 출력 계약은 REQUIREMENTS.md 참조.

주의: 이 노드는 MGM 10ms 루프와 별도 프로세스다 (CLAUDE.md §5.2).
무거운 처리(YOLO 등)가 이 노드 안에서 얼마나 걸리든 MGM은 최신 스냅샷을 pull한다.
"""
import rclpy
from rclpy.node import Node

from fma_interfaces.msg import AvoidStatus


class StackAvoidNode(Node):

    def __init__(self):
        super().__init__('stack_avoid_node')
        self.pub = self.create_publisher(AvoidStatus, '/perception/avoid', 1)
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        msg = AvoidStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.obstacle_detected = False
        msg.avoidable = False     # TODO: TTC·측방 여유 충분 여부
        msg.ttc = 1.0e9           # TODO: [s] 장애물 없으면 큰 값 유지 (0 금지!)
        msg.narrow_gap = False    # TODO: 여유 폭 좁음 → MGM이 감속 근거로 사용
        msg.maneuver_done = False # TODO: 회피 기동 완료 → MGM 복귀 트리거
        msg.v_suggest = 0.0       # TODO: 회피 기하 기반 권장 속도
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StackAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
