"""stack_parking 스켈레톤 — 주차공간 인식·로컬맵·주차 경로 + MPC/Vehicle MGM(dSPACE)
담당: 손상민

지금은 중립 placeholder를 퍼블리시한다 (MGM 파이프라인 관통용).
실제 인지 로직으로 교체할 것. 출력 계약은 REQUIREMENTS.md 참조.

주의: 이 노드는 MGM 10ms 루프와 별도 프로세스다 (CLAUDE.md §5.2).
무거운 처리(YOLO 등)가 이 노드 안에서 얼마나 걸리든 MGM은 최신 스냅샷을 pull한다.
"""
import rclpy
from rclpy.node import Node

from fma_interfaces.msg import ParkingStatus


class StackParkingNode(Node):

    def __init__(self):
        super().__init__('stack_parking_node')
        self.pub = self.create_publisher(ParkingStatus, '/perception/parking', 1)
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        msg = ParkingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.space_found = False   # TODO: LiDAR 로컬맵에서 주차공간 인식
        msg.path_blocked = False  # TODO: 동적 침범만 True (콘·연석은 로컬맵 입력일 뿐)
        msg.done = False          # TODO: 주차 완료 판정
        msg.v_suggest = 0.0       # TODO: 주차 진행 권장 속도 (후진 음수)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StackParkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
