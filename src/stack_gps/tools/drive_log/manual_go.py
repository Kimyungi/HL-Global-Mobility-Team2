#!/usr/bin/env python3
"""수동 출발/정지 레버 — GPS 단독 주행용 (라이다·stack_estop 미사용 구성).

estop 해제(false) 하트비트를 50ms 주기로 발행해 MGM의 출발 조건(§5.7 estop
신선도 워치독)을 채운다. 즉:

  실행       = 출발 (다른 정지 요구가 없는 한)
  Ctrl-C     = 발행 중단 → 250ms 내 MGM이 정지 (소프트웨어 정지 레버)

⚠⚠ 돌발 장애물 자동 정지 없음 — 이 도구를 쓰는 동안 유일한 비상 수단은
   물리 비상정지와 Ctrl-C뿐이다. 저속·개활지·비상정지 담당 배치 필수.
   라이다를 쓰는 정식 구성에서는 이 도구 대신 stack_estop을 켤 것.
   stack_estop과 동시 실행 금지 (신호가 50ms마다 충돌한다).

사용:  python3 manual_go.py
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from fma_interfaces.msg import EstopRequest


class ManualGo(Node):

    def __init__(self):
        super().__init__('manual_go')
        self.pub = self.create_publisher(EstopRequest, '/perception/estop', 1)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().warn(
            '★ 수동 GO 발행 시작 — 차량이 출발할 수 있습니다! '
            '정지: 이 터미널 Ctrl-C (250ms) 또는 물리 비상정지')

    def tick(self):
        m = EstopRequest()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.estop = False
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ManualGo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print("\n[manual_go] 발행 중단 — 250ms 내 정지됩니다.")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
