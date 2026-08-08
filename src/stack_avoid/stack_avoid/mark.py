#!/usr/bin/env python3
"""현장 구간 표시기 — 실차 시험 중 타이핑한 라벨을 /test/event로 발행한다.  이기돈

실차 세션은 한 번에 여러 항목(①②③ⓐⓑⓒ)을 돌리므로, 나중에 bag에서 "이 30초가
어떤 시험이었나"를 알 수 있어야 한다. 그걸 남기는 도구.

  ros2 run stack_avoid mark
  > 3            ← 프리셋: 콘 3m 시작
  > ⓑ 1m 급투입   ← 아무 문자열이나 그대로 라벨이 됨
  > q            ← 종료

프리셋 (숫자만 치면 됨):
  1 / 2 / 3      감지 신뢰 거리 — 콘 1m / 2m / 3m 구간 시작   (실측 ③)
  a / b / c      경계 시험 ⓐ / ⓑ / ⓒ 구간 시작
  x              직전 구간 종료(무효 표시) — 재시도할 때
"""
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


PRESETS = {
    '1': 'cone 1m 시작 (③ 감지 신뢰 거리)',
    '2': 'cone 2m 시작 (③ 감지 신뢰 거리)',
    '3': 'cone 3m 시작 (③ 감지 신뢰 거리)',
    'a': 'ⓐ 3m 콘 회피 — estop 미발동 확인',
    'b': 'ⓑ 1m 급투입 — estop 발동 확인',
    'c': 'ⓒ 연석 접근 — avoid 미진입 + 정지 확인',
    'x': '직전 구간 무효 (재시도)',
}


def main(args=None):
    """표준입력을 읽어 각 줄을 /test/event로 발행한다."""
    rclpy.init(args=args)
    node = Node('test_marker')
    pub = node.create_publisher(String, '/test/event', 10)
    # 발행 직후 종료해도 유실되지 않도록 잠깐 스핀한다.
    node.get_logger().info('구간 표시기 — 프리셋 1/2/3/a/b/c/x, 그 외는 자유 라벨, q 종료')
    for key, desc in PRESETS.items():
        print(f'  {key} → {desc}')
    try:
        while rclpy.ok():
            try:
                line = input('mark> ').strip()
            except EOFError:
                break
            if not line:
                continue
            if line in ('q', 'quit', 'exit'):
                break
            label = PRESETS.get(line, line)
            m = String()
            m.data = label
            pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.1)
            print(f'  ▶ {label}')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
