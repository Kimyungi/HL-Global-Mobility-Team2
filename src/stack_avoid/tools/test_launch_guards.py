#!/usr/bin/env python3
"""launch 공용 가드 회귀 시험 (ROS 그래프·하드웨어 없음).  이기돈

`stack_avoid.launch_parts` 의 안전 가드가 실제로 걸리는지 확인한다. launch 를 실제로
띄우지 않고 OpaqueFunction 본체를 직접 호출해 **반환된 액션**을 본다.

  python3 src/stack_avoid/tools/test_launch_guards.py      # 전부 ✓ 여야 함

검사 항목:
  1) estop_off 미지정 → on+0.10 자동 파생, stack_estop 기동
  2) estop_on:=0.90 (예전에 즉사하던 값) → 자동 파생으로 정상 기동
  3) on >= off 모순 → Shutdown (노드 생성 안 됨)
  4) on <= 0 → Shutdown
  5) stack_estop 노드에 on_exit=Shutdown 이 걸려 있는가 (안전 노드 사망 = 세션 중단)
  6) can 가드가 브리지 + 종료 후 0송신 + 폴백 3종을 모두 내는가
"""
import sys

from launch import LaunchContext
from launch.actions import Shutdown
from launch_ros.actions import Node

from stack_avoid.launch_parts import can_bridge_with_zero_guard, safety_node


def ctx(**kv):
    c = LaunchContext()
    for k, v in kv.items():
        c.launch_configurations[k] = v
    return c


def kinds(actions):
    return [type(a).__name__ for a in actions]


def estop_nodes(actions):
    return [a for a in actions if isinstance(a, Node)]


def node_params(node):
    """Node 액션의 파라미터 dict → 이름:값.

    launch_ros 는 키를 TextSubstitution 튜플로 보관한다. 값 비교를 위해 평문 이름으로
    되돌린다(치환이 필요 없는 리터럴 키만 다루므로 .text 로 충분하다).
    """
    out = {}
    for k, v in node._Node__parameters[0].items():
        name = ''.join(getattr(t, 'text', '') for t in k)
        out[name] = v
    return out


def main():
    results = []

    def check(name, ok, detail=''):
        results.append(ok)
        print(f"{'✓' if ok else '✗'} {name:44s} {detail}")

    # 1) 자동 파생
    acts = safety_node(ctx(estop_on_distance_m='0.70', estop_off_distance_m='', dynamic='true'))
    nodes = estop_nodes(acts)
    ok = len(nodes) == 1 and not any(isinstance(a, Shutdown) for a in acts)
    off = node_params(nodes[0])['estop_off_distance_m'] if nodes else None
    check('1) off 미지정 → 자동 파생', ok and abs(off - 0.80) < 1e-9, f'off={off}')

    # 2) 예전에 즉사하던 값
    acts = safety_node(ctx(estop_on_distance_m='0.90', estop_off_distance_m='', dynamic='true'))
    nodes = estop_nodes(acts)
    off = node_params(nodes[0])['estop_off_distance_m'] if nodes else None
    check('2) on:=0.90 (예전 즉사값) → 정상 기동', len(nodes) == 1 and off == 1.0, f'off={off}')

    # 3) 모순 → Shutdown, 노드 없음
    acts = safety_node(ctx(estop_on_distance_m='0.90', estop_off_distance_m='0.80',
                           dynamic='true'))
    check('3) on >= off → 기동 전 중단',
          any(isinstance(a, Shutdown) for a in acts) and not estop_nodes(acts), str(kinds(acts)))

    # 4) 0 이하 → Shutdown
    acts = safety_node(ctx(estop_on_distance_m='0', estop_off_distance_m='', dynamic='true'))
    check('4) on <= 0 → 기동 전 중단',
          any(isinstance(a, Shutdown) for a in acts) and not estop_nodes(acts), str(kinds(acts)))

    # 5) 안전 노드 사망 시 세션 중단
    acts = safety_node(ctx(estop_on_distance_m='0.70', estop_off_distance_m='', dynamic='true'))
    node = estop_nodes(acts)[0]
    # launch(Humble) 는 on_exit 를 ExecuteLocal 에 name-mangled 로 보관한다.
    on_exit = next((getattr(node, a) for a in dir(node) if a.endswith('__on_exit')), None)
    has = isinstance(on_exit, Shutdown) or (
        isinstance(on_exit, (list, tuple)) and any(isinstance(a, Shutdown) for a in on_exit))
    check('5) stack_estop 사망 → 세션 Shutdown', bool(has), type(on_exit).__name__)

    # 6) CAN 가드 3종
    acts = can_bridge_with_zero_guard()
    k = kinds(acts)
    check('6) can 가드 = 브리지+종료후0+폴백',
          k == ['Node', 'RegisterEventHandler', 'Node'], str(k))

    bad = results.count(False)
    print('\n=== 결과: ' + ('전부 통과' if not bad else f'{bad}건 실패') + ' ===')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
