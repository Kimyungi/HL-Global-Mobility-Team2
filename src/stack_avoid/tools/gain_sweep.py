#!/usr/bin/env python3
"""조향 게인 스윕 — 어느 레버가 실제로 조향을 키우는지 실차에서 확정한다.  이기돈

배경: 회피 목표점(예: 2.76, 0.46)은 그대로 두는데도 실차 조향이 작았다(8/6).
avoid_to_ref가 목표점을 그대로 보내지 않고 호 위 lookahead 점으로 바꿔 보내기 때문에
실제 송신 ref는 (0.400, 0.0094) — y가 목표의 2%다. 키울 수 있는 레버가 둘인데
dSPACE가 어느 쪽에 반응하는지가 실증되지 않았다:

  A) curvature_gain — 송신 위치(x,y,yaw) 불변, 곡률만 배수. 물리 한계는 7.4배(κ 0.870).
  B) lookahead_m    — 회피 목표점은 불변, 호 위 어느 점을 보낼지만 변경.
                      호 길이를 넘기면 자동 클램프되어 목표점 자체가 송신된다.

이 도구는 한 파라미터를 값 목록대로 바꾸며 각 구간에 /test/event 라벨을 남긴다.
그 bag을 analyze_field_bag.py에 넣으면 구간별 명령 κ 대비 실제 |str|이 표로 나온다.

  # B 스윕 (권장 — 8/6 실측이 ref y에 반응했다)
  python3 tools/gain_sweep.py --param lookahead_m --values 0.4 1.2 2.0 2.8 --dwell 15
  # A 스윕
  python3 tools/gain_sweep.py --param curvature_gain --values 1.0 3.0 5.0 7.0 --dwell 15

★ 한 번에 한 레버만. 둘을 같이 흔들면 어느 쪽이 들었는지 알 수 없다.
★ 스탠드(바퀴 듦)에서 먼저. 지상에서 할 때는 통제된 구간에서.
★ 끝나면 원래 값으로 되돌린다(중단해도 되돌리도록 finally 처리).
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import String


class Sweeper(Node):
    """대상 노드의 파라미터를 순회 설정하며 구간 라벨을 발행한다."""

    def __init__(self, target):
        """target 노드의 파라미터 서비스에 연결하고 이벤트 퍼블리셔를 만든다."""
        super().__init__('gain_sweep')
        self.target = target
        self.event = self.create_publisher(String, '/test/event', 10)
        self.set_cli = self.create_client(SetParameters, f'{target}/set_parameters')
        self.get_cli = self.create_client(GetParameters, f'{target}/get_parameters')

    def wait(self, timeout=10.0):
        """대상 노드가 뜰 때까지 대기."""
        ok = self.set_cli.wait_for_service(timeout_sec=timeout)
        ok &= self.get_cli.wait_for_service(timeout_sec=timeout)
        return ok

    def get(self, name):
        """현재 double 파라미터 값 (없으면 None)."""
        req = GetParameters.Request()
        req.names = [name]
        fut = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if not res or not res.values:
            return None
        v = res.values[0]
        if v.type != ParameterType.PARAMETER_DOUBLE:
            return None
        return v.double_value

    def set(self, name, value):
        """double 파라미터 설정 → 성공 여부."""
        p = Parameter()
        p.name = name
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=float(value))
        req = SetParameters.Request()
        req.parameters = [p]
        fut = self.set_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        return bool(res and res.results and res.results[0].successful)

    def mark(self, label):
        """구간 라벨 발행 — 분석기가 이걸로 구간을 자른다."""
        m = String()
        m.data = label
        self.event.publish(m)
        rclpy.spin_once(self, timeout_sec=0.1)
        print(f'  ▶ {label}')


def main():
    """인자를 파싱해 스윕을 실행하고, 끝나면 원래 값으로 되돌린다."""
    ap = argparse.ArgumentParser(description='조향 게인 스윕')
    ap.add_argument('--node', default='/avoid_to_ref', help='대상 노드 (기본 /avoid_to_ref)')
    ap.add_argument('--param', required=True, help='lookahead_m 또는 curvature_gain')
    ap.add_argument('--values', nargs='+', type=float, required=True)
    ap.add_argument('--dwell', type=float, default=15.0, help='값마다 유지 [s]')
    a = ap.parse_args()

    rclpy.init()
    n = Sweeper(a.node)
    if not n.wait():
        print(f'✗ {a.node} 파라미터 서비스 없음 — 노드가 떠 있는지 확인')
        n.destroy_node()
        rclpy.shutdown()
        return 1

    original = n.get(a.param)
    if original is None:
        print(f'✗ {a.node} 에 double 파라미터 {a.param} 없음')
        n.destroy_node()
        rclpy.shutdown()
        return 1
    print(f'{a.node} {a.param}: 현재값 {original} → 스윕 {a.values} (각 {a.dwell}s)')

    try:
        for v in a.values:
            if not n.set(a.param, v):
                print(f'✗ {a.param}={v} 설정 실패 — 중단')
                break
            n.mark(f'sweep {a.param}={v}')
            end = time.time() + a.dwell
            while time.time() < end and rclpy.ok():
                rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\n중단됨')
    finally:
        n.mark(f'sweep end ({a.param} → {original} 복원)')
        n.set(a.param, original)
        print(f'{a.param} 을 {original} 로 복원했다.')
        n.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
