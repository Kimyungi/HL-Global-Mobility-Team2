#!/usr/bin/env python3
"""조향 미반영 진단용: 우리가 보낸 목표(y/yaw)와 dSPACE의 실제 조향각(str)을
같은 타임라인에 기록해서 상관관계를 확인한다.

- /adas/target_ref (ROS, MGM 출력 — 우리가 보낸 목표)
- CAN 0x202 VEH_COMMIT의 str 필드 (dSPACE가 실제로 계산한 조향각, raw 소켓으로 직접 수신)

둘을 시간 정렬해서 CSV로 남긴다. str이 target y/yaw와 같이 움직이면 dSPACE
계산까지는 정상 — 이후 액추에이션(서보) 쪽 문제. str이 안 움직이면 dSPACE가
우리 목표를 애초에 반영 안 하는 것 — 컨트롤러모드/CAN 수신 설정 쪽 확인 필요.

사용:
  python3 scripts/log_steering_correlation.py --iface can0 --out steering_corr.csv
  (Ctrl-C로 종료, 종료 시 상관계수 요약 출력)
"""
import argparse
import csv
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node

from fma_interfaces.msg import TargetRef

ID_VEH_COMMIT = 0x202


def open_can_socket(iface: str) -> socket.socket:
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    return s


class CorrelationLogger(Node):

    def __init__(self, iface: str, out_path: str):
        super().__init__('log_steering_correlation')
        self.rows = []
        self.lock = threading.Lock()
        self.latest_target = None  # (y, yaw)

        self.sub = self.create_subscription(
            TargetRef, '/adas/target_ref', self.on_target, 1)

        self.can_sock = open_can_socket(iface)
        self.can_sock.settimeout(0.5)
        self.running = True
        self.can_thread = threading.Thread(target=self.can_loop, daemon=True)
        self.can_thread.start()

        self.out_path = out_path
        self.get_logger().info(f'조향 상관관계 로깅 시작 — {out_path} (Ctrl-C로 종료)')

    def on_target(self, msg: TargetRef):
        if msg.ref_points:
            p = msg.ref_points[0]
            p_last = msg.ref_points[-1]
            with self.lock:
                # state: 0=lane 1=waypoint 2=avoid 3=parking (CLAUDE.md §4) —
                # waypoint 중엔 gps_path가 없어 ref가 마지막 lane 값에 멈춰있을
                # 수 있음(assemble() hold). state를 같이 남겨야 그 구간을 구분 가능.
                # n_points/last_x: 2026-08-08 다점 진단의 핵심 변수 — 몇 점이
                # 실제로 나갔는지, 원거리 점까지 채워졌는지 바로 확인하기 위함.
                # p.x(REF_POINT_00의 x): 2026-08-10 3계층 진단용 — 지금까지 y만
                # 남겨서 pure-pursuit 이론 곡률(κ=2y/(x²+y²))을 실측 str과 비교할
                # 방법이 없었음. x가 있어야 Vehicle MGM 추종 품질을 이론값 대비로
                # 따로 떼어 볼 수 있다.
                self.latest_target = (
                    p.y, p.yaw, msg.v_ref, msg.state, len(msg.ref_points), p_last.x, p_last.y, p.x)

    def can_loop(self):
        while self.running:
            try:
                frame = self.can_sock.recv(16)
            except socket.timeout:
                continue
            can_id, dlc, data = struct.unpack("<IB3x8s", frame)
            can_id &= 0x1FFFFFFF
            if can_id != ID_VEH_COMMIT:
                continue
            str_val, counter, _ = struct.unpack("<fHH", data[:8])
            with self.lock:
                target = self.latest_target
            row = {
                'wall_time': time.time(),
                'dspace_str_rad': str_val,
                'dspace_counter': counter,
                'target_y_m': target[0] if target else None,
                'target_yaw_rad': target[1] if target else None,
                'target_v_ref': target[2] if target else None,
                'mgm_state': target[3] if target else None,  # 0=lane 1=waypoint 2=avoid 3=parking
                'n_points': target[4] if target else None,
                'target_last_x_m': target[5] if target else None,
                'target_last_y_m': target[6] if target else None,
                'target_x_m': target[7] if target else None,
            }
            with self.lock:
                self.rows.append(row)

    def save_and_summarize(self):
        self.running = False
        if not self.rows:
            print("기록된 프레임 없음")
            return
        with open(self.out_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)
        print(f"저장됨: {self.out_path} ({len(self.rows)}행)")

        strs = [r['dspace_str_rad'] for r in self.rows]
        yaws = [r['target_yaw_rad'] for r in self.rows if r['target_yaw_rad'] is not None]
        print(f"\ndspace str 범위: min={min(strs):.4f} max={max(strs):.4f} "
              f"std={(sum((s - sum(strs)/len(strs))**2 for s in strs)/len(strs))**0.5:.5f}")
        if yaws:
            print(f"우리가 보낸 target yaw 범위: min={min(yaws):.4f} max={max(yaws):.4f}")

        # state별 분포 — waypoint에 갇혀있었는지, lane 구간에서도 str이 안 움직였는지 구분
        state_names = {0: 'lane', 1: 'waypoint', 2: 'avoid', 3: 'parking'}
        states = [r['mgm_state'] for r in self.rows if r['mgm_state'] is not None]
        if states:
            print("\nMGM state 분포:")
            for sv in sorted(set(states)):
                n = states.count(sv)
                print(f"  {state_names.get(sv, sv)}: {n}개 ({n/len(states)*100:.1f}%)")

            lane_rows = [r for r in self.rows if r.get('mgm_state') == 0]
            if lane_rows:
                lane_strs = [r['dspace_str_rad'] for r in lane_rows]
                print(f"\nlane 상태였던 구간만 볼 때 str 변동폭: "
                      f"min={min(lane_strs):.4f} max={max(lane_strs):.4f} "
                      f"(폭={max(lane_strs)-min(lane_strs):.4f})")
                if max(lane_strs) - min(lane_strs) < 0.01:
                    print(">>> lane 상태에서도 str이 안 움직임 — waypoint 갇힘 문제가 아니라 "
                          "dSPACE가 lane 목표 자체를 반영 안 하는 것으로 확정적")
            else:
                print("\n>>> 이번 로그엔 lane 상태 구간이 아예 없었음 — "
                      "waypoint에 계속 갇혀있었을 가능성 있음, target이 lane 값에서 멈춰있었을 수 있음")

        # n_points별 분포 — GPS 성공 로그 대조 결과 이게 핵심 변수로 확인됨
        # (n_points=1 구간 str 변동폭 0.1° vs n_points=20 구간 55.6°, 같은 세션 내 대조).
        npoints_list = [r['n_points'] for r in self.rows if r.get('n_points') is not None]
        if npoints_list:
            print("\nn_points 분포:")
            for nv in sorted(set(npoints_list)):
                n = npoints_list.count(nv)
                print(f"  {nv}점: {n}개 ({n/len(npoints_list)*100:.1f}%)")
            many_rows = [r for r in self.rows if (r.get('n_points') or 0) >= 10]
            few_rows = [r for r in self.rows if r.get('n_points') is not None and r['n_points'] < 10]
            if many_rows:
                many_strs = [r['dspace_str_rad'] for r in many_rows]
                print(f"\nn_points>=10 구간 str 변동폭: {max(many_strs)-min(many_strs):.4f}rad "
                      f"({len(many_rows)}개)")
            if few_rows:
                few_strs = [r['dspace_str_rad'] for r in few_rows]
                print(f"n_points<10 구간 str 변동폭: {max(few_strs)-min(few_strs):.4f}rad "
                      f"({len(few_rows)}개)")

        if max(strs) - min(strs) < 0.01:
            print("\n>>> str이 거의 안 움직임 (변동폭 <0.01rad) — dSPACE가 목표를 반영 안 하는 것으로 보임")
        else:
            print("\n>>> str이 움직이고 있음 — target_yaw/y와 같이 변하는지 CSV 열어서 직접 비교 필요"
                  " (같이 움직이면 계산은 정상, 액추에이션 쪽 확인)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iface', default='can0')
    parser.add_argument('--out', default='steering_correlation.csv')
    args = parser.parse_args()

    rclpy.init()
    node = CorrelationLogger(args.iface, args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_and_summarize()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
