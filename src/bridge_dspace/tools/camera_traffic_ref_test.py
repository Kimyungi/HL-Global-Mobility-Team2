#!/usr/bin/env python3
"""카메라(stack_traffic) 연동 CAN 테스트 케이스.

목적: 실제 판단 로직(MGM) 없이, 카메라가 빨간불/정지선을 인식하는 것만으로
dSPACE로 나가는 v_ref가 줄어드는지 CAN 왕복으로 확인한다. dummy_ref_publisher
(고정 v_ref)의 카메라-반응형 버전이며, /adas/target_ref로 발행하므로
bridge_dspace의 can_bridge_node가 그대로 CAN 인코딩·counter 부여를 맡는다.

⚠ dummy_ref_publisher와 마찬가지로 adas_mgm(mgm_node)이 올라오면 대체된다 —
같은 /adas/target_ref에 두 발행자가 동시에 붙으면 안 되므로 **mgm_node와
동시 실행 금지**. 여기서 검증한 "정지선 소실 edge 시드 + 실측 차속
dead-reckoning + traffic_stop_required 진입 전 낡은 값 방지" 로직은
adas_mgm/core/mgm_step.cpp(신호등 정지 ramp, 2026-09-01)로 그대로
이식됐다 — dSPACE로 내려가는 판단은 이제 그쪽 한 곳(§5.1)에서 관리하고,
이 스크립트는 카메라 단독 벤치 테스트용으로만 남는다.

로직 (사용자 지정, 2026-09-01):
1. ref point는 항상 (x=1, y=0, yaw=0, curvature=0) 고정 — 직진. dx/dy/dyaw는
   항상 0(이 스택 자체가 GPS 갱신을 흉내내지 않음), update는 매 tick +1.
2. v_ref 기본값 1.0 m/s.
3. stack_traffic의 red_active(디바운스된 빨간불 판정)가 한 번이라도 뜨면
   내부 'traffic' 상태로 전환. green_active가 뜨면 다시 해제(재출발) —
   2026-09-01 추가. 이 상태는 CAN TARGET_HEADER의 state 필드와 무관한
   이 테스트 노드 자체의 내부 값이다.
4. stopline_detected가 True -> False로 떨어지는 시점(=화면에서 정지선이
   사라진 시점)마다 내부 dist를 1.5m로 (재)설정한다. 다시 보였다 사라지면
   또 1.5m로 리셋된다.
5. 매 tick, /vehicle/vector의 실제 속도(v)로 dist를 dead-reckoning
   감쇠한다: dist -= v * dt (0 하한). 단, traffic 상태로 한 번도 안
   들어간 채 dist가 stop_distance_m 이하로 떨어지면 reset_distance_m로
   되돌리고 traffic 상태가 될 때까지 계속 붙잡아둔다(2026-09-01 추가) —
   신호와 무관하게 스쳐 지나간 정지선 때문에 쌓인 낡은 감쇠값이 나중에
   빨간불이 뜨는 순간 그대로 급정지로 이어지는 걸 막는다.
6. traffic 상태이고 dist <= 0.5m이면 v_ref = 0.
7. traffic 상태이고 dist > 0.5m이면 v_ref = clip(dist / 1.5, 0, 1).
   traffic 상태가 아니면 v_ref = 기본값(1.0 m/s).

⚠ TargetRef 발행 주기 = 10ms 고정 (tx_period_ms). PROTOCOL.md의 dSPACE
watchdog은 TARGET_HEADER counter가 30ms(3주기) 안에 안 바뀌면 v_ref를 0으로
강제한다. can_bridge_node는 TargetRef 수신 시에만 CAN을 내보내고 자체
재송신이 없으므로(브리지 keep-alive 금지 원칙), 100ms로 발행하면 매 주기
watchdog에 걸려 차가 안 굴러간다(2026-09-01 실차에서 확인). 로직/로그는
log_period_ms(기본 100ms)마다 한 번만 찍는다 — dist 감쇠·판정은 매 10ms tick
그대로 수행되고 로그만 솎아낸다.

로깅: log_period_ms(기본 100ms)마다 v_actual/v_ref/state/red/green/stopline/dist
한 행을 CSV로 저장소 안 log/camera_traffic_test/<타임스탬프>.csv에 자동 기록한다
(-p csv_path:=/원하는/경로.csv로 재지정 가능). /tmp 스크래치패드에 두지
않는 이유: 세션 재시작마다 지워진다(2026-09-01 유실 실측).

사용:
    ros2 run 없이 직접 실행 (bridge_dspace tools/ 관례):
    python3 src/bridge_dspace/tools/camera_traffic_ref_test.py
    (사전에 `source install/setup.bash`, stack_traffic·can_bridge_node 기동)
"""

from __future__ import annotations

import csv
import time
from pathlib import Path

import rclpy
from fma_interfaces.msg import RefPoint, TargetRef, TrafficStop, VehicleVector
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# src/bridge_dspace/tools/이 파일 위치 -> 저장소 루트/log/camera_traffic_test/.
# /tmp 스크래치패드에 두면 세션 재시작마다 지워진다(2026-09-01 실측 유실).
REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CSV_DIR = REPO_ROOT / "log" / "camera_traffic_test"


class CameraTrafficRefTestNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_traffic_ref_test_node")

        # 10ms 고정 — dSPACE watchdog(30ms=3주기, counter 미갱신 시 v_ref
        # 강제 0)을 만족하려면 이 값보다 느리게 발행하면 안 된다.
        self.declare_parameter("tx_period_ms", 10)
        self.declare_parameter("log_period_ms", 100)
        self.declare_parameter("default_v_ref", 1.0)
        self.declare_parameter("stopline_reset_distance_m", 1.5)
        # adas_mgm/core/mgm_step.cpp의 traffic_stop_offset_m과 동일한 값으로
        # 맞춘다 — 2026-09-02 사용자 지정 0.5→1.0m (seed 1.5m와의 차 0.5m가
        # 그대로 제동 구간).
        self.declare_parameter("stop_distance_m", 1.0)
        # 빈 문자열이면 저장소 안 log/camera_traffic_test/<타임스탬프>.csv에
        # 자동 저장 — /tmp 스크래치패드는 세션 재시작 시 지워지므로 쓰지 않는다.
        self.declare_parameter("csv_path", "")

        self.period_s = self.get_parameter("tx_period_ms").value / 1000.0
        self.log_every_n_ticks = max(
            1,
            round(
                self.get_parameter("log_period_ms").value
                / self.get_parameter("tx_period_ms").value
            ),
        )
        self.default_v_ref = float(
            self.get_parameter("default_v_ref").value
        )
        self.reset_distance_m = float(
            self.get_parameter("stopline_reset_distance_m").value
        )
        self.stop_distance_m = float(
            self.get_parameter("stop_distance_m").value
        )

        csv_path_param = str(self.get_parameter("csv_path").value).strip()
        if csv_path_param:
            self.csv_path = Path(csv_path_param).expanduser()
        else:
            DEFAULT_CSV_DIR.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self.csv_path = DEFAULT_CSV_DIR / f"{stamp}.csv"
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "stamp_s",
                "v_actual_mps",
                "v_ref_mps",
                "state",
                "red_active",
                "green_active",
                "stopline_detected",
                "dist_m",
            ]
        )
        self.csv_file.flush()

        self.traffic_state = False
        self.dist_m: float | None = None
        self.previous_stopline_detected = False
        self.red_active = False
        self.green_active = False
        self.stopline_detected = False
        self.actuator_velocity_mps = 0.0
        self.update_counter = 0

        self.target_ref_pub = self.create_publisher(
            TargetRef, "/adas/target_ref", 1
        )
        self.create_subscription(
            TrafficStop,
            "/perception/traffic_stop",
            self._on_traffic_stop,
            1,
        )
        self.create_subscription(
            VehicleVector,
            "/vehicle/vector",
            self._on_vehicle_vector,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(self.period_s, self._tick)
        self.get_logger().info(
            "camera_traffic_ref_test 시작 | "
            f"tx_period={self.period_s * 1000.0:.0f}ms "
            f"log_period={self.log_every_n_ticks * self.period_s * 1000.0:.0f}ms "
            f"default_v_ref={self.default_v_ref:.2f}m/s "
            f"reset_dist={self.reset_distance_m:.2f}m "
            f"stop_dist={self.stop_distance_m:.2f}m "
            f"csv={self.csv_path}"
        )

    def _on_traffic_stop(self, msg: TrafficStop) -> None:
        self.red_active = bool(msg.red_active)
        self.green_active = bool(msg.green_active)
        self.stopline_detected = bool(msg.stopline_detected)

    def _on_vehicle_vector(self, msg: VehicleVector) -> None:
        self.actuator_velocity_mps = float(msg.v)

    def _tick(self) -> None:
        if not self.traffic_state and self.red_active:
            self.traffic_state = True
            self.get_logger().info("빨간불 인식 -> traffic 상태 전환")
        elif self.traffic_state and self.green_active:
            self.traffic_state = False
            self.dist_m = self.reset_distance_m
            self.get_logger().info(
                "파란불 인식 -> traffic 상태 해제, "
                f"dist={self.dist_m:.2f}m로 재설정 (재출발)"
            )

        if self.previous_stopline_detected and not self.stopline_detected:
            self.dist_m = self.reset_distance_m
            self.get_logger().info(
                f"정지선 소실 -> dist={self.dist_m:.2f}m로 리셋"
            )
        self.previous_stopline_detected = self.stopline_detected

        if self.dist_m is not None:
            self.dist_m = max(
                0.0,
                self.dist_m - self.actuator_velocity_mps * self.period_s,
            )

        # traffic 상태로 한 번도 안 들어간 채(=아직 빨간불을 못 봤는데) dist가
        # stop_distance 이하로 떨어졌다면, 나중에 red_active가 뜨는 순간
        # 그동안 쌓인 감쇠분 때문에 바로 급정지(v_ref=0)해 버린다 — 지금
        # 신호와 무관하게 소진된 값이 다음 신호에 그대로 넘어가는 버그다.
        # traffic_state가 될 때까지 dist를 reset_distance에 계속 고정한다.
        if (
            not self.traffic_state
            and self.dist_m is not None
            and self.dist_m <= self.stop_distance_m
        ):
            self.dist_m = self.reset_distance_m
            self.get_logger().info(
                "traffic 상태 진입 전 dist 소진 -> "
                f"{self.dist_m:.2f}m로 재설정, 진입 전까지 감쇠 보류"
            )

        if self.traffic_state and self.dist_m is not None:
            if self.dist_m <= self.stop_distance_m:
                v_ref = 0.0
            else:
                v_ref = min(1.0, max(0.0, self.dist_m / self.reset_distance_m))
        else:
            v_ref = self.default_v_ref

        self.update_counter += 1

        msg = TargetRef()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.state = TargetRef.STATE_LANE
        msg.v_ref = float(v_ref)
        msg.ref_points = [
            RefPoint(x=1.0, y=0.0, yaw=0.0, curvature=0.0)
        ]
        msg.dx = 0.0
        msg.dy = 0.0
        msg.dyaw = 0.0
        msg.update = self.update_counter
        self.target_ref_pub.publish(msg)

        if self.update_counter % self.log_every_n_ticks == 0:
            state_text = "traffic" if self.traffic_state else "normal"
            dist_text = (
                "n/a" if self.dist_m is None else f"{self.dist_m:.2f}m"
            )
            self.get_logger().info(
                f"v_actual={self.actuator_velocity_mps:+.3f}m/s "
                f"v_ref={v_ref:.3f}m/s "
                f"state={state_text} "
                f"red={int(self.red_active)} green={int(self.green_active)} "
                f"stopline={int(self.stopline_detected)} "
                f"dist={dist_text}"
            )
            self.csv_writer.writerow(
                [
                    f"{time.time():.3f}",
                    f"{self.actuator_velocity_mps:.4f}",
                    f"{v_ref:.4f}",
                    state_text,
                    int(self.red_active),
                    int(self.green_active),
                    int(self.stopline_detected),
                    "" if self.dist_m is None else f"{self.dist_m:.4f}",
                ]
            )
            self.csv_file.flush()

    def close(self) -> None:
        # 매 로그 tick마다 flush하므로 SIGTERM으로 죽어도 그때까지의 행은
        # 이미 디스크에 있다 — 이건 정상 종료 경로의 마무리일 뿐이다.
        if not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f"CSV 저장 완료: {self.csv_path}")


def main() -> None:
    rclpy.init()
    node = CameraTrafficRefTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
