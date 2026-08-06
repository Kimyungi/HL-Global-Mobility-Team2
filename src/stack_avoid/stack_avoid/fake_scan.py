#!/usr/bin/env python3
"""합성 LaserScan 퍼블리셔 — 하드웨어 없이 stack_avoid 회피 테스트.

장애물을 vehicle frame(x 전방+, y 좌측+, 후축 원점)으로 지정하면, 실제 라이다
(forward_angle=270 관례)와 동일하게 보이는 `/scan` 을 만들어 낸다. stack_avoid
노드가 이 스캔을 받으면 지정한 vehicle 좌표 그대로 장애물을 인식한다.

런타임에 장애물을 바꿔 시나리오를 즉시 시험:
    ros2 param set /fake_scan obstacles "2.0,0.0"          # 정면 1개
    ros2 param set /fake_scan obstacles "2.0,0.4; 2.0,-0.4"  # 양쪽
    ros2 param set /fake_scan obstacles "1.5,0.2; 1.5,-0.2"  # 좁은 틈(→narrow_gap)
    ros2 param set /fake_scan obstacles ""                   # 장애물 없음
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan


def parse_obstacles(s):
    """'x1,y1; x2,y2' → [(x1,y1),(x2,y2)] (vehicle frame, m)."""
    out = []
    for part in str(s).split(';'):
        part = part.strip()
        if not part:
            continue
        try:
            x, y = (float(v) for v in part.split(','))
            out.append((x, y))
        except ValueError:
            pass
    return out


class FakeScan(Node):

    def __init__(self):
        super().__init__('fake_scan')
        self.lidar_x = self.declare_parameter('lidar_x_m', 0.76).value
        self.lidar_y = self.declare_parameter('lidar_y_m', 0.0).value
        self.front_center = math.radians(
            self.declare_parameter('forward_angle_deg', 270.0).value)
        self.n = int(self.declare_parameter('num_points', 720).value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 10.0).value)
        self.obst_radius = float(self.declare_parameter('obstacle_radius_m', 0.10).value)
        self.range_max = float(self.declare_parameter('range_max_m', 12.0).value)
        self.frame_id = self.declare_parameter('frame_id', 'laser_frame').value
        self.obstacles = parse_obstacles(
            self.declare_parameter('obstacles', '2.0,0.0').value)

        self.amin = -math.pi
        self.ainc = 2.0 * math.pi / self.n

        self.pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.add_on_set_parameters_callback(self._on_set)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            f"fake_scan: 장애물 {self.obstacles} (vehicle frame) | "
            f"forward={math.degrees(self.front_center):.0f}deg, "
            f"{self.n}점 @ {self.rate_hz}Hz → /scan")

    def _on_set(self, params):
        for p in params:
            if p.name == 'obstacles':
                self.obstacles = parse_obstacles(p.value)
                self.get_logger().info(f"장애물 갱신 → {self.obstacles}")
        return SetParametersResult(successful=True)

    def tick(self):
        ranges = [float('inf')] * self.n
        for (xv, yv) in self.obstacles:
            dx = xv - self.lidar_x        # 라이다 기준 전방거리
            dy = yv - self.lidar_y        # 라이다 기준 좌측거리
            base = math.hypot(dx, dy)
            if base < 1e-3:
                continue
            rel0 = math.atan2(dy, dx)      # 라이다 정면 기준 방위 (forward=0)
            # 반경 obst_radius 만큼 여러 각도에 표면점 삽입
            span = max(1, int(math.atan2(self.obst_radius, base) / self.ainc))
            for k in range(-span, span + 1):
                # scan 각도 = 라이다프레임 방위 + forward_center (노드의 역변환과 대응)
                sa = rel0 + k * self.ainc + self.front_center
                sa = math.atan2(math.sin(sa), math.cos(sa))
                idx = int(round((sa - self.amin) / self.ainc)) % self.n
                if base < ranges[idx]:
                    ranges[idx] = base

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = self.amin
        msg.angle_max = self.amin + (self.n - 1) * self.ainc
        msg.angle_increment = self.ainc
        msg.range_min = 0.03
        msg.range_max = self.range_max
        msg.ranges = ranges
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
