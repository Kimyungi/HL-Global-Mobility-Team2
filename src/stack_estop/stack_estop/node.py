"""Short, distance-only emergency-stop path for the official stack."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from fma_interfaces.msg import EstopRequest


class DistanceEstopController:
    """Scan-driven E-Stop hysteresis independent of perception tracking."""

    def __init__(
        self,
        on_distance_m=0.70,
        off_distance_m=0.80,
        clear_confirm_scans=3,
    ):
        if not 0.0 < on_distance_m < off_distance_m:
            raise ValueError(
                'Require 0 < estop_on_distance_m < estop_off_distance_m'
            )
        if int(clear_confirm_scans) < 1:
            raise ValueError('estop_clear_confirm_scans must be >= 1')
        self.on_distance_m = float(on_distance_m)
        self.off_distance_m = float(off_distance_m)
        self.clear_confirm_scans = int(clear_confirm_scans)
        self.current_final_estop = True
        self.clear_count = 0

    def update_from_scan(self, nearest_corridor_x_m):
        """Update exactly once for each new, valid LaserScan."""
        danger_now = bool(
            nearest_corridor_x_m is not None
            and nearest_corridor_x_m <= self.on_distance_m
        )
        if danger_now:
            self.current_final_estop = True
            self.clear_count = 0
            return danger_now

        safe_now = bool(
            nearest_corridor_x_m is None
            or nearest_corridor_x_m >= self.off_distance_m
        )
        if safe_now:
            if self.current_final_estop:
                self.clear_count += 1
                if self.clear_count >= self.clear_confirm_scans:
                    self.current_final_estop = False
                    self.clear_count = 0
            else:
                self.clear_count = 0
        else:
            # Hysteresis band: retain the level and restart clear confirmation.
            self.clear_count = 0
        return danger_now

    def force_timeout(self):
        self.current_final_estop = True
        self.clear_count = 0


def nearest_corridor_cluster_x(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    *,
    min_range_m=0.15,
    max_range_m=5.0,
    corridor_min_x_m=0.15,
    corridor_max_x_m=1.50,
    corridor_half_width_m=0.30,
    cluster_min_points=4,
    max_index_gap=1,
    max_neighbor_distance_m=0.12,
):
    """Return (valid_scan, nearest x) using only the current LaserScan."""
    if (
        not ranges
        or not math.isfinite(angle_increment)
        or angle_increment == 0.0
        or not math.isfinite(angle_min)
    ):
        return False, None
    minimum = max(float(range_min), float(min_range_m))
    maximum = min(float(range_max), float(max_range_m))
    if (
        not math.isfinite(minimum)
        or not math.isfinite(maximum)
        or maximum < minimum
    ):
        return False, None

    points = []
    for index, value in enumerate(ranges):
        if (
            not math.isfinite(value)
            or value <= 0.0
            or not minimum <= value <= maximum
        ):
            continue
        angle = angle_min + index * angle_increment
        x_value = float(value) * math.cos(angle)
        y_value = float(value) * math.sin(angle)
        if (
            corridor_min_x_m <= x_value <= corridor_max_x_m
            and abs(y_value) <= corridor_half_width_m
        ):
            points.append((index, x_value, y_value))

    clusters = []
    for point in points:
        if not clusters:
            clusters.append([point])
            continue
        previous = clusters[-1][-1]
        index_close = point[0] - previous[0] <= max_index_gap + 1
        spatial_close = math.hypot(
            point[1] - previous[1], point[2] - previous[2]
        ) <= max_neighbor_distance_m
        if index_close and spatial_close:
            clusters[-1].append(point)
        else:
            clusters.append([point])

    nearest = min(
        (
            min(point[1] for point in cluster)
            for cluster in clusters
            if len(cluster) >= cluster_min_points
        ),
        default=None,
    )
    return True, nearest


class StackEstopNode(Node):

    def __init__(self):
        super().__init__('stack_estop_node')
        self.declare_parameter('estop_on_distance_m', 0.70)
        self.declare_parameter('estop_off_distance_m', 0.80)
        self.declare_parameter('estop_clear_confirm_scans', 3)
        self.declare_parameter('scan_timeout_sec', 0.25)
        self.declare_parameter('publish_period_sec', 0.05)
        self.declare_parameter('min_range_m', 0.15)
        self.declare_parameter('max_range_m', 5.0)
        self.declare_parameter('corridor_min_x_m', 0.15)
        self.declare_parameter('corridor_max_x_m', 1.50)
        self.declare_parameter('corridor_half_width_m', 0.30)
        self.declare_parameter('cluster_min_points', 4)
        self.declare_parameter('max_index_gap', 1)
        self.declare_parameter('max_neighbor_distance_m', 0.12)

        self.controller = DistanceEstopController(
            self.get_parameter('estop_on_distance_m').value,
            self.get_parameter('estop_off_distance_m').value,
            self.get_parameter('estop_clear_confirm_scans').value,
        )
        self.scan_timeout_sec = float(
            self.get_parameter('scan_timeout_sec').value
        )
        self.publish_period_sec = float(
            self.get_parameter('publish_period_sec').value
        )
        if not math.isfinite(self.scan_timeout_sec) or self.scan_timeout_sec <= 0:
            raise ValueError('scan_timeout_sec must be finite and > 0')
        if (
            not math.isfinite(self.publish_period_sec)
            or self.publish_period_sec <= 0
        ):
            raise ValueError('publish_period_sec must be finite and > 0')

        self.geometry = {
            name: self.get_parameter(name).value
            for name in (
                'min_range_m',
                'max_range_m',
                'corridor_min_x_m',
                'corridor_max_x_m',
                'corridor_half_width_m',
                'cluster_min_points',
                'max_index_gap',
                'max_neighbor_distance_m',
            )
        }
        self.last_valid_scan_time = None
        self.last_processed_scan_stamp = None
        self.danger_present = False

        self.pub = self.create_publisher(
            EstopRequest, '/perception/estop', 1
        )
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.timer = self.create_timer(
            self.publish_period_sec, self.publish_heartbeat
        )

    def scan_callback(self, msg):
        scan_stamp = (
            int(msg.header.stamp.sec),
            int(msg.header.stamp.nanosec),
        )
        if scan_stamp == self.last_processed_scan_stamp:
            return
        self.last_processed_scan_stamp = scan_stamp

        valid_scan, nearest_x = nearest_corridor_cluster_x(
            msg.ranges,
            float(msg.angle_min),
            float(msg.angle_increment),
            float(msg.range_min),
            float(msg.range_max),
            min_range_m=float(self.geometry['min_range_m']),
            max_range_m=float(self.geometry['max_range_m']),
            corridor_min_x_m=float(self.geometry['corridor_min_x_m']),
            corridor_max_x_m=float(self.geometry['corridor_max_x_m']),
            corridor_half_width_m=float(
                self.geometry['corridor_half_width_m']
            ),
            cluster_min_points=int(self.geometry['cluster_min_points']),
            max_index_gap=int(self.geometry['max_index_gap']),
            max_neighbor_distance_m=float(
                self.geometry['max_neighbor_distance_m']
            ),
        )
        if not valid_scan:
            return

        danger_now = self.controller.update_from_scan(nearest_x)
        self.last_valid_scan_time = self.get_clock().now()
        if danger_now and not self.danger_present:
            self.publish_current_level()
        self.danger_present = danger_now

    def publish_current_level(self):
        msg = EstopRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.estop = self.controller.current_final_estop
        self.pub.publish(msg)

    def publish_heartbeat(self):
        # No distance decision and no clear-count change are allowed here.
        now = self.get_clock().now()
        if self.last_valid_scan_time is None:
            self.controller.force_timeout()
        else:
            scan_age_sec = (
                now - self.last_valid_scan_time
            ).nanoseconds * 1e-9
            if scan_age_sec > self.scan_timeout_sec:
                self.controller.force_timeout()
        self.publish_current_level()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StackEstopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
