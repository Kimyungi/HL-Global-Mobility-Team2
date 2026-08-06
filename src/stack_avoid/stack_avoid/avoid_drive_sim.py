#!/usr/bin/env python3
"""라이다 단독 회피 주행 폐루프 시뮬레이터 (테스트 전용).

목적: 실제 stack_avoid 노드가 내는 회피 목표점으로 가상 차량을 몰아서
"장애물을 제대로 피하는지 + 범위 내 장애물이 사라지면 멈추는지"를 RViz로 확인.

루프(map 프레임):
  ① 현재 차 자세에서 본 스캔 생성 → /scan  (forward_angle 반영, stack_avoid가 그대로 소비)
  ② stack_avoid → /perception/avoid (목표점, vehicle frame)
  ③ 제어(테스트용 최소 판단, 스테이트머신 아님):
       - 전방 감지범위 내 장애물 인식 O + 목표점 O → 목표점으로 pure-pursuit 조향, v=target_speed
       - 인식 O + 목표점 X(narrow 등) → 직진 유지
       - 인식 X (범위 내 장애물 없음) → **정지 (v=0, 그대로 멈춤)**
  ④ kinematic bicycle 적분 → 새 자세 → TF map→base_link + 마커
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from fma_interfaces.msg import AvoidStatus


def parse_obs(s):
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


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class AvoidDriveSim(Node):

    def __init__(self):
        super().__init__('avoid_drive_sim')
        # 시나리오
        self.obstacles = parse_obs(self.declare_parameter('obstacles', '3.0,0.0').value)
        self.px = float(self.declare_parameter('start_x', 0.0).value)
        self.py = float(self.declare_parameter('start_y', 0.0).value)
        self.pyaw = math.radians(float(self.declare_parameter('start_yaw_deg', 0.0).value))
        # 차량/센서 (params.yaml 실측값과 일치)
        self.target_speed = float(self.declare_parameter('target_speed_mps', 0.5).value)
        self.wheelbase = float(self.declare_parameter('wheelbase_m', 0.595).value)
        self.max_steer = math.radians(float(self.declare_parameter('max_steer_deg', 27.3).value))
        self.veh_w = float(self.declare_parameter('vehicle_width_m', 0.62).value)
        self.veh_l = float(self.declare_parameter('vehicle_length_m', 0.85).value)
        self.lidar_x = float(self.declare_parameter('lidar_x_m', 0.76).value)
        self.front_center = math.radians(float(self.declare_parameter('forward_angle_deg', 270.0).value))
        self.detect_range = float(self.declare_parameter('detect_range_m', 3.0).value)
        self.fov_half = math.radians(float(self.declare_parameter('fov_half_deg', 90.0).value))
        self.obst_r = float(self.declare_parameter('obstacle_radius_m', 0.10).value)
        # pure-pursuit lookahead (작을수록 목표 오프셋에 빨리 도달). 실차 MPC 근사용.
        self.lookahead = float(self.declare_parameter('lookahead_m', 1.0).value)
        self.n = int(self.declare_parameter('num_points', 720).value)
        self.dt = float(self.declare_parameter('dt', 0.05).value)
        self.world = self.declare_parameter('world_frame', 'map').value

        self.amin = -math.pi
        self.ainc = 2.0 * math.pi / self.n
        self.last_avoid = None
        self.trail = []
        self.stopped_logged = False

        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.mk_pub = self.create_publisher(MarkerArray, 'sim/markers', 1)
        self.sub = self.create_subscription(AvoidStatus, '/perception/avoid', self._on_avoid, 10)
        self.tf = TransformBroadcaster(self)
        self.add_on_set_parameters_callback(self._on_set)
        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f"avoid_drive_sim: 장애물(map){self.obstacles} 차시작({self.px},{self.py}) "
            f"v={self.target_speed} detect<{self.detect_range}m forward={math.degrees(self.front_center):.0f}deg")

    def _on_set(self, params):
        for p in params:
            if p.name == 'obstacles':
                self.obstacles = parse_obs(p.value)
                self.get_logger().info(f"장애물 갱신 → {self.obstacles}")
        return SetParametersResult(successful=True)

    def _on_avoid(self, msg):
        self.last_avoid = msg

    # 월드 장애물 → 차량(후축) 프레임
    def _to_vehicle(self, ox, oy):
        dx, dy = ox - self.px, oy - self.py
        c, s = math.cos(self.pyaw), math.sin(self.pyaw)
        return (c * dx + s * dy, -s * dx + c * dy)   # (x_fwd, y_left)

    def _build_scan(self):
        ranges = [float('inf')] * self.n
        for ox, oy in self.obstacles:
            xv, yv = self._to_vehicle(ox, oy)
            lx, ly = xv - self.lidar_x, yv
            base = math.hypot(lx, ly)
            if base < 1e-3:
                continue
            rel0 = math.atan2(ly, lx)
            span = max(1, int(math.atan2(self.obst_r, base) / self.ainc))
            for k in range(-span, span + 1):
                sa = rel0 + k * self.ainc + self.front_center
                sa = math.atan2(math.sin(sa), math.cos(sa))
                idx = int(round((sa - self.amin) / self.ainc)) % self.n
                if base < ranges[idx]:
                    ranges[idx] = base
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'laser_frame'
        m.angle_min = self.amin
        m.angle_max = self.amin + (self.n - 1) * self.ainc
        m.angle_increment = self.ainc
        m.range_min = 0.03
        m.range_max = 12.0
        m.ranges = ranges
        return m

    def _obstacle_in_range(self):
        """전방 FOV·감지거리 안에 장애물이 인식되는가 (라이다가 보는 범위)."""
        for ox, oy in self.obstacles:
            xv, yv = self._to_vehicle(ox, oy)
            lx, ly = xv - self.lidar_x, yv
            if lx <= 0:
                continue
            if math.hypot(lx, ly) <= self.detect_range and abs(math.atan2(ly, lx)) <= self.fov_half:
                return True
        return False

    def step(self):
        # ① 스캔 발행 + TF
        self.scan_pub.publish(self._build_scan())
        self._publish_tf()

        # ③ 제어 결정
        in_range = self._obstacle_in_range()
        v, steer, mode = 0.0, 0.0, 'STOP'
        if in_range:
            a = self.last_avoid
            if a is not None and a.obstacle_detected and a.points:
                tx, ty = a.points[0].x, a.points[0].y
                dist = math.hypot(tx, ty)
                if dist > 1e-6:
                    # lookahead pure-pursuit: 목표 방향의 lookahead 지점을 조준 → 오프셋 제때 도달
                    ld = max(0.6, min(dist, self.lookahead))
                    ay = ty * ld / dist
                    kappa = 2.0 * ay / (ld * ld)
                else:
                    kappa = 0.0
                steer = max(-self.max_steer, min(self.max_steer, math.atan(self.wheelbase * kappa)))
                v, mode = self.target_speed, 'AVOID'
            else:
                v, steer, mode = self.target_speed, 0.0, 'FWD'

        # ④ 적분 (kinematic bicycle, 후축 기준)
        self.px += v * math.cos(self.pyaw) * self.dt
        self.py += v * math.sin(self.pyaw) * self.dt
        self.pyaw += (v / self.wheelbase) * math.tan(steer) * self.dt
        self.pyaw = math.atan2(math.sin(self.pyaw), math.cos(self.pyaw))

        if v > 0:
            self.trail.append((self.px, self.py))
            if len(self.trail) > 2000:
                self.trail.pop(0)
            self.stopped_logged = False
        elif not self.stopped_logged:
            self.get_logger().info("범위 내 장애물 없음 → 정지(멈춤 유지)")
            self.stopped_logged = True

        self._publish_markers(mode, v, steer)

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.px
        t.transform.translation.y = self.py
        qx, qy, qz, qw = yaw_to_quat(self.pyaw)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf.sendTransform(t)

    def _publish_markers(self, mode, v, steer):
        arr = MarkerArray()
        # 장애물 (원기둥, map)
        for i, (ox, oy) in enumerate(self.obstacles):
            mk = Marker()
            mk.header.frame_id = self.world
            mk.ns, mk.id, mk.type, mk.action = 'obstacles', i, Marker.CYLINDER, Marker.ADD
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = ox, oy, 0.15
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = 2 * self.obst_r
            mk.scale.z = 0.3
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.9, 0.3, 0.2, 0.9
            arr.markers.append(mk)
        # 차량 몸체 (map, 후축 기준 → 기하중심으로 오프셋)
        veh = Marker()
        veh.header.frame_id = self.world
        veh.ns, veh.id, veh.type, veh.action = 'vehicle', 0, Marker.CUBE, Marker.ADD
        cx = self.px + (self.veh_l / 2 - 0.09) * math.cos(self.pyaw)
        cy = self.py + (self.veh_l / 2 - 0.09) * math.sin(self.pyaw)
        veh.pose.position.x, veh.pose.position.y, veh.pose.position.z = cx, cy, 0.1
        qx, qy, qz, qw = yaw_to_quat(self.pyaw)
        veh.pose.orientation.x, veh.pose.orientation.y = qx, qy
        veh.pose.orientation.z, veh.pose.orientation.w = qz, qw
        veh.scale.x, veh.scale.y, veh.scale.z = self.veh_l, self.veh_w, 0.2
        drive = mode != 'STOP'
        veh.color.r = 0.2 if drive else 0.5
        veh.color.g = 0.7 if drive else 0.5
        veh.color.b = 0.9 if drive else 0.5
        veh.color.a = 0.85
        arr.markers.append(veh)
        # 경로 자취
        if len(self.trail) >= 2:
            tr = Marker()
            tr.header.frame_id = self.world
            tr.ns, tr.id, tr.type, tr.action = 'trail', 0, Marker.LINE_STRIP, Marker.ADD
            tr.scale.x = 0.03
            tr.color.g, tr.color.b, tr.color.a = 0.9, 0.5, 0.8
            tr.pose.orientation.w = 1.0
            tr.points = [Point(x=x, y=y, z=0.02) for x, y in self.trail]
            arr.markers.append(tr)
        # 상태 텍스트
        txt = Marker()
        txt.header.frame_id = self.world
        txt.ns, txt.id, txt.type, txt.action = 'status', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = self.px, self.py, 0.7
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.22
        txt.color.r = txt.color.g = txt.color.b = 1.0
        txt.color.a = 1.0
        label = {'AVOID': '회피주행', 'FWD': '직진', 'STOP': '정지(범위내 장애물 없음)'}[mode]
        txt.text = f"{label}  v={v:.2f}  steer={math.degrees(steer):+.0f}deg"
        arr.markers.append(txt)
        self.mk_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = AvoidDriveSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
