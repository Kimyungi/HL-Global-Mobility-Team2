#!/usr/bin/env python3
"""각도 눈금 마커 — 라이다 원(raw) 각도 방향을 RViz에 숫자로 표시.

`/angle_labels` 에 방사선 + 0/45/.../315° 텍스트 + 0°(빨강)·90°(초록) 축 화살표를
발행한다. 기본 프레임 = laser_frame(라이다 원각도). RViz fixed frame 을 laser_frame 으로
두면, 정면에 물체를 놓았을 때 그 물체가 **몇 도**에 있는지 바로 읽을 수 있다.
→ 그 각도값이 곧 forward_angle_deg (차량 전방을 가리키는 스캔 각도).
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def _mk(ns, mid, mtype, frame):
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = mtype
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    return m


def _pt(x, y, z=0.0):
    return Point(x=float(x), y=float(y), z=float(z))


class AngleLabels(Node):

    def __init__(self):
        super().__init__('angle_labels')
        self.frame = self.declare_parameter('frame_id', 'laser_frame').value
        self.R = float(self.declare_parameter('radius_m', 2.5).value)
        self.pub = self.create_publisher(MarkerArray, 'angle_labels', 1)
        self.timer = self.create_timer(0.5, self.tick)
        self.get_logger().info(
            f'angle_labels → /angle_labels (frame={self.frame}, R={self.R}m) '
            f'| 0°=빨강 화살표, 90°=초록 화살표')

    def tick(self):
        arr = MarkerArray()
        angles = [0, 45, 90, 135, 180, 225, 270, 315]

        # 방사선 (각 각도로 뻗는 회색 선)
        rays = _mk('rays', 0, Marker.LINE_LIST, self.frame)
        rays.scale.x = 0.01
        rays.color.r = rays.color.g = rays.color.b = 0.5
        rays.color.a = 0.6
        for a in angles:
            rad = math.radians(a)
            rays.points.append(_pt(0, 0))
            rays.points.append(_pt(self.R * math.cos(rad), self.R * math.sin(rad)))
        arr.markers.append(rays)

        # 거리 링 (1m, 2m)
        for j, rr in enumerate((1.0, 2.0)):
            ring = _mk('ring', j, Marker.LINE_STRIP, self.frame)
            ring.scale.x = 0.008
            ring.color.r = ring.color.g = ring.color.b = 0.4
            ring.color.a = 0.5
            for k in range(0, 361, 6):
                rad = math.radians(k)
                ring.points.append(_pt(rr * math.cos(rad), rr * math.sin(rad)))
            arr.markers.append(ring)

        # 0°(빨강, 라이다 +x), 90°(초록, 라이다 +y) 축 화살표
        for a, (r, g, b) in [(0, (1.0, 0.0, 0.0)), (90, (0.0, 1.0, 0.0))]:
            rad = math.radians(a)
            ax = _mk('axis', a, Marker.ARROW, self.frame)
            ax.scale.x, ax.scale.y, ax.scale.z = 0.03, 0.09, 0.14
            ax.points = [_pt(0, 0), _pt(self.R * math.cos(rad), self.R * math.sin(rad))]
            ax.color.r, ax.color.g, ax.color.b, ax.color.a = r, g, b, 0.95
            arr.markers.append(ax)

        # 각도 텍스트
        for i, a in enumerate(angles):
            rad = math.radians(a)
            t = _mk('deg', i, Marker.TEXT_VIEW_FACING, self.frame)
            t.pose.position = _pt(self.R * 1.1 * math.cos(rad), self.R * 1.1 * math.sin(rad), 0.0)
            t.scale.z = 0.28
            t.color.r = t.color.g = t.color.b = 1.0
            t.color.a = 1.0
            t.text = '0 / 360°' if a == 0 else f'{a}°'
            arr.markers.append(t)

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = AngleLabels()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
