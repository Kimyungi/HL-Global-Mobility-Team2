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
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
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
        # 눈금 간격 [deg]. 기본 45 = 기존 동작 유지. 90 이면 0/90/180/270 만.
        self.step = int(self.declare_parameter('step_deg', 45).value)
        # 전체 색조 [r,g,b] 0~1. 비우면 기존 배색(회색 선 + 흰 글씨 + 빨강/초록 축).
        # 라이다 2대를 동시에 띄울 때 어느 눈금이 어느 라이다 것인지 구분하려고 쓴다.
        # ★ 타입을 명시해야 한다 — 기본값 [] 만 주면 rclpy 가 BYTE_ARRAY 로 추론해
        #   실제 [r,g,b] 를 넣을 때 InvalidParameterTypeException 으로 노드가 죽는다.
        self.color = [float(v) for v in self.declare_parameter(
            'color', [],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)).value] or None
        self.pub = self.create_publisher(MarkerArray, 'angle_labels', 1)
        self.timer = self.create_timer(0.5, self.tick)
        self.get_logger().info(
            f'angle_labels → /angle_labels (frame={self.frame}, R={self.R}m) '
            f'| 0°=빨강 화살표, 90°=초록 화살표')

    def _tint(self, marker, base_gray, alpha):
        """color 파라미터가 있으면 그 색으로, 없으면 기존 회색조 그대로."""
        if self.color:
            marker.color.r, marker.color.g, marker.color.b = self.color[:3]
        else:
            marker.color.r = marker.color.g = marker.color.b = base_gray
        marker.color.a = alpha

    def tick(self):
        arr = MarkerArray()
        angles = list(range(0, 360, max(1, self.step)))

        # 방사선 (각 각도로 뻗는 선)
        rays = _mk('rays', 0, Marker.LINE_LIST, self.frame)
        rays.scale.x = 0.01
        self._tint(rays, 0.5, 0.6)
        for a in angles:
            rad = math.radians(a)
            rays.points.append(_pt(0, 0))
            rays.points.append(_pt(self.R * math.cos(rad), self.R * math.sin(rad)))
        arr.markers.append(rays)

        # 거리 링 (1m, 2m)
        for j, rr in enumerate((1.0, 2.0)):
            ring = _mk('ring', j, Marker.LINE_STRIP, self.frame)
            ring.scale.x = 0.008
            self._tint(ring, 0.4, 0.5)
            for k in range(0, 361, 6):
                rad = math.radians(k)
                ring.points.append(_pt(rr * math.cos(rad), rr * math.sin(rad)))
            arr.markers.append(ring)

        # 0°(빨강, 라이다 +x), 90°(초록, 라이다 +y) 축 화살표.
        # color 를 준 경우엔 그 색으로 통일한다 — 2대를 겹쳐 띄울 때 축 색이 라이다
        # 구분색과 충돌하면 어느 쪽 눈금인지 알 수 없기 때문이다.
        for a, (r, g, b) in [(0, (1.0, 0.0, 0.0)), (90, (0.0, 1.0, 0.0))]:
            rad = math.radians(a)
            ax = _mk('axis', a, Marker.ARROW, self.frame)
            ax.scale.x, ax.scale.y, ax.scale.z = 0.03, 0.09, 0.14
            ax.points = [_pt(0, 0), _pt(self.R * math.cos(rad), self.R * math.sin(rad))]
            if self.color:
                ax.color.r, ax.color.g, ax.color.b = self.color[:3]
            else:
                ax.color.r, ax.color.g, ax.color.b = r, g, b
            ax.color.a = 0.95
            arr.markers.append(ax)

        # 각도 텍스트
        for i, a in enumerate(angles):
            rad = math.radians(a)
            t = _mk('deg', i, Marker.TEXT_VIEW_FACING, self.frame)
            t.pose.position = _pt(self.R * 1.1 * math.cos(rad), self.R * 1.1 * math.sin(rad), 0.0)
            t.scale.z = 0.28
            self._tint(t, 1.0, 1.0)
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
