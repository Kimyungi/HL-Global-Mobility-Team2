#!/usr/bin/env python3
"""차량·라이다 배치를 RViz 에 그린다 — 숫자로만 있던 장착값을 눈으로 검증한다.

이 파일의 역할:
    lidar_extrinsics.yaml / lidar_mounts.yaml 의 장착값과 FOV 는 지금까지 숫자였다.
    점군이 이상해도 "값이 틀린 건지 센서가 이상한 건지"를 가릴 수단이 없었다.
    이 노드는 **융합 노드와 완전히 같은 파라미터**를 읽어 배치를 그림으로 낸다.

    ★ FOV 부채꼴은 **각 라이다 자기 좌표계(lidar_*_link)** 에 그린다. base_link 로
      직접 계산해서 그리지 않는다 — TF(=장착 yaw)가 틀리면 부채꼴도 같이 틀어져야
      "TF 가 틀렸다"는 사실이 화면에 드러나기 때문이다. base_link 에서 미리 계산해
      그리면 TF 오류를 그림이 덮어버린다.

입력 topic : 없음 (파라미터만 읽는다)
출력 topic : /lidar/vehicle_layout   (visualization_msgs/MarkerArray, latched 아님 · 1Hz)
frame      : 차체·ROI = base_link,  FOV 부채꼴·라벨 = lidar_<id>_link

주요 파라미터 (융합 노드와 같은 이름 = 같은 YAML 을 그대로 먹인다):
    sensor_ids                     ["a1","a2","b1","b2"]
    target_frame                   "base_link"
    extrinsics.<id>.{x,y,z,yaw}    장착값 [m, rad]
    sensors.<id>.fov_enabled       FOV 사용 여부
    sensors.<id>.fov_min_deg/max_deg   센서 기준 시야 [deg] (min>max = ±180 가로지름)
    sensors.<id>.max_range         부채꼴 최대 반지름 산정에 참고
    filter.vehicle_length/width, filter.vehicle_center_x/y, filter.self_margin
    filter.roi_{min,max}_{x,y}     ROI 사각형
  이 노드 전용:
    wedge_radius_m   1.5    부채꼴 반지름 [m] (0 = max_range 사용)
    show_roi         true
    period_s         1.0

관계: multi_lidar_fusion.launch.py 가 융합 노드와 **같은 parameters 목록**으로 띄운다.
      RViz 설정(multi_lidar.rviz)의 "vehicle layout" 디스플레이가 이 토픽을 본다.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

# multi_lidar.rviz 의 센서별 점군 색과 맞춘다 — 같은 센서는 화면에서 같은 색이어야
# "이 부채꼴이 저 점군의 것"이라는 대응이 바로 보인다.
COLORS = {
    'a1': (1.00, 0.24, 0.24),   # red
    'a2': (1.00, 0.78, 0.16),   # yellow
    'b1': (0.24, 0.78, 1.00),   # cyan
    'b2': (0.47, 1.00, 0.47),   # green
}
FALLBACK_COLOR = (0.8, 0.8, 0.8)

# 슬롯 -> 사람이 읽는 위치 이름. 라벨에만 쓴다.
POSITION_NAME = {'a1': 'front', 'a2': 'rear', 'b1': 'left', 'b2': 'right'}


def _pt(x, y, z=0.0):
    return Point(x=float(x), y=float(y), z=float(z))


def _wrap_deg(a):
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


class LayoutMarkers(Node):

    def __init__(self):
        super().__init__('layout_markers')

        self.ids = self.declare_parameter('sensor_ids', ['a1', 'a2', 'b1', 'b2']).value
        self.base = self.declare_parameter('target_frame', 'base_link').value
        self.wedge_r = float(self.declare_parameter('wedge_radius_m', 1.5).value)
        self.show_roi = bool(self.declare_parameter('show_roi', True).value)
        period = float(self.declare_parameter('period_s', 1.0).value)

        # 차체 / self filter
        self.veh_l = float(self.declare_parameter('filter.vehicle_length', 0.85).value)
        self.veh_w = float(self.declare_parameter('filter.vehicle_width', 0.62).value)
        self.veh_cx = float(self.declare_parameter('filter.vehicle_center_x', 0.335).value)
        self.veh_cy = float(self.declare_parameter('filter.vehicle_center_y', 0.0).value)
        self.margin = float(self.declare_parameter('filter.self_margin', 0.02).value)

        self.roi = {
            k: float(self.declare_parameter('filter.roi_' + k, d).value)
            for k, d in (('min_x', -5.0), ('max_x', 10.0), ('min_y', -5.0), ('max_y', 5.0))
        }

        # 센서별 장착값 + FOV
        self.sensors = {}
        for sid in self.ids:
            e = 'extrinsics.' + sid
            s = 'sensors.' + sid
            self.sensors[sid] = {
                'x': float(self.declare_parameter(e + '.x', 0.0).value),
                'y': float(self.declare_parameter(e + '.y', 0.0).value),
                'z': float(self.declare_parameter(e + '.z', 0.0).value),
                'yaw': float(self.declare_parameter(e + '.yaw', 0.0).value),
                'frame': self.declare_parameter(s + '.frame_id', 'lidar_' + sid + '_link').value,
                'fov_on': bool(self.declare_parameter(s + '.fov_enabled', False).value),
                'fov_min': float(self.declare_parameter(s + '.fov_min_deg', -180.0).value),
                'fov_max': float(self.declare_parameter(s + '.fov_max_deg', 180.0).value),
                'max_range': float(self.declare_parameter(s + '.max_range', 12.0).value),
            }

        self.pub = self.create_publisher(MarkerArray, 'vehicle_layout', 1)
        self.create_timer(period, self.tick)

        for sid in self.ids:
            c = self.sensors[sid]
            self.get_logger().info(
                f'{sid} ({POSITION_NAME.get(sid, "?")}): '
                f'xy=({c["x"]:+.3f}, {c["y"]:+.3f}) yaw={math.degrees(c["yaw"]):+.1f}deg '
                f'FOV={"%.0f~%.0f" % (c["fov_min"], c["fov_max"]) if c["fov_on"] else "360"}')

    # ── 마커 만들기 ────────────────────────────────────────────────────
    def _mk(self, ns, mid, mtype, frame):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _rect(self, ns, mid, cx, cy, length, width, rgb, alpha, width_line, z=0.0):
        m = self._mk(ns, mid, Marker.LINE_STRIP, self.base)
        m.scale.x = width_line
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = alpha
        hl, hw = length / 2.0, width / 2.0
        for dx, dy in ((+hl, +hw), (+hl, -hw), (-hl, -hw), (-hl, +hw), (+hl, +hw)):
            m.points.append(_pt(cx + dx, cy + dy, z))
        return m

    def _wedge(self, sid, cfg, mid):
        """FOV 부채꼴 — **센서 좌표계** 에 그린다 (TF 검증이 목적, 위 docstring 참조)."""
        m = self._mk('fov', mid, Marker.TRIANGLE_LIST, cfg['frame'])
        rgb = COLORS.get(sid, FALLBACK_COLOR)
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = 0.13          # 점군을 가리지 않을 정도로만
        m.scale.x = m.scale.y = m.scale.z = 1.0

        r = self.wedge_r if self.wedge_r > 0.0 else cfg['max_range']
        if cfg['fov_on']:
            lo, hi = _wrap_deg(cfg['fov_min']), _wrap_deg(cfg['fov_max'])
            # min > max = ±180 을 가로지르는 구간. 위로 한 바퀴 펴서 훑는다.
            if lo > hi:
                hi += 360.0
        else:
            lo, hi = -180.0, 180.0

        step = 2.0
        a = lo
        while a < hi - 1e-9:
            a2 = min(a + step, hi)
            m.points.append(_pt(0, 0))
            m.points.append(_pt(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))))
            m.points.append(_pt(r * math.cos(math.radians(a2)), r * math.sin(math.radians(a2))))
            a = a2
        return m

    def tick(self):
        arr = MarkerArray()
        mid = 0

        # ── 차체 외곽 (실측 제원) ──
        arr.markers.append(self._rect(
            'body', mid, self.veh_cx, self.veh_cy, self.veh_l, self.veh_w,
            (0.9, 0.9, 0.95), 0.9, 0.012))
        mid += 1
        # ── self filter 박스 (여기 들어온 점은 지워진다) ──
        arr.markers.append(self._rect(
            'body', mid, self.veh_cx, self.veh_cy,
            self.veh_l + 2 * self.margin, self.veh_w + 2 * self.margin,
            (1.0, 0.4, 0.4), 0.5, 0.006))
        mid += 1

        # ── 후축(원점) 표시 — 모든 장착값의 기준점 ──
        axle = self._mk('body', mid, Marker.LINE_LIST, self.base)
        mid += 1
        axle.scale.x = 0.01
        axle.color.r, axle.color.g, axle.color.b, axle.color.a = 0.6, 0.9, 0.6, 0.9
        axle.points += [_pt(0, -self.veh_w / 2), _pt(0, self.veh_w / 2)]
        arr.markers.append(axle)

        origin = self._mk('body', mid, Marker.TEXT_VIEW_FACING, self.base)
        mid += 1
        origin.pose.position = _pt(0.0, -self.veh_w / 2 - 0.12, 0.0)
        origin.scale.z = 0.07
        origin.color.r = origin.color.g = origin.color.b = origin.color.a = 1.0
        origin.text = 'base_link = 후축 중심'
        arr.markers.append(origin)

        # ── ROI 사각형 ──
        if self.show_roi:
            arr.markers.append(self._rect(
                'roi', mid,
                (self.roi['min_x'] + self.roi['max_x']) / 2.0,
                (self.roi['min_y'] + self.roi['max_y']) / 2.0,
                self.roi['max_x'] - self.roi['min_x'],
                self.roi['max_y'] - self.roi['min_y'],
                (0.5, 0.5, 0.6), 0.45, 0.01))
            mid += 1

        # ── 라이다 4대 ──
        for sid in self.ids:
            cfg = self.sensors[sid]
            rgb = COLORS.get(sid, FALLBACK_COLOR)

            arr.markers.append(self._wedge(sid, cfg, mid))
            mid += 1

            # 유닛 몸체 (센서 좌표계 원점)
            body = self._mk('unit', mid, Marker.CYLINDER, cfg['frame'])
            mid += 1
            body.scale.x = body.scale.y = 0.055
            body.scale.z = 0.03
            body.color.r, body.color.g, body.color.b = rgb
            body.color.a = 0.95
            arr.markers.append(body)

            # 이 유닛의 스캔 0도 방향 — 화면에서 "정면"과 헷갈리기 쉬운 값이라 명시한다.
            zero = self._mk('zero_deg', mid, Marker.ARROW, cfg['frame'])
            mid += 1
            zero.scale.x, zero.scale.y, zero.scale.z = 0.012, 0.03, 0.05
            zero.color.r, zero.color.g, zero.color.b = rgb
            zero.color.a = 0.95
            zero.points = [_pt(0, 0), _pt(0.35, 0)]
            arr.markers.append(zero)

            label = self._mk('label', mid, Marker.TEXT_VIEW_FACING, cfg['frame'])
            mid += 1
            label.pose.position = _pt(0.0, 0.0, 0.16)
            label.scale.z = 0.075
            label.color.r, label.color.g, label.color.b = rgb
            label.color.a = 1.0
            fov = (f'{_wrap_deg(cfg["fov_min"]):.0f}~{_wrap_deg(cfg["fov_max"]):.0f}'
                   if cfg['fov_on'] else '360')
            yaw_deg = math.degrees(cfg['yaw'])
            label.text = (f'{sid} {POSITION_NAME.get(sid, "")}\n'
                          f'({cfg["x"]:+.3f}, {cfg["y"]:+.3f}) yaw {yaw_deg:+.0f}\n'
                          f'FOV {fov}')
            arr.markers.append(label)

        self.pub.publish(arr)


def main():
    rclpy.init()
    node = LayoutMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
