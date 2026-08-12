#!/usr/bin/env python3
"""회피 출력 시각화 — `/perception/avoid` 를 RViz 마커로 그린다.

corridor(감지 통로)·감지거리·FOV·회피 목표점·상태 텍스트를 base_link 프레임
MarkerArray(`/avoid_markers`)로 발행. 장애물을 옮기며(fake_scan 또는 실라이다)
회피 목표점이 올바른 쪽에 찍히는지, narrow_gap/avoidable 이 맞게 뜨는지 눈으로 확인.

파라미터 기본값은 config/params.yaml 실측값과 동일하게 맞춘다(런치에서 주입).
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from fma_interfaces.msg import AvoidStatus, TargetRef


def _mk(ns, mid, mtype, frame='base_link'):
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


class AvoidViz(Node):

    def __init__(self):
        super().__init__('avoid_viz')
        self.lidar_x = float(self.declare_parameter('lidar_x_m', 0.76).value)
        self.vehicle_width = float(self.declare_parameter('vehicle_width_m', 0.62).value)
        self.lateral_margin = float(self.declare_parameter('lateral_margin_m', 0.15).value)
        self.detect_range = float(self.declare_parameter('detect_range_m', 3.0).value)
        self.offset_max = float(self.declare_parameter('offset_max_m', 1.0).value)
        self.roi_angle = float(self.declare_parameter('roi_angle_deg', 180.0).value)
        self.corr = self.vehicle_width / 2.0 + self.lateral_margin
        self.half = math.radians(self.roi_angle / 2.0)

        self.pub = self.create_publisher(MarkerArray, 'avoid_markers', 1)
        self.sub = self.create_subscription(AvoidStatus, '/perception/avoid', self.cb, 10)
        # 실제로 dSPACE 로 나가는 ref 점. 인지 목표점과 다를 수 있다(당김 방식) —
        # RViz 에 목표점만 그리면 "바뀐 게 없어 보이는" 착시가 생겨서 같이 그린다.
        self.last_ref = None
        self.ref_sub = self.create_subscription(
            TargetRef, '/adas/target_ref', self._on_ref, 1)
        self.get_logger().info(
            f"avoid_viz → /avoid_markers | corridor ±{self.corr:.2f}m, "
            f"detect {self.detect_range}m, FOV ±{math.degrees(self.half):.0f}deg")

    def _on_ref(self, msg: TargetRef):
        self.last_ref = msg.ref_points[0] if msg.ref_points else None

    def cb(self, msg: AvoidStatus):
        arr = MarkerArray()
        cx = self.lidar_x

        # corridor (감지 통로 좌우 경계)
        corr = _mk('corridor', 0, Marker.LINE_LIST)
        corr.scale.x = 0.02
        corr.color.r, corr.color.g, corr.color.b, corr.color.a = 0.6, 0.6, 0.6, 0.8
        for s in (+1.0, -1.0):
            corr.points.append(_pt(cx, s * self.corr))
            corr.points.append(_pt(cx + self.detect_range, s * self.corr))
        arr.markers.append(corr)

        # FOV 부채꼴 (감지거리 반경 + 좌우 경계)
        fov = _mk('fov', 1, Marker.LINE_STRIP)
        fov.scale.x = 0.015
        fov.color.r, fov.color.g, fov.color.b, fov.color.a = 0.2, 0.5, 1.0, 0.7
        fov.points.append(_pt(cx, 0.0))
        steps = 48
        for i in range(steps + 1):
            a = -self.half + (2.0 * self.half) * i / steps
            fov.points.append(_pt(cx + self.detect_range * math.cos(a),
                                  self.detect_range * math.sin(a)))
        fov.points.append(_pt(cx, 0.0))
        arr.markers.append(fov)

        # 회피 목표점 (있으면 초록 구, 없으면 삭제)
        tgt = _mk('target', 2, Marker.SPHERE)
        tgt.scale.x = tgt.scale.y = tgt.scale.z = 0.18
        arw = _mk('to_target', 3, Marker.ARROW)
        arw.scale.x, arw.scale.y, arw.scale.z = 0.03, 0.08, 0.12
        if msg.points:
            p = msg.points[0]
            tgt.pose.position.x, tgt.pose.position.y = float(p.x), float(p.y)
            tgt.color.g, tgt.color.a = 1.0, 0.95
            arw.points = [_pt(cx, 0.0), _pt(p.x, p.y)]
            arw.color.g, arw.color.b, arw.color.a = 0.8, 0.3, 0.9
        else:
            tgt.action = Marker.DELETE
            arw.action = Marker.DELETE
        arr.markers.append(tgt)
        arr.markers.append(arw)

        # 상태 텍스트
        txt = _mk('status', 4, Marker.TEXT_VIEW_FACING)
        txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = cx, 0.0, 0.6
        txt.scale.z = 0.16
        txt.color.r = txt.color.g = txt.color.b = 1.0
        txt.color.a = 0.95
        state = 'DETECTED' if msg.obstacle_detected else 'clear'
        tgt_s = (f"({msg.points[0].x:.2f}, {msg.points[0].y:+.2f})"
                 if msg.points else '-')
        txt.text = (f"{state} | ttc {msg.ttc:.2f}s | narrow_gap {msg.narrow_gap} | "
                    f"avoidable {msg.avoidable}\n"
                    f"target {tgt_s} | v_suggest {msg.v_suggest:.2f}")
        arr.markers.append(txt)

        # ── 차량 정면(+x) 화살표 + 좌/우 라벨 ──
        # 조향 방향 뒤집힘을 눈으로 가리기 위한 기준. base_link 규약(REP-103):
        #   +x = 전방, +y = 좌측, 좌회전 곡률 κ > 0.
        # 회피 목표점 y 가 +면 화살표 왼쪽으로 벗어나야 정상이다.
        fwd = _mk('vehicle_forward', 5, Marker.ARROW)
        fwd.scale.x, fwd.scale.y, fwd.scale.z = 0.04, 0.10, 0.15
        fwd.color.r, fwd.color.g, fwd.color.b, fwd.color.a = 1.0, 0.9, 0.1, 1.0
        fwd.points = [_pt(0.0, 0.0), _pt(1.0, 0.0)]      # 후축 원점 → 전방 1m
        arr.markers.append(fwd)

        # ── 실제 송신 ref 점 (당김 적용 후) ──
        # 인지 목표점(초록)과 별개. 이게 dSPACE 가 실제로 받는 점이다.
        if self.last_ref is not None:
            r = self.last_ref
            sent = _mk('sent_ref', 8, Marker.SPHERE)
            sent.pose.position.x, sent.pose.position.y = float(r.x), float(r.y)
            sent.scale.x = sent.scale.y = sent.scale.z = 0.16
            sent.color.r, sent.color.g, sent.color.b, sent.color.a = 1.0, 0.35, 0.0, 0.95
            arr.markers.append(sent)

            line = _mk('sent_ref', 9, Marker.LINE_LIST)
            line.scale.x = 0.025
            line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.35, 0.0, 0.8
            line.points = [_pt(0.0, 0.0), _pt(r.x, r.y)]
            arr.markers.append(line)

            st = _mk('sent_ref', 10, Marker.TEXT_VIEW_FACING)
            st.pose.position.x, st.pose.position.y = float(r.x), float(r.y)
            st.pose.position.z = 0.35
            st.scale.z = 0.13
            st.color.r, st.color.g, st.color.b, st.color.a = 1.0, 0.6, 0.2, 0.95
            deg = math.degrees(math.atan(0.595 * r.curvature))
            st.text = (f"SENT ({r.x:.2f}, {r.y:+.2f})\n"
                       f"k {r.curvature:+.3f} = {deg:+.1f}deg")
            arr.markers.append(st)

        for mid, side, y in ((6, 'LEFT (+y)', 0.7), (7, 'RIGHT (-y)', -0.7)):
            lab = _mk('side_label', mid, Marker.TEXT_VIEW_FACING)
            lab.pose.position.x, lab.pose.position.y = 0.9, y
            lab.pose.position.z = 0.25
            lab.scale.z = 0.14
            lab.color.r = lab.color.g = lab.color.b = 0.9
            lab.color.a = 0.9
            lab.text = side
            arr.markers.append(lab)

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = AvoidViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
