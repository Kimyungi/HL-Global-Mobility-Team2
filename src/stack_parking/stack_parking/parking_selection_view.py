"""Read-only visualization of the exact candidate footprint selection test."""
import math
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from .t_reference_parking import footprint_hits, inspect_candidate, selection_path


def selection_markers(candidates, cfg, pose, scan, selected=None):
    result = MarkerArray()
    available = scan is not None and len(scan.points) > 0
    findings = [inspect_candidate(c, pose, scan, cfg) if available else (False, 0.)
                for c in candidates]
    def marker(i, kind, color, alpha=1.):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns = 'PARKING_SELECTION_AREA'
        m.id = i
        m.type = kind
        m.pose.orientation.w = 1.
        m.color.r, m.color.g, m.color.b = color
        m.color.a = alpha
        m.lifetime.sec = 1
        result.markers.append(m)
        return m

    for i, candidate in enumerate(candidates):
        inspected = selection_path(candidate, cfg)
        blocked, observed = findings[i]
        eligible = (available and not blocked
                    and (findings[1-i][0] or observed >= cfg.observed_fraction)
                    and (not cfg.enforce_min_radius or candidate.minimum_radius >= cfg.min_radius))
        color = (0., .8, 1.) if i == 0 else (1., .65, 0.)
        area = marker(i*3, Marker.TRIANGLE_LIST, color, .07)
        area.scale.x = area.scale.y = area.scale.z = 1.
        hit = np.zeros(len(scan.points), dtype=bool) if available else None
        for p in inspected:
            c, s = math.cos(p.yaw), math.sin(p.yaw)
            corners = [Point(x=float(p.x+c*x-s*y), y=float(p.y+s*x+c*y), z=.06+i*.015)
                       for x,y in ((-cfg.rear-cfg.margin,-cfg.width/2-cfg.margin),
                                   (cfg.front+cfg.margin,-cfg.width/2-cfg.margin),
                                   (cfg.front+cfg.margin,cfg.width/2+cfg.margin),
                                   (-cfg.rear-cfg.margin,cfg.width/2+cfg.margin))]
            area.points.extend(corners[j] for j in (0,1,2,0,2,3))
            if available:
                hit |= footprint_hits(p, pose, scan, cfg)
        points = marker(i*3+1, Marker.SPHERE_LIST, (1.,0.,0.))
        points.scale.x = points.scale.y = points.scale.z = .10
        if available:
            c, s = math.cos(pose.yaw), math.sin(pose.yaw)
            points.points = [Point(x=float(pose.x+c*x-s*y), y=float(pose.y+s*x+c*y), z=.25)
                             for x,y in scan.points[hit]]
        label = marker(i*3+2, Marker.TEXT_VIEW_FACING, color)
        label.scale.z = .22
        label.pose.position.x = float(inspected[-1].x)
        label.pose.position.y = float(inspected[-1].y)
        label.pose.position.z = .7+i*.35
        state = 'NO SCAN' if not available else ('BLOCKED' if blocked else ('ELIGIBLE' if eligible else 'LOW COVERAGE'))
        label.text = (f'REV {i+1:02d}: {state}' + (' [LOCKED]' if selected == i else '')
                      + f'\nhits={int(hit.sum()) if available else 0} observed={observed:.0%}')
    return result
