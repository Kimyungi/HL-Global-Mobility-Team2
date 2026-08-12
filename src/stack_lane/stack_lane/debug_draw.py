"""디버그 시각화 오버레이 — node.py(라이브 배포)와 tools/visualize_lane_fit.py
(개발 도구) 양쪽에서 공유. ROS 무의존 (cv2 numpy만 사용).
"""
from __future__ import annotations

import cv2
import numpy as np

from stack_lane.bev import BevGrid
from stack_lane.lane_fit import LaneFitResult
from stack_lane.lane_path import LaneEstimate


def overlay_lane_mask(canvas: np.ndarray, ll_mask: np.ndarray) -> np.ndarray:
    color = np.zeros_like(canvas)
    color[ll_mask == 1] = (0, 0, 255)
    hit = color.any(axis=2)
    blended = canvas.copy()
    blended[hit] = (canvas[hit] * 0.5 + color[hit] * 0.5).astype(np.uint8)
    return blended


def draw_ref_point_on_canvas(vis: np.ndarray, estimate: LaneEstimate, H_inv: np.ndarray) -> None:
    """world(x,y) lookahead 지점을 H_inv로 원본 캔버스 픽셀로 역투영해 표시."""
    if estimate.mode == "none":
        return
    world_pt = np.array([[estimate.x, estimate.y]], dtype=np.float64).reshape(-1, 1, 2)
    px = cv2.perspectiveTransform(world_pt, H_inv).reshape(-1, 2)[0]
    px_i = (int(round(px[0])), int(round(px[1])))
    if 0 <= px_i[0] < vis.shape[1] and 0 <= px_i[1] < vis.shape[0]:
        cv2.drawMarker(vis, px_i, (0, 255, 0), markerType=cv2.MARKER_STAR, markerSize=26, thickness=3)
        cv2.putText(vis, "ref_point", (px_i[0] + 14, px_i[1] + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)


def draw_info_text(vis: np.ndarray, estimate: LaneEstimate, infer_ms: float, is_placeholder: bool) -> None:
    info = (f"{infer_ms:.1f}ms mode={estimate.mode} conf={estimate.confidence:.2f} "
            f"x={estimate.x:.2f} y={estimate.y:.2f} "
            f"yaw={np.degrees(estimate.yaw):.1f}deg kappa={estimate.curvature:.3f}")
    cv2.putText(vis, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    if is_placeholder:
        cv2.putText(vis, "PLACEHOLDER HOMOGRAPHY", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)


def draw_bev_debug(bev_mask: np.ndarray, fit_result: LaneFitResult, grid: BevGrid, lookahead_m: float) -> np.ndarray:
    vis = cv2.cvtColor((bev_mask * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)

    def draw_side(side, color) -> None:
        if side is None:
            return
        cols, rows = grid.world_to_px(side.points_x_m, side.points_y_m)
        for c, r in zip(cols.astype(int), rows.astype(int)):
            cv2.circle(vis, (c, r), 2, color, -1)
        xs_line = np.linspace(0.0, grid.x_max, 50)
        ys_line = np.polyval(side.coeffs, xs_line)
        cols_l, rows_l = grid.world_to_px(xs_line, ys_line)
        pts = np.stack([cols_l, rows_l], axis=1).astype(int)
        cv2.polylines(vis, [pts], False, color, 2)

    draw_side(fit_result.left, (255, 0, 0))
    draw_side(fit_result.right, (0, 0, 255))

    if fit_result.center_coeffs is not None:
        xs_line = np.linspace(0.0, grid.x_max, 50)
        ys_line = np.polyval(fit_result.center_coeffs, xs_line)
        cols_l, rows_l = grid.world_to_px(xs_line, ys_line)
        pts = np.stack([cols_l, rows_l], axis=1).astype(int)
        cv2.polylines(vis, [pts], False, (0, 255, 255), 2)

        la_y = float(np.polyval(fit_result.center_coeffs, lookahead_m))
        la_col, la_row = grid.world_to_px(np.array([lookahead_m]), np.array([la_y]))
        cv2.drawMarker(vis, (int(la_col[0]), int(la_row[0])), (0, 255, 0),
                        markerType=cv2.MARKER_STAR, markerSize=20, thickness=2)

    return vis


def build_debug_frame(canvas: np.ndarray, ll_mask: np.ndarray, estimate: LaneEstimate, debug: dict,
                       grid: BevGrid, lookahead_m: float, H_inv: np.ndarray,
                       infer_ms: float, is_placeholder: bool) -> np.ndarray:
    """원본+오버레이(좌) + BEV 디버그(우) 합친 프레임 하나 생성 — 화면 표시/발행/저장 공용."""
    vis = overlay_lane_mask(canvas, ll_mask)
    draw_info_text(vis, estimate, infer_ms, is_placeholder)
    draw_ref_point_on_canvas(vis, estimate, H_inv)

    bev_vis = draw_bev_debug(debug["bev_mask"], debug["fit"], grid, lookahead_m)
    target_w = max(1, vis.shape[0] * bev_vis.shape[1] // bev_vis.shape[0])
    bev_vis_resized = cv2.resize(bev_vis, (target_w, vis.shape[0]))
    return np.hstack([vis, bev_vis_resized])
