"""디버깅용 CSV 프레임 로거. ROS 무의존, 개발 도구 전용(실차 노드에서는 안 씀).

매 프레임 lane_path.LaneEstimate + lane_fit.LaneFitResult의 값을 한 줄씩 기록한다.
스크린샷으로 서로 다른 걸 짐작하는 대신, 이 CSV를 열어서 mode/confidence가
왜 그렇게 나왔는지(예: n_left_candidates=0처럼) 바로 숫자로 확인할 수 있게 하는 게 목적.
"""
from __future__ import annotations

import csv
import math
import time
from pathlib import Path

FIELDNAMES = [
    "frame_idx", "wall_time", "infer_ms",
    "mode", "confidence", "x_m", "y_m", "yaw_deg", "curvature",
    # 2026-08-08 다점 출력 추가 — 실제 몇 점이 나갔는지, 범위가 정상인지
    # 바로 확인하기 위함 (조향 진단의 핵심 변수가 됨).
    "n_points", "points_x_min", "points_x_max", "points_y_min", "points_y_max",
    "points_raw",  # "x:y:yaw:curv|x:y:yaw:curv|..." 전체 점 원본 (디버깅용)
    # REF_POINT_00 근거리 치환 실험 로깅 (2026-08-08, 조향 게인 진단) — 이 프레임에
    # 실제 적용됐는지·최종 x가 얼마였는지를 남겨서, 세 완화안(접선 외삽/신뢰도
    # 게이팅/거리) 중 뭘 켰을 때 str 반응이 어떻게 달라졌는지 사후 대조 가능하게 함.
    "ref_point0_applied", "ref_point0_x",
    "n_left_candidates", "n_right_candidates", "width_m",
    "left_hit_ratio", "left_rms_residual_m", "left_c2", "left_c1", "left_c0",
    "right_hit_ratio", "right_rms_residual_m", "right_c2", "right_c1", "right_c0",
    "is_placeholder_homography",
]


class CsvFrameLogger:
    def __init__(self, path, *, is_placeholder_homography: bool = False):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDNAMES)
        self._writer.writeheader()
        self._frame_idx = 0
        self._is_placeholder = is_placeholder_homography

    def log(self, *, infer_ms: float, estimate, fit_result) -> None:
        row = {
            "frame_idx": self._frame_idx,
            "wall_time": round(time.time(), 3),
            "infer_ms": round(infer_ms, 3),
            "mode": estimate.mode,
            "confidence": round(estimate.confidence, 4),
            "x_m": round(estimate.x, 4),
            "y_m": round(estimate.y, 4),
            "yaw_deg": round(math.degrees(estimate.yaw), 3),
            "curvature": round(estimate.curvature, 5),
            "n_left_candidates": fit_result.n_left_candidates,
            "n_right_candidates": fit_result.n_right_candidates,
            "width_m": round(fit_result.width_m, 4) if fit_result.width_m is not None else "",
            "is_placeholder_homography": self._is_placeholder,
            "ref_point0_applied": getattr(estimate, "ref_point0_applied", False),
            "ref_point0_x": round(getattr(estimate, "ref_point0_x", 0.0), 4),
        }
        points = getattr(estimate, "points", None) or []
        if points:
            xs = [p.x for p in points]
            ys = [p.y for p in points]
            row["n_points"] = len(points)
            row["points_x_min"] = round(min(xs), 3)
            row["points_x_max"] = round(max(xs), 3)
            row["points_y_min"] = round(min(ys), 3)
            row["points_y_max"] = round(max(ys), 3)
            row["points_raw"] = "|".join(
                f"{p.x:.3f}:{p.y:.3f}:{p.yaw:.4f}:{p.curvature:.4f}" for p in points)
        else:
            row["n_points"] = 0
            row["points_x_min"] = row["points_x_max"] = ""
            row["points_y_min"] = row["points_y_max"] = ""
            row["points_raw"] = ""
        for name, side in (("left", fit_result.left), ("right", fit_result.right)):
            if side is not None:
                row[f"{name}_hit_ratio"] = round(side.hit_ratio, 4)
                row[f"{name}_rms_residual_m"] = round(side.rms_residual_m, 5)
                c2, c1, c0 = side.coeffs
                row[f"{name}_c2"], row[f"{name}_c1"], row[f"{name}_c0"] = round(float(c2), 6), round(float(c1), 6), round(float(c0), 6)
            else:
                row[f"{name}_hit_ratio"] = ""
                row[f"{name}_rms_residual_m"] = ""
                row[f"{name}_c2"] = row[f"{name}_c1"] = row[f"{name}_c0"] = ""
        self._writer.writerow(row)
        self._frame_idx += 1

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> "CsvFrameLogger":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
