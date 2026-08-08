"""픽셀 <-> vehicle frame(지면) 변환.

호모그래피는 calibrate_homography.py가 만든 config/homography.json을 최우선으로
쓰고, 파일이 없으면 카메라 스펙 기반 analytic placeholder로 대체한다 — "캘리브레이션
은 나중에 파라미터로 교체" 설계 (PROJECT_BRIEF.md §9). 실측 파일을 그 경로에 놓기만
하면 이후 코드 변경 없이 자동으로 전환된다.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np

DEFAULT_HOMOGRAPHY_PATH = Path(__file__).resolve().parent.parent / "config" / "homography.json"

# placeholder 기본값: OAK-D Pro 표준 화각 대략치 + PROJECT_BRIEF.md §6 높이(1.40m).
# pitch_deg는 실측 전 추정값 — 알고리즘 검증용이지 실좌표 정확도 보장 아님.
DEFAULT_PLACEHOLDER_PARAMS = dict(
    fx=950.0, fy=950.0, cx=640.0, cy=360.0,
    camera_height_m=1.40, pitch_deg=12.0,
)


def analytic_placeholder_homography(
    fx: float, fy: float, cx: float, cy: float,
    camera_height_m: float, pitch_deg: float,
) -> np.ndarray:
    """평평한 지면 가정 하 pinhole 카메라의 픽셀->지면좌표 호모그래피를 닫힌 형태로 계산.

    유도: 카메라 좌표계 광선 [ (u-cx)/fx, (v-cy)/fy, 1 ]을 world-up 좌표계로 뒤집고
    pitch만큼 회전시켜 지면(Y=0, 카메라 높이=H)과의 교점을 구하면, 결과가 (u,v)의
    유리식이 되고 그 분자/분모가 각각 (u,v,1)의 선형결합이라 정확히 3x3 호모그래피로
    표현된다 (평면 지면 가정 자체는 근사, 이 유도 과정 자체는 근사 아님).
    """
    theta = math.radians(pitch_deg)
    h = camera_height_m
    return np.array([
        [0.0, -h * math.sin(theta) / fy, h * math.cos(theta) + h * math.sin(theta) * cy / fy],
        [-h / fx, 0.0, h * cx / fx],
        [0.0, math.cos(theta) / fy, math.sin(theta) - math.cos(theta) * cy / fy],
    ])


def load_homography(path=None, *, fallback_kwargs: dict | None = None):
    """저장된 호모그래피가 있으면 로드, 없으면 placeholder 생성.

    반환: (H, is_placeholder, meta)
    """
    resolved = Path(path) if path is not None else DEFAULT_HOMOGRAPHY_PATH
    if resolved.exists():
        with open(resolved, encoding="utf-8") as f:
            data = json.load(f)
        H = np.array(data["H"], dtype=np.float64)
        return H, False, data

    kwargs = dict(DEFAULT_PLACEHOLDER_PARAMS)
    if fallback_kwargs:
        kwargs.update(fallback_kwargs)
    H = analytic_placeholder_homography(**kwargs)
    return H, True, {"placeholder_params": kwargs}


class BevGrid:
    """world(x,y)[m] <-> BEV 픽셀(col,row) 변환 상수.

    scale_px_per_m은 우리가 정하는 렌더링 해상도일 뿐 캘리브레이션 결과가 아니다.
    즉 H가 placeholder든 실측이든 이 값은 그대로 — 나중에 실측 H로 교체해도 슬라이딩
    윈도우 파라미터(margin_px, window 개수 등)는 재튜닝할 필요 없다.
    """

    def __init__(self, scale_px_per_m: float = 100.0, x_max_m: float = 6.0, lateral_half_m: float = 3.0):
        self.scale = scale_px_per_m
        self.x_max = x_max_m
        self.lateral_half = lateral_half_m
        self.width = int(2 * lateral_half_m * scale_px_per_m)
        self.height = int(x_max_m * scale_px_per_m)

    def affine_matrix(self) -> np.ndarray:
        s = self.scale
        # world(x,y) -> bev(col,row): x(전방) 클수록 위(row 작음), y(좌측) 클수록 왼쪽(col 작음)
        return np.array([
            [0.0, -s, s * self.lateral_half],
            [-s, 0.0, s * self.x_max],
            [0.0, 0.0, 1.0],
        ])

    def world_to_px(self, x_m, y_m):
        col = -self.scale * y_m + self.scale * self.lateral_half
        row = -self.scale * x_m + self.scale * self.x_max
        return col, row

    def px_to_world(self, col, row):
        y_m = (self.scale * self.lateral_half - col) / self.scale
        x_m = (self.scale * self.x_max - row) / self.scale
        return x_m, y_m


def warp_to_bev(mask: np.ndarray, H: np.ndarray, grid: BevGrid, interp: int = cv2.INTER_NEAREST) -> np.ndarray:
    M = grid.affine_matrix() @ H
    return cv2.warpPerspective(mask.astype(np.uint8), M, (grid.width, grid.height), flags=interp)
