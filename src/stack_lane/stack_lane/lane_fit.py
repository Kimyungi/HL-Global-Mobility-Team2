"""BEV 이진 차선 마스크 -> 좌/우 차선 후보 선택 -> 다항식 피팅 -> 중심선.

설계 근거 (대화 로그, PROJECT_BRIEF.md §9):
  - 근거리(하단) 히스토그램에서 임계값 넘는 "모든" 봉우리를 후보로 남긴다
    (top-2만 보지 않음 — 인접 차로 라인을 내 차선으로 오선택하는 것 방지).
  - 중심(y=0, BEV 중앙열) 기준 좌/우로 나눠, 각 편에서 중심에 가까운 후보부터
    순서대로 짝지어보며 차선폭(3.7m, 오차범위 이내) 검증을 통과하는 첫 조합을 채택.
  - 검증 통과 조합이 없으면 편측(중심에 가장 가까운 후보 하나) 검출로 폴백.
  - 편측만 검출된 경우, 차선폭 절반(1.85m)을 안쪽으로 오프셋해 중심선을 추정한다
    (신뢰도는 lane_path.py의 confidence 계산에서 페널티를 받는다).
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from stack_lane.bev import BevGrid

LANE_WIDTH_M = 3.7  # PROJECT_BRIEF.md §6 확정값


@dataclass
class SideFit:
    coeffs: np.ndarray          # np.polyfit(deg=2) 결과 [c2, c1, c0], y_m = c2 x^2 + c1 x + c0
    points_x_m: np.ndarray
    points_y_m: np.ndarray
    hit_ratio: float            # 슬라이딩 윈도우 중 유효(minpix 이상) 검출 비율
    rms_residual_m: float       # 포인트-피팅 잔차 RMS [m]


@dataclass
class LaneFitResult:
    mode: str                            # 'both' | 'left_only' | 'right_only' | 'none'
    center_coeffs: np.ndarray | None
    left: SideFit | None
    right: SideFit | None
    width_m: float | None
    n_left_candidates: int = 0           # 진단용 — 0이면 좌측 탐색범위에 픽셀 자체가 없었다는 뜻
    n_right_candidates: int = 0


def filter_lane_like_components(bev_mask: np.ndarray, grid: BevGrid, *,
                                 min_forward_extent_m: float = 1.0,
                                 max_lateral_to_forward_ratio: float = 1.5) -> np.ndarray:
    """진행방향과 거의 수직인 마킹(과속방지턱·정지선·횡단보도 등)을 제거한다.

    2026-08-07 실주행에서 발견: 과속방지턱 줄무늬를 차선 후보로 잘못 집어서
    경로가 프레임마다 크게 튀는 문제가 있었음. 실제 차선 경계는 BEV에서 전방
    (row) 방향으로 길게 이어지는 반면, 저런 마킹은 좌우(col)로만 넓고 전방으로는
    짧다 — 연결요소(connected component)별 bounding box 가로세로 비로 걸러낸다.

    한계: 차선과 수직 마킹이 마스크 상에서 서로 붙어(touch) 있으면 하나의
    연결요소로 합쳐져 이 필터를 통과할 수 있음 — 완전한 해법은 아니고 1차 방어선.
    """
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(bev_mask, connectivity=8)
    keep = np.zeros_like(bev_mask)
    min_forward_extent_px = min_forward_extent_m * grid.scale
    for label in range(1, num_labels):  # 0 = 배경
        _x, _y, width, height, _area = stats[label]
        forward_extent = height  # row 방향 = 전방
        lateral_extent = width   # col 방향 = 좌우
        if forward_extent < min_forward_extent_px:
            continue  # 전방으로 너무 짧음 (차선이라기엔 조각남)
        if lateral_extent > forward_extent * max_lateral_to_forward_ratio:
            continue  # 좌우로 더 넓음 -> 수직 마킹으로 판단, 제외
        keep[labels == label] = 1
    return keep


def _find_base_candidates(hist: np.ndarray, threshold: float, min_width: int = 3, merge_gap: int = 10) -> list[int]:
    """근거리 히스토그램에서 임계값 넘는 모든 봉우리(후보 컬럼)를 찾는다."""
    above = hist >= threshold
    candidates: list[int] = []
    w = len(hist)
    col = 0
    while col < w:
        if not above[col]:
            col += 1
            continue
        start = col
        end = col
        gap = 0
        while col < w and (above[col] or gap < merge_gap):
            if above[col]:
                end = col
                gap = 0
            else:
                gap += 1
            col += 1
        if end - start + 1 >= min_width:
            seg = hist[start:end + 1].astype(np.float64)
            cols = np.arange(start, end + 1)
            candidates.append(int(np.average(cols, weights=seg)))
    return candidates


def _climb_windows(bev_mask: np.ndarray, base_col: int, n_windows: int, margin: int, minpix: int,
                    max_window_spread_px: float | None = None):
    """윈도우별로 위로 올라가며 픽셀을 모은다.

    max_window_spread_px: 한 윈도우 안에서 검출된 픽셀의 좌우 폭이 이 값을 넘으면
    그 윈도우를 통째로 버린다(중심 갱신도, 점 누적도 안 함). 진행방향에 수직인
    마킹(과속방지턱 등)이 차선과 마스크 상에서 맞닿아 있으면 연결요소 필터만으론
    못 걸러지는데(2026-08-07 실주행에서 발견 — 맞닿으면 하나의 연결요소로 합쳐짐),
    그 마킹이 걸리는 윈도우는 폭이 비정상적으로 넓어지므로 여기서 잡아낼 수 있다.
    """
    h, _w = bev_mask.shape
    window_h = max(1, h // n_windows)
    ys_all, xs_all = bev_mask.nonzero()
    current_col = base_col
    xs: list[int] = []
    ys: list[int] = []
    hits = 0
    for i in range(n_windows):
        row_hi = h - i * window_h
        row_lo = h - (i + 1) * window_h
        col_lo = current_col - margin
        col_hi = current_col + margin
        sel = (ys_all >= row_lo) & (ys_all < row_hi) & (xs_all >= col_lo) & (xs_all < col_hi)
        n = int(sel.sum())
        if n < minpix:
            continue
        window_xs = xs_all[sel]
        spread = int(window_xs.max() - window_xs.min())
        if max_window_spread_px is not None and spread > max_window_spread_px:
            continue  # 오염된 윈도우로 판단 — 중심 유지, 점 누적 안 함
        xs.extend(window_xs.tolist())
        ys.extend(ys_all[sel].tolist())
        current_col = int(window_xs.mean())
        hits += 1
    hit_ratio = hits / n_windows
    return np.array(xs), np.array(ys), hit_ratio


def _fit_side(bev_mask: np.ndarray, base_col: int, grid: BevGrid,
              n_windows: int, margin: int, minpix: int,
              max_window_spread_m: float | None = 0.5) -> SideFit | None:
    max_spread_px = max_window_spread_m * grid.scale if max_window_spread_m is not None else None
    px_x, px_y, hit_ratio = _climb_windows(bev_mask, base_col, n_windows, margin, minpix, max_spread_px)
    if len(px_x) < 3:
        return None
    x_m, y_m = grid.px_to_world(px_x, px_y)
    order = np.argsort(x_m)
    x_m, y_m = x_m[order], y_m[order]
    try:
        coeffs = np.polyfit(x_m, y_m, deg=2)
    except np.linalg.LinAlgError:
        return None
    residual = y_m - np.polyval(coeffs, x_m)
    rms = float(np.sqrt(np.mean(residual ** 2)))
    return SideFit(coeffs=coeffs, points_x_m=x_m, points_y_m=y_m, hit_ratio=hit_ratio, rms_residual_m=rms)


def fit_lane(bev_mask: np.ndarray, grid: BevGrid, *,
             strip_frac: float = 1.0, hist_threshold: float = 15.0,
             n_windows: int = 9, margin_px: int = 60, minpix: int = 40,
             width_tolerance_m: float = 0.6,
             filter_perpendicular_markings: bool = True,
             max_window_spread_m: float | None = 0.5) -> LaneFitResult:
    """strip_frac: 후보 탐색에 쓸 BEV 하단(근거리) 비율. 기본 1.0(전체 높이) —
    카메라 최소 가시거리(예: 2.5m)로 인한 근거리 사각지대가 있으면 좁은 근거리
    스트립만 볼 경우 그 안에 픽셀이 아예 없어 후보가 0개로 잡히는 문제가 있었음
    (2026-08-07 실차 테스트에서 발견). 사각지대 위치를 몰라도 항상 실제 차선이
    있는 구간을 잡도록 전체 높이를 기본값으로 바꿈.

    filter_perpendicular_markings: 과속방지턱·정지선 등 진행방향에 수직인 마킹을
    후보에서 제외 (2026-08-07 실주행에서 발견 — strip_frac을 전체 높이로 넓힌 부작용
    으로 이런 마킹까지 후보 히스토그램에 들어와 경로가 프레임마다 크게 튀는 문제 있었음).
    """
    if filter_perpendicular_markings:
        bev_mask = filter_lane_like_components(bev_mask, grid)

    h, w = bev_mask.shape
    strip = bev_mask[int(h * (1 - strip_frac)):, :]
    hist = strip.sum(axis=0)
    candidates = _find_base_candidates(hist, threshold=hist_threshold)

    center_col = w / 2.0
    left_cands = sorted([c for c in candidates if c < center_col], key=lambda c: center_col - c)
    right_cands = sorted([c for c in candidates if c > center_col], key=lambda c: c - center_col)
    n_left, n_right = len(left_cands), len(right_cands)

    lane_width_px = LANE_WIDTH_M * grid.scale
    tol_px = width_tolerance_m * grid.scale

    def try_side(col: int) -> SideFit | None:
        return _fit_side(bev_mask, col, grid, n_windows, margin_px, minpix, max_window_spread_m)

    # 1) 폭 검증 통과하는 좌우 조합 탐색 (중심에 가까운 후보부터, greedy)
    for lc in left_cands:
        for rc in right_cands:
            if abs((rc - lc) - lane_width_px) <= tol_px:
                left_fit = try_side(lc)
                right_fit = try_side(rc)
                if left_fit is not None and right_fit is not None:
                    center = (left_fit.coeffs + right_fit.coeffs) / 2.0
                    width_m = (rc - lc) / grid.scale
                    return LaneFitResult("both", center, left_fit, right_fit, width_m, n_left, n_right)

    # 2) 폭 맞는 조합 없음 -> 편측 폴백 (중심에 가장 가까운 후보 하나)
    if left_cands:
        left_fit = try_side(left_cands[0])
        if left_fit is not None:
            center = left_fit.coeffs.copy()
            center[-1] -= LANE_WIDTH_M / 2.0  # 상수항만 이동 = 곡선 형태 유지한 채 평행이동
            return LaneFitResult("left_only", center, left_fit, None, None, n_left, n_right)

    if right_cands:
        right_fit = try_side(right_cands[0])
        if right_fit is not None:
            center = right_fit.coeffs.copy()
            center[-1] += LANE_WIDTH_M / 2.0
            return LaneFitResult("right_only", center, None, right_fit, None, n_left, n_right)

    return LaneFitResult("none", None, None, None, None, n_left, n_right)
