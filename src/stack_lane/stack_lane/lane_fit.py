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

# 3.7 → 4.06 (2026-08-15 실차 실측). PROJECT_BRIEF.md §6의 3.7m은 도면·규격값이고,
# 실제 코스를 BEV로 재면 **4.072 ± 0.11m** 였다 (run_0815_181526·182032 합쳐
# both 검출 1900프레임, 범위 3.65~4.30).
#
# 이 상수는 두 군데에 쓰이며 둘 다 어긋나 있었다:
#   ① 편측 검출 시 중심선 평행이동량 (LANE_WIDTH_M/2). 1.85m를 쓰는데 실제
#      절반은 2.03m — 편측 프레임마다 0.18m 계통 오차가 나고, left_only ↔
#      right_only 가 뒤집히면 **0.36m 계단 점프**가 된다. 실측 run에서
#      편측이 46%(right_only 32% + left_only 13%)라 상시 발생했다.
#   ② 후보 쌍 수용창 (LANE_WIDTH_M ± width_tolerance_m = 3.7±0.6 → 3.10~4.30m).
#      실제 폭 4.07이 상한 4.30에 너무 가까워, **정상 쌍이 거부되어 편측으로
#      떨어지고 있었다** — width_m 히스토그램이 4.30에서 칼같이 잘려 있고
#      (4.25m 이상이 6.6%) 그 위 꼬리가 없다. 4.06으로 옮기면 수용창이
#      3.46~4.66이 되어 실측 분포를 대칭으로 감싼다.
#
# ⚠ 주의: 이 값은 **BEV 좌표에서 잰 폭**과 맞춰야 한다. 호모그래피 배율이
# 설령 틀렸더라도 width_m과 이 상수가 같은 공간에 있으면 편측/양측 경로가
# 서로 일치한다 — 그래서 도면값(3.7)이 아니라 실측 BEV값을 쓴다.
# 실제 노면을 줄자로 재서 3.7m가 맞다면 호모그래피 배율이 ~10% 부풀려진
# 것이므로 재캘리브레이션이 필요하다(이 상수와는 별개 문제).
LANE_WIDTH_M = 4.06


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
                    max_window_spread_px: float | None = None, row_start: int | None = None):
    """윈도우별로 위로 올라가며 픽셀을 모은다.

    max_window_spread_px: 한 윈도우 안에서 검출된 픽셀의 좌우 폭이 이 값을 넘으면
    그 윈도우를 통째로 버린다(중심 갱신도, 점 누적도 안 함). 진행방향에 수직인
    마킹(과속방지턱 등)이 차선과 마스크 상에서 맞닿아 있으면 연결요소 필터만으론
    못 걸러지는데(2026-08-07 실주행에서 발견 — 맞닿으면 하나의 연결요소로 합쳐짐),
    그 마킹이 걸리는 윈도우는 폭이 비정상적으로 넓어지므로 여기서 잡아낼 수 있다.

    row_start: 클라이밍을 시작할 행(row) — 기본(None)은 이미지 맨 아래(bev_mask.shape[0],
    즉 x=0m, 차량 바로 앞)부터. 2026-08-08 발견: 카메라 최소 가시거리(기본 2.5m) 안쪽은
    항상 빈 구간인데도 여기서부터 세면 n_windows 중 상당수가 처음부터 무조건 빈 윈도우가
    됨(예: 9개 중 3개, 실측 hit_ratio가 완벽한 조건에서도 ~0.67을 못 넘던 원인). 호출부
    (_fit_side)가 실제 가시거리에 해당하는 row를 계산해 넘겨주면, n_windows 전부가 실제로
    뭔가 보일 수 있는 구간에만 배치된다.
    """
    h, _w = bev_mask.shape
    if row_start is None:
        row_start = h
    window_h = max(1, row_start // n_windows)
    ys_all, xs_all = bev_mask.nonzero()
    current_col = base_col
    xs: list[int] = []
    ys: list[int] = []
    hits = 0
    for i in range(n_windows):
        row_hi = row_start - i * window_h
        row_lo = row_start - (i + 1) * window_h
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
              max_window_spread_m: float | None = 0.5,
              row_start: int | None = None) -> SideFit | None:
    max_spread_px = max_window_spread_m * grid.scale if max_window_spread_m is not None else None
    px_x, px_y, hit_ratio = _climb_windows(bev_mask, base_col, n_windows, margin, minpix, max_spread_px,
                                            row_start=row_start)
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
             max_window_spread_m: float | None = 0.5,
             visible_x_min_m: float | None = 2.5,
             max_c2_diff: float = 0.15) -> LaneFitResult:
    """strip_frac: 후보 탐색에 쓸 BEV 하단(근거리) 비율. 기본 1.0(전체 높이) —
    카메라 최소 가시거리(예: 2.5m)로 인한 근거리 사각지대가 있으면 좁은 근거리
    스트립만 볼 경우 그 안에 픽셀이 아예 없어 후보가 0개로 잡히는 문제가 있었음
    (2026-08-07 실차 테스트에서 발견). 사각지대 위치를 몰라도 항상 실제 차선이
    있는 구간을 잡도록 전체 높이를 기본값으로 바꿈.

    filter_perpendicular_markings: 과속방지턱·정지선 등 진행방향에 수직인 마킹을
    후보에서 제외 (2026-08-07 실주행에서 발견 — strip_frac을 전체 높이로 넓힌 부작용
    으로 이런 마킹까지 후보 히스토그램에 들어와 경로가 프레임마다 크게 튀는 문제 있었음).

    visible_x_min_m: 슬라이딩 윈도우 클라이밍을 시작할 최소 x — 기본 2.5m(카메라
    최소 가시거리, points_x_start와 동일 값이어야 함). 2026-08-08 발견: 이걸 안
    쓰고 이미지 맨 아래(x=0m)부터 클라이밍하면 n_windows 중 상당수가 항상 빈
    사각지대만 훑게 돼 hit_ratio가 구조적으로 낮아지고(실측: 점선/흐린 구간에서
    윈도우 9개 중 1~3개만 적중 → 점 3개짜리 불안정한 다항식 피팅 → yaw 폭주),
    None이면 기존 동작(x=0부터)으로 되돌아감.

    max_c2_diff: "both" 조합 채택 전 좌우 곡률(c2) 일치성 검사 임계값 — 근거는
    "1) 폭 검증 통과" 분기 코드 주석 참조(연석 오검출 사례). 0.15는 실측 1건 기반
    임시값이라 향후 실주행 로그로 재검토 필요.
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

    row_start = None
    if visible_x_min_m is not None:
        _col_ignore, row_start_f = grid.world_to_px(visible_x_min_m, 0.0)
        row_start = int(round(row_start_f))

    def try_side(col: int) -> SideFit | None:
        return _fit_side(bev_mask, col, grid, n_windows, margin_px, minpix, max_window_spread_m,
                          row_start=row_start)

    # 1) 폭 검증 통과하는 좌우 조합 탐색 (중심에 가까운 후보부터, greedy)
    for lc in left_cands:
        for rc in right_cands:
            if abs((rc - lc) - lane_width_px) <= tol_px:
                left_fit = try_side(lc)
                right_fit = try_side(rc)
                if left_fit is not None and right_fit is not None:
                    # 곡률 일치성 검사 (2026-08-08, 연석 오검출 진단) — 폭만 맞으면
                    # "both"로 확정하던 게 문제: 실주행에서 왼쪽(진짜 차선, c2=-0.004)과
                    # 오른쪽(연석, c2=-0.516)이 우연히 폭 조건(3.7±0.6m)을 통과해
                    # 잘못 짝지어짐 — 연석이 코너를 따라 급하게 꺾이면서 중심선 yaw가
                    # 58°까지 폭주했음. 절대 각도로는 진짜 급커브(S자/ㄱ자, 반경
                    # 1.4~2.6m 코스 표준)와 구분이 안 되지만(진짜 급커브도 60°대 나옴),
                    # "두 차선이 서로 평행하게 같이 휘는가"는 급커브에서도 유지되는
                    # 성질이라 여기서 걸러낸다 — 진짜 차선 쌍은 곡률(c2)이 비슷해야 하고,
                    # 연석처럼 한쪽만 이상하게 휘면 큰 차이가 남.
                    if abs(left_fit.coeffs[0] - right_fit.coeffs[0]) > max_c2_diff:
                        continue  # 평행한 차선 쌍이 아님 — 이 조합 버리고 다음 후보 시도
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
