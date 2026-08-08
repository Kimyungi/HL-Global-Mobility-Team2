"""전체 파이프라인: 차선 이진 마스크 -> BEV -> 슬라이딩 윈도우 -> 중심선 ->
경로 다점(多點, x,y,yaw,curvature) + confidence.

⚠️ 2026-08-08 설계 변경: REQUIREMENTS.md/PROTOCOL.md는 원래 "점 1개"로 합의됐었으나
(2026-07-29), 실차 조향 진단 결과 dSPACE MPC가 실제로는 다점(최대 20, CLAUDE.md의
"예측 지평 200ms/N=20"과 일치)을 받아야 궤적 추종을 시작하는 것으로 확인됨 — GPS
성공 로그(gps_run_20260806_scurve) 내에서 n_points=1 구간은 str 변동폭 0.1°(무반응),
n_points=20 구간은 55.6°(정상 반응)로 같은 세션 안에서 직접 대조됨. 이 문서 갱신은
팀 논의 후 진행 예정 — 지금은 실측 검증 우선.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from stack_lane.bev import BevGrid, warp_to_bev
from stack_lane.lane_fit import LANE_WIDTH_M, LaneFitResult, SideFit, fit_lane


@dataclass
class PathPoint:
    x: float
    y: float
    yaw: float
    curvature: float


@dataclass
class LaneEstimate:
    x: float
    y: float
    yaw: float
    curvature: float
    confidence: float
    mode: str  # 'both' | 'left_only' | 'right_only' | 'none'
    points: list[PathPoint] = field(default_factory=list)  # 근거리->원거리 순, 1개 이상


def _side_score(side: SideFit, hit_ratio_target: float, residual_tolerance_m: float) -> float:
    hit_score = min(side.hit_ratio / hit_ratio_target, 1.0)
    resid_score = max(0.0, 1.0 - side.rms_residual_m / residual_tolerance_m)
    return hit_score * resid_score


def compute_confidence(result: LaneFitResult, *, hit_ratio_target: float = 0.5,
                        residual_tolerance_m: float = 0.30,
                        width_tolerance_m: float = 1.0,
                        single_side_penalty: float = 0.85) -> float:
    """윈도우 검출률 x 피팅 잔차 x (both일 때만) 폭 일치도, 편측이면 페널티 곱.

    residual_tolerance_m=0.30, width_tolerance_m=1.0 근거 (2026-08-07, 정상 주행
    자세 실주행 로그 1319개 both 프레임 역산): 원래 0.15/0.6이었을 때 hit_score는
    이미 0.90~0.95로 좋았는데 resid_score(0.69~0.77)·width_score(0.79)가 병목이라
    confidence가 0.7을 넘는 비율이 1.8%뿐이었음. 근데 실측 잔차 자체는 3.5~4.6cm로
    실제로는 정밀한 피팅이었고, 폭 오차(3.7m 목표 대비 +0.13m)도 placeholder
    호모그래피의 알려진 편향 때문(§9)이라 실제 검출 품질 저하가 아니었음. 0.30/1.0로
    완화하니 같은 로그에서 mean=0.70, 0.7 이상 비율 56%로 개선. 이 tolerance는
    fit_lane()의 폭 검증(하드 게이트, 인접 차로 오선택 방지용)과는 별개 파라미터라
    완화해도 그 판별력엔 영향 없음. 실측 캘리브레이션 이후 width 편향이 줄면 재검토.

    single_side_penalty=0.85 근거 (2026-08-07, 삐딱한 주행 로그 역산):
    adas_mgm의 lane_conf_return=0.6(waypoint->lane 복귀 임계, params.yaml)을 넘지
    못하면 차선이 한쪽만 보이는 구간에서 영영 lane 모드로 복귀 못 하는 문제가 있었음.
    원래 0.7이었을 때 실제 편측 검출(hit_ratio~0.48, residual~2.5cm, 품질 자체는
    나쁘지 않았음)의 confidence가 0.59에서 막혀 0.6을 절대 못 넘었음. 0.85로 올리면
    같은 로그 기준 편측 프레임의 99.9%가 0.6 이상 — 노이즈 심한 프레임은 hit_score/
    resid_score 자체가 낮아지므로 penalty와 무관하게 여전히 낮게 나옴(판별력 유지).
    실측 캘리브레이션 이후 residual 분포가 바뀌면 재검토 필요.
    """
    if result.mode == "none":
        return 0.0

    if result.mode == "both":
        s = (_side_score(result.left, hit_ratio_target, residual_tolerance_m)
             + _side_score(result.right, hit_ratio_target, residual_tolerance_m)) / 2.0
        width_score = max(0.0, 1.0 - abs(result.width_m - LANE_WIDTH_M) / width_tolerance_m)
        return float(np.clip(s * width_score, 0.0, 1.0))

    side = result.left if result.mode == "left_only" else result.right
    return float(np.clip(_side_score(side, hit_ratio_target, residual_tolerance_m) * single_side_penalty, 0.0, 1.0))


def _point_from_fit(center_coeffs: np.ndarray, x: float) -> PathPoint:
    """center_coeffs: np.polyfit(deg=2) 결과 [c2, c1, c0], y_m = c2 x^2 + c1 x + c0."""
    c2, c1, c0 = center_coeffs
    y = c2 * x * x + c1 * x + c0
    dy = 2 * c2 * x + c1
    yaw = float(np.arctan(dy))
    curvature = float((2 * c2) / (1.0 + dy ** 2) ** 1.5)
    return PathPoint(x=float(x), y=float(y), yaw=yaw, curvature=curvature)


def lookahead_from_fit(center_coeffs: np.ndarray, lookahead_m: float) -> tuple[float, float, float, float]:
    """단일 지점 버전 — 하위호환용(디버그 시각화 등). 다점 출력은 sample_path_points 참조."""
    p = _point_from_fit(center_coeffs, lookahead_m)
    return p.x, p.y, p.yaw, p.curvature


def sample_path_points(center_coeffs: np.ndarray, x_start: float, x_end: float,
                        n_points: int) -> list[PathPoint]:
    """근거리(x_start)~원거리(x_end) 구간을 n_points개로 등간격 샘플링.

    dSPACE MPC가 다점 지평을 요구하는 것으로 실측 확인됨(모듈 docstring 참조) —
    GPS 실측 범위(x≈1~6m, 최대 20점)를 참고해 기본값을 잡음.
    """
    n_points = max(1, n_points)
    if n_points == 1:
        xs = [x_end]
    else:
        xs = [x_start + (x_end - x_start) * i / (n_points - 1) for i in range(n_points)]
    return [_point_from_fit(center_coeffs, x) for x in xs]


def estimate_lane_path(lane_mask: np.ndarray, H: np.ndarray, grid: BevGrid, *,
                        lookahead_m: float = 3.0,
                        n_points: int = 20, points_x_start: float = 2.5, points_x_end: float | None = None,
                        fit_kwargs: dict | None = None,
                        confidence_kwargs: dict | None = None):
    """points_x_start 기본 2.5m = 카메라 최소 가시거리(PROJECT_BRIEF.md §6) —
    그보다 가까운 구간은 실측 근거 없이 다항식을 외삽하는 것이라 신뢰도가 낮음.
    points_x_end 기본값은 grid.x_max(현재 6.0m, GPS 실측 범위와 유사)."""
    x_end = points_x_end if points_x_end is not None else grid.x_max
    bev_mask = warp_to_bev(lane_mask, H, grid)
    result = fit_lane(bev_mask, grid, **(fit_kwargs or {}))

    if result.mode == "none" or result.center_coeffs is None:
        neutral = PathPoint(x=lookahead_m, y=0.0, yaw=0.0, curvature=0.0)
        estimate = LaneEstimate(x=neutral.x, y=neutral.y, yaw=neutral.yaw, curvature=neutral.curvature,
                                 confidence=0.0, mode="none", points=[neutral])
        return estimate, {"bev_mask": bev_mask, "fit": result}

    x, y, yaw, curvature = lookahead_from_fit(result.center_coeffs, lookahead_m)
    points = sample_path_points(result.center_coeffs, points_x_start, x_end, n_points)
    confidence = compute_confidence(result, **(confidence_kwargs or {}))
    estimate = LaneEstimate(x=x, y=y, yaw=yaw, curvature=curvature, confidence=confidence,
                             mode=result.mode, points=points)
    return estimate, {"bev_mask": bev_mask, "fit": result}
