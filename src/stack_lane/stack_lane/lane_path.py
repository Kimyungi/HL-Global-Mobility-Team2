"""차선 마스크 -> BEV -> 다점 중심선 피팅 -> station +2.5m 목표점 1개.

차량 원점의 중심선 최근접 투영을 현재 station으로 삼는다. 내부 다점 검사는
유지하지만 반환은 같은 곡선의 x/y/yaw/curvature 한 점뿐이다 (2026-09-12).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from stack_lane.bev import BevGrid, warp_to_bev
from stack_lane.lane_fit import LANE_WIDTH_M, LaneFitResult, SideFit, fit_lane
from stack_lane.station_preview import CAMERA_PREVIEW_STATION_M, station_preview_x

# 물리적 타당성 상한 (2026-08-08, 조향 진단).
#
# yaw는 절대각도로 거르지 않기로 함 (2026-08-08, 연석 오검출 사례 이후 재검토):
# 처음엔 ±20°를 넘으면 'none' 처리했는데, 실제 코스에 S자/ㄱ자(반경 1.4~2.6m)가
# 있어서 진짜 급커브도 근거리 점에서 yaw가 60°대까지 나올 수 있음(κ=1/1.4≈0.714,
# x=2.5m → atan(κx)≈61°) — 연석 오검출 사례(58°)와 크기가 겹쳐 절대각도로는
# 구분이 안 됨. "both"(양쪽 다 검출) 모드는 대신 좌우 곡률 일치성 검사로 대체
# (lane_fit.py의 max_c2_diff — 진짜 평행한 차선 쌍은 곡률이 비슷하고, 연석처럼
# 한쪽만 이상하게 휘면 크게 어긋남, 급커브에도 안전). 편측(한쪽만 검출)은 비교할
# 상대가 없어 애초에 이 방법도 못 씀 — 검출 품질(hit_ratio 기반 confidence·편측
# 페널티)로만 신뢰도를 매기고 그대로 따라간다(사용자 판단, 2026-08-08).
#
# y(횡오프셋)는 여전히 절대값으로 거른다 — 다항식이 수치적으로 폭주하면(예:
# 근거리 사각지대 낭비 시절 실측된 y=35m) yaw보다 y가 먼저/더 극단적으로 튀는
# 경향이 있었고, 이건 급커브로도 설명 안 되는 명백한 이상값이라 판단.
#
# 2.5m -> 4.0m로 완화 (2026-08-08, 실주행에서 발견): 편측 폴백은 검출된 한쪽
# 위치에 차선폭 절반(1.85m)을 더해 중심을 추정하는데, 정상적으로 잘 검출된
# (hit_ratio 0.78~0.89) 직선 하나가 근거리 점에서 2.539m로 옛 기준(2.5m)을
# 3.9cm 초과했다는 이유만으로 20개 점 전체가 폐기되고 차량이 정지함. 실제
# 폭주 사례(35m)와는 자릿수가 다른 정상 범위라 4.0m로 여유를 둠.
MAX_ABS_Y_M = 4.0


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
    points: list[PathPoint] = field(default_factory=list)  # 반환 목표점 1개
    # 기존 CSV 필드 이름 유지: 유효 station preview 여부와 실제 목표점 x를 기록한다.
    # ref_point0_x는 station 거리(항상 +2.5m)와 다르다.
    ref_point0_applied: bool = False
    ref_point0_x: float = 0.0
    # 'none'이 된 이유 분류 (2026-08-10, 오실레이션 3계층 진단용) — 카메라/인식
    # 계층에서 "애초에 검출 실패"인지 "검출은 됐지만 우리 필터가 거부"인지 로그로
    # 구분하기 위함. 값: '' | 'no_fit'(fit_lane 자체가 none) | 'implausible'(y 폭주)
    # | 'discontinuous'(직전값 대비 과도한 점프) | 'invalid_preview'(station 계산/기하 무효).
    reject_reason: str = ""


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
    """지정 x에서 평가. 기존 raw 연속성 검사의 기준점이며 station preview와 별개."""
    p = _point_from_fit(center_coeffs, lookahead_m)
    return p.x, p.y, p.yaw, p.curvature


def sample_path_points(center_coeffs: np.ndarray, x_start: float, x_end: float,
                        n_points: int) -> list[PathPoint]:
    """근거리(x_start)~원거리(x_end) 구간을 n_points개로 등간격 샘플링.

    내부 다항식 타당성 검사 표본이다. 이 배열을 제어 출력으로 반환하지 않는다.
    """
    n_points = max(1, n_points)
    if n_points == 1:
        xs = [x_end]
    else:
        xs = [x_start + (x_end - x_start) * i / (n_points - 1) for i in range(n_points)]
    return [_point_from_fit(center_coeffs, x) for x in xs]
def _is_plausible(points: list[PathPoint]) -> bool:
    """다항식 외삽이 물리적으로 타당한 범위 안에 있는지. yaw는 검사 안 함(근거는
    모듈 상단 주석 — 급커브와 오검출을 각도만으론 구분 못 함)."""
    return all(np.isfinite([p.x, p.y, p.yaw, p.curvature]).all()
               and abs(p.y) <= MAX_ABS_Y_M for p in points)


def estimate_lane_path(lane_mask: np.ndarray, H: np.ndarray, grid: BevGrid, *,
                        lookahead_m: float = 3.0,
                        n_points: int = 20, points_x_start: float = 2.5, points_x_end: float | None = None,
                        prev_y: float | None = None, max_y_jump_m: float = 1.0,
                        prev_coeffs: np.ndarray | None = None, coeff_smoothing_alpha: float = 1.0,
                        fit_kwargs: dict | None = None,
                        confidence_kwargs: dict | None = None):
    """points_x_start 기본 2.5m = 카메라 최소 가시거리(PROJECT_BRIEF.md §6) —
    그보다 가까운 구간은 실측 근거 없이 다항식을 외삽하는 것이라 신뢰도가 낮음.
    points_x_end 기본값은 grid.x_max(현재 6.0m, GPS 실측 범위와 유사).

    반환 points와 estimate의 x/y/yaw/curvature는 station +2.5m의 한 점이다.
    n_points는 내부 검사 표본 수, lookahead_m은 raw 연속성 검사의 x 기준이다.
    confidence/곡률/횡오차로 preview 거리를 조절하거나 다점 배열을 반환하지 않는다.

    prev_y/max_y_jump_m: 프레임 간 연속성 체크 (2026-08-08, 편측 오검출 진단) —
    fit_lane()은 매 프레임 후보를 처음부터 새로 찾기 때문에(직전 프레임 기억 없음),
    검출이 애매한 순간 "왼쪽"이라 부르는 대상이 프레임마다 다른 실제 선으로
    튈 수 있음. 실측: 3초 사이 모드가 6번 넘게 바뀌다가 엉뚱한 선에 고정돼
    y가 갑자기 -2.58m로 튀고 그 상태로 여러 초 유지된 사례 확인. 호출부(node.py)가
    직전에 채택했던 y를 prev_y로 넘겨주면, 그로부터 max_y_jump_m(기본 1.0m)보다
    많이 튄 결과는 'none' 처리한다 — 속도(~0.5m/s)·프레임 간격(~45ms) 기준
    실제 물리적 이동은 한 프레임에 수 cm뿐이라 1m 이상 점프는 대부분 오검출.

    prev_coeffs/coeff_smoothing_alpha: 다항식 계수 지수이동평균(EMA) 저역통과
    필터 (2026-08-08, 오실레이션 완화 — GPS stack_gps의 fusion_alpha와 동일 기법).
    REF_POINT_00을 가깝게 당길수록(조향 게인↑) 정상 검출 중에도 남아있는
    프레임 간 미세 흔들림(~0.05~0.1m)까지 증폭돼 저주파 오실레이션으로 나타남 —
    연속성 체크(위)는 "엉뚱한 값"만 거르지 "정상 범위 내 미세 흔들림"은 못 잡음.
    smoothed = alpha*이번프레임 + (1-alpha)*직전_smoothed로 매끄럽게 만든다.
    **주의**: 이상치 검사(연속성/타당성)는 반드시 원본(raw) 계수로 먼저 하고,
    통과한 것만 스무딩한다 — 스무딩을 먼저 하면 큰 튐이 여러 프레임에 걸쳐
    서서히 섞여 들어가 연속성 체크가 아예 못 잡게 되는 부작용이 있기 때문.
    alpha=1.0(기본)은 스무딩 없음 = 기존 동작과 동일. 작을수록 부드럽지만
    실제 경로 변화에 대한 반응도 함께 느려짐(트레이드오프, 실측 튜닝 필요).
    """
    x_end = points_x_end if points_x_end is not None else grid.x_max
    bev_mask = warp_to_bev(lane_mask, H, grid)
    # visible_x_min_m 기본값을 points_x_start와 맞춰 단일 진실원천으로 유지 —
    # fit_kwargs가 명시하면 그쪽이 우선(오버라이드 가능).
    result = fit_lane(bev_mask, grid, **{"visible_x_min_m": points_x_start, **(fit_kwargs or {})})

    if result.mode == "none" or result.center_coeffs is None:
        neutral = PathPoint(x=lookahead_m, y=0.0, yaw=0.0, curvature=0.0)
        estimate = LaneEstimate(x=neutral.x, y=neutral.y, yaw=neutral.yaw, curvature=neutral.curvature,
                                 confidence=0.0, mode="none", points=[neutral],
                                 ref_point0_applied=False, ref_point0_x=neutral.x,
                                 reject_reason="no_fit")
        return estimate, {"bev_mask": bev_mask, "fit": result}

    confidence = compute_confidence(result, **(confidence_kwargs or {}))

    # 1) 원본(raw) 계수로 먼저 이상치 검사 — 스무딩을 거치기 전에 걸러야
    # 큰 튐이 여러 프레임에 걸쳐 섞여 들어가는 걸 막을 수 있다(docstring 참조).
    raw_x, raw_y, raw_yaw, raw_curvature = lookahead_from_fit(result.center_coeffs, lookahead_m)
    raw_points = sample_path_points(result.center_coeffs, points_x_start, x_end, n_points)
    implausible = not _is_plausible([PathPoint(x=raw_x, y=raw_y, yaw=raw_yaw, curvature=raw_curvature)] + raw_points)
    discontinuous = prev_y is not None and abs(raw_y - prev_y) > max_y_jump_m

    if implausible or discontinuous:
        # 피팅은 됐지만(mode는 both/left_only/right_only) 신뢰 못 함 — 외삽 폭주
        # (implausible) 또는 직전 추적 위치에서 갑자기 너무 멀리 튐(discontinuous,
        # 엉뚱한 선을 잡았을 가능성). 'none'과 동일하게: confidence=0으로 MGM
        # 히스테리시스가 lane 이탈 판단을 하게 둔다(판단은 여전히 MGM 스테이트머신 몫).
        neutral = PathPoint(x=lookahead_m, y=0.0, yaw=0.0, curvature=0.0)
        reason = "implausible" if implausible else "discontinuous"
        estimate = LaneEstimate(x=neutral.x, y=neutral.y, yaw=neutral.yaw, curvature=neutral.curvature,
                                 confidence=0.0, mode="none", points=[neutral],
                                 ref_point0_applied=False, ref_point0_x=neutral.x,
                                 reject_reason=reason)
        return estimate, {"bev_mask": bev_mask, "fit": result, "raw_y": raw_y}

    # 2) 통과한 것만 스무딩 — 최종 출력(x/y/yaw/curvature/points/REF_POINT_00)은
    # 전부 스무딩된 계수 하나로부터 일관되게 계산한다.
    coeffs = result.center_coeffs
    if prev_coeffs is not None and coeff_smoothing_alpha < 1.0:
        smoothed_coeffs = coeff_smoothing_alpha * coeffs + (1.0 - coeff_smoothing_alpha) * prev_coeffs
    else:
        smoothed_coeffs = coeffs

    debug = {"bev_mask": bev_mask, "fit": result, "raw_y": raw_y,
             "smoothed_coeffs": smoothed_coeffs}
    try:
        projection_x, target_x = station_preview_x(smoothed_coeffs)
        point = _point_from_fit(smoothed_coeffs, target_x)
        if not _is_plausible([point]):
            raise ValueError("invalid station preview geometry")
    except (ValueError, OverflowError, np.linalg.LinAlgError):
        neutral = PathPoint(x=lookahead_m, y=0.0, yaw=0.0, curvature=0.0)
        estimate = LaneEstimate(x=neutral.x, y=neutral.y, yaw=neutral.yaw, curvature=neutral.curvature,
                                confidence=0.0, mode="none", points=[neutral],
                                ref_point0_x=neutral.x, reject_reason="invalid_preview")
        return estimate, debug

    debug.update(preview_projection_x=projection_x, preview_station_m=CAMERA_PREVIEW_STATION_M)
    estimate = LaneEstimate(x=point.x, y=point.y, yaw=point.yaw, curvature=point.curvature,
                            confidence=confidence, mode=result.mode, points=[point],
                            ref_point0_applied=True, ref_point0_x=point.x)
    return estimate, debug
