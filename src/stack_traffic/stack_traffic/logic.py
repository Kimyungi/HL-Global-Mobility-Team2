"""ROS나 카메라에 의존하지 않는 신호등 정지 판단 로직."""

import math
import statistics


BBox = tuple[int, int, int, int]
BBoxCandidate = tuple[BBox, float]


def camera_poll_timed_out(
    read_status: str,
    silence_sec: float,
    timeout_sec: float,
) -> bool:
    """정상적인 빈 non-blocking poll과 실제 카메라 장애를 구분한다."""
    if timeout_sec <= 0.0:
        raise ValueError("timeout_sec는 0보다 커야 합니다.")
    return read_status != "empty" or silence_sec >= timeout_sec


def normalized_roi_to_bbox(
    frame_shape,
    x_min: float,
    y_min: float,
    x_max: float,
    y_max: float,
) -> BBox:
    """정규화 좌표를 영상 경계 안의 정수 ROI bbox로 바꾼다."""
    frame_height, frame_width = frame_shape[:2]
    x1 = max(0, min(int(round(frame_width * x_min)), frame_width - 1))
    y1 = max(0, min(int(round(frame_height * y_min)), frame_height - 1))
    x2 = max(x1 + 1, min(int(round(frame_width * x_max)), frame_width))
    y2 = max(y1 + 1, min(int(round(frame_height * y_max)), frame_height))
    return x1, y1, x2, y2


def select_horizontal_roi_tile(
    search_roi: BBox,
    tile_width_px: int,
    tracked_bbox: BBox | None,
    scan_index: int,
) -> BBox:
    """넓은 검색 ROI에서 물체 크기를 보존할 가로 타일 하나를 고른다.

    추적 대상이 있으면 대상 중심을 따라가고, 없으면 중앙·좌·우 순서로
    검색한다. 한 주기에 YOLO를 한 번만 실행하므로 타일을 써도 연산량은
    거의 늘지 않는다.
    """
    roi_x1, roi_y1, roi_x2, roi_y2 = search_roi
    roi_width = roi_x2 - roi_x1
    if roi_width < 1 or roi_y2 <= roi_y1:
        raise ValueError("search_roi는 양의 면적이어야 합니다.")
    if tile_width_px < 1:
        raise ValueError("tile_width_px는 1 이상이어야 합니다.")
    if tile_width_px >= roi_width:
        return search_roi

    half_width = 0.5 * tile_width_px
    if tracked_bbox is not None:
        target_center_x = 0.5 * (tracked_bbox[0] + tracked_bbox[2])
    else:
        roi_center_x = 0.5 * (roi_x1 + roi_x2)
        scan_centers = (
            roi_center_x,
            roi_x1 + half_width,
            roi_x2 - half_width,
        )
        target_center_x = scan_centers[scan_index % len(scan_centers)]

    tile_x1 = int(round(target_center_x - half_width))
    tile_x1 = max(roi_x1, min(tile_x1, roi_x2 - tile_width_px))
    return tile_x1, roi_y1, tile_x1 + tile_width_px, roi_y2


def frame_bbox_to_roi(
    bbox: BBox,
    roi_bbox: BBox,
) -> BBox | None:
    """전체 영상 bbox를 ROI 내부 좌표로 자르고 변환한다."""
    x1, y1, x2, y2 = bbox
    roi_x1, roi_y1, roi_x2, roi_y2 = roi_bbox
    clipped_x1 = max(x1, roi_x1)
    clipped_y1 = max(y1, roi_y1)
    clipped_x2 = min(x2, roi_x2)
    clipped_y2 = min(y2, roi_y2)
    if clipped_x2 <= clipped_x1 or clipped_y2 <= clipped_y1:
        return None
    return (
        clipped_x1 - roi_x1,
        clipped_y1 - roi_y1,
        clipped_x2 - roi_x1,
        clipped_y2 - roi_y1,
    )


def roi_bbox_to_frame(bbox: BBox, roi_bbox: BBox) -> BBox:
    """ROI 내부 bbox를 전체 영상 좌표로 되돌린다."""
    x1, y1, x2, y2 = bbox
    roi_x1, roi_y1, _, _ = roi_bbox
    return (
        x1 + roi_x1,
        y1 + roi_y1,
        x2 + roi_x1,
        y2 + roi_y1,
    )


def bbox_iou(first: BBox, second: BBox) -> float:
    """두 bbox의 IoU를 반환한다."""
    first_x1, first_y1, first_x2, first_y2 = first
    second_x1, second_y1, second_x2, second_y2 = second

    intersection_width = max(
        0,
        min(first_x2, second_x2) - max(first_x1, second_x1),
    )
    intersection_height = max(
        0,
        min(first_y2, second_y2) - max(first_y1, second_y1),
    )
    intersection_area = intersection_width * intersection_height

    first_area = max(0, first_x2 - first_x1) * max(
        0,
        first_y2 - first_y1,
    )
    second_area = max(0, second_x2 - second_x1) * max(
        0,
        second_y2 - second_y1,
    )
    union_area = first_area + second_area - intersection_area
    if union_area <= 0:
        return 0.0
    return intersection_area / union_area


def select_tracking_candidate(
    candidates: list[BBoxCandidate],
    previous_bbox: BBox,
    minimum_iou: float,
    maximum_center_shift_ratio: float,
    minimum_size_similarity: float,
) -> BBoxCandidate | None:
    """이전 bbox와 공간적으로 이어지는 후보 중 가장 안정적인 것을 고른다."""
    previous_x1, previous_y1, previous_x2, previous_y2 = previous_bbox
    previous_width = max(1, previous_x2 - previous_x1)
    previous_height = max(1, previous_y2 - previous_y1)
    previous_diagonal = max(
        1.0,
        math.hypot(previous_width, previous_height),
    )
    previous_center_x = 0.5 * (previous_x1 + previous_x2)
    previous_center_y = 0.5 * (previous_y1 + previous_y2)

    best_candidate = None
    best_score = -1.0

    for bbox, confidence in candidates:
        x1, y1, x2, y2 = bbox
        width = max(1, x2 - x1)
        height = max(1, y2 - y1)
        center_x = 0.5 * (x1 + x2)
        center_y = 0.5 * (y1 + y2)

        overlap = bbox_iou(previous_bbox, bbox)
        width_similarity = min(previous_width, width) / max(
            previous_width,
            width,
        )
        height_similarity = min(previous_height, height) / max(
            previous_height,
            height,
        )
        size_similarity = min(width_similarity, height_similarity)
        if size_similarity < minimum_size_similarity:
            continue

        center_shift_ratio = math.hypot(
            center_x - previous_center_x,
            center_y - previous_center_y,
        ) / previous_diagonal
        if (
            overlap < minimum_iou
            and center_shift_ratio > maximum_center_shift_ratio
        ):
            continue

        center_similarity = max(
            0.0,
            1.0 - center_shift_ratio / maximum_center_shift_ratio,
        )
        score = (
            0.55 * overlap
            + 0.25 * center_similarity
            + 0.15 * size_similarity
            + 0.05 * confidence
        )
        if score > best_score:
            best_score = score
            best_candidate = (bbox, confidence)

    return best_candidate


def is_red_clear_confirmed(
    red_history,
    bbox_observed_history,
    window_size: int,
    minimum_bbox_observations: int,
) -> bool:
    """같은 최근 창에서 적색 0회와 실제 bbox 관측 횟수를 확인한다."""
    recent_red = list(red_history)[-window_size:]
    recent_bbox = list(bbox_observed_history)[-window_size:]
    return (
        len(recent_red) == window_size
        and len(recent_bbox) == window_size
        and sum(recent_red) == 0
        and sum(recent_bbox) >= minimum_bbox_observations
    )


def should_record_color_vote(
    detection_fresh: bool,
    red_raw: bool,
    green_raw: bool,
    red_fresh_seeded: bool = False,
) -> bool:
    """template 적색은 fresh YOLO 적색으로 seed된 target에서만 쓴다."""
    if detection_fresh:
        return bool(red_raw or green_raw)
    return bool(red_raw and red_fresh_seeded)


def should_clear_visual_track(
    yolo_missed_frames: int,
    template_failed_frames: int,
    maximum_yolo_misses: int,
    maximum_template_failures: int,
) -> bool:
    """YOLO와 template이 모두 연속 실패했을 때만 target을 해제한다."""
    return bool(
        yolo_missed_frames >= maximum_yolo_misses
        and template_failed_frames >= maximum_template_failures
    )


def update_stop_latch(
    current: bool,
    red_active: bool,
    pixel_approaching: bool,
    green_active: bool,
    resume_on_green: bool,
    red_clear_active: bool = False,
    resume_on_red_clear: bool = False,
) -> bool:
    """적색+접근 조건으로 정지하고 설정된 안전 조건에서만 해제한다."""
    if red_active and pixel_approaching:
        return True
    if current and resume_on_green and green_active and not red_active:
        return False
    if current and resume_on_red_clear and red_clear_active and not red_active:
        return False
    return current


def update_red_phase_latch(
    current: bool,
    red_active: bool,
    green_active: bool,
) -> bool:
    """확정 적색을 fresh 초록 확정 전까지 기억한다.

    신호등과 정지선은 같은 영상에서도 서로 다른 프레임에 안정 검출될 수 있다.
    적색 3/5를 한 번 확정했다면 짧은 bbox 소실로 그 사실을 버리지 않고, fresh
    초록 3/5만 새로운 신호 페이즈의 근거로 인정한다. 두 색이 동시에 활성화된
    비정상 투표창에서는 안전측인 적색이 이긴다.
    """
    if red_active:
        return True
    if current and green_active:
        return False
    return current


def should_accept_anchored_green(
    red_phase_latched: bool,
    anchor_available: bool,
    green_raw: bool,
) -> bool:
    """확정 적색 bbox의 최신 영상에서 본 초록을 해제 투표로 허용한다.

    YOLO가 초록 화살표/비원형 점등 뒤 housing을 놓쳐도, 적색으로 확정했던 같은
    화면 위치만 재검사한다. 적색 페이즈나 anchor가 없으면 화면의 임의 초록색을
    해제 근거로 사용할 수 없다.
    """
    return bool(red_phase_latched and anchor_available and green_raw)


def classify_color_ratios(
    red_ratio: float,
    green_ratio: float,
    minimum_red_ratio: float,
    minimum_green_ratio: float,
) -> tuple[bool, bool]:
    """색 비율로 상호 배타적인 red/green 단일 프레임 판정을 만든다."""
    red_raw = (
        red_ratio >= minimum_red_ratio
        and red_ratio >= green_ratio
    )
    green_raw = (
        green_ratio >= minimum_green_ratio
        and green_ratio > red_ratio
    )
    return red_raw, green_raw


def is_stopline_approaching(
    median_distance_m: float,
    stop_distance_m: float,
    current_line_detected: bool,
    current_depth_accepted: bool,
    line_stable: bool,
    valid_distance_samples: int,
    minimum_distance_samples: int,
) -> bool:
    """현재 검출과 거리 표본이 모두 유효할 때만 정지선 접근을 확정한다."""
    return (
        stop_distance_m > 0.0
        and current_line_detected
        and current_depth_accepted
        and line_stable
        and valid_distance_samples >= minimum_distance_samples
        and math.isfinite(median_distance_m)
        and 0.0 <= median_distance_m <= stop_distance_m
    )


def is_stopline_y_approaching(
    median_y_px: float,
    frame_height_px: int,
    stop_y_ratio: float,
    current_line_detected: bool,
    line_stable: bool,
    valid_y_samples: int,
    minimum_y_samples: int,
) -> bool:
    """고정 카메라에서 정지선의 영상 y 위치가 임계값에 도달했는지 본다.

    영상 아래쪽으로 갈수록 y가 커진다. 픽셀 절대값 대신 높이 대비 비율을
    사용해 같은 종횡비의 해상도 변경에도 정지 위치가 유지되게 한다.
    """
    return (
        0.0 < stop_y_ratio <= 1.10
        and frame_height_px > 0
        and current_line_detected
        and line_stable
        and valid_y_samples >= minimum_y_samples
        and math.isfinite(median_y_px)
        and median_y_px / float(frame_height_px) >= stop_y_ratio
    )


def combine_stopline_proximity(
    depth_near: bool,
    y_near: bool,
    depth_threshold_m: float,
    y_threshold_ratio: float,
) -> bool:
    """활성화된 정지선 근접 조건을 결합한다.

    하나만 설정하면 그 조건만 사용하고, 둘 다 설정하면 둘을 모두 만족해야
    한다. 두 임계값이 모두 0이면 측정 전용 모드이므로 정지를 만들지 않는다.
    """
    depth_enabled = depth_threshold_m > 0.0
    y_enabled = y_threshold_ratio > 0.0
    if depth_enabled and y_enabled:
        return bool(depth_near and y_near)
    if depth_enabled:
        return bool(depth_near)
    if y_enabled:
        return bool(y_near)
    return False


def robust_nonnegative_median(
    value_history,
    minimum_samples: int,
) -> tuple[float, int]:
    """NaN과 음수를 제외한 거리 이력의 중앙값과 표본 수를 반환한다."""
    valid = [
        float(value)
        for value in value_history
        if math.isfinite(float(value)) and float(value) >= 0.0
    ]
    if len(valid) < minimum_samples:
        return math.nan, len(valid)
    return float(statistics.median(valid)), len(valid)
