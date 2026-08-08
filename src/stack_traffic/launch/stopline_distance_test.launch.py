"""노트북 단독 OAK-D 신호등 + 정지선 거리 진단."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    oak_depth_enabled = LaunchConfiguration("oak_depth_enabled")
    stop_distance = LaunchConfiguration("stopline_stop_distance_m")
    stop_y_ratio = LaunchConfiguration("stopline_stop_y_ratio")
    resume_on_green = LaunchConfiguration("resume_on_green")
    parameters = {
        "camera_backend": "oak",
        "oak_width": 1280,
        "oak_height": 720,
        "oak_fps": 30.0,
        "oak_depth_enabled": ParameterValue(
            oak_depth_enabled,
            value_type=bool,
        ),
        "oak_depth_confidence_threshold": 245,
        "oak_depth_left_right_check": True,
        "oak_depth_subpixel": False,
        "oak_depth_median_filter_size": 3,
        "oak_depth_decimation_factor": 2,
        "oak_depth_speckle_filter": False,
        "oak_depth_spatial_filter": False,
        "oak_depth_temporal_filter": False,
        "process_period_sec": 0.02,
        "camera_timeout_sec": 0.50,
        "yolo_image_size": 576,
        "yolo_inference_interval": 2,
        "detection_roi_enabled": True,
        # 기존 상단 위치는 유지하고 정지선 ROI와 같은 가로폭을 쓴다.
        "detection_roi_x_min": 0.08,
        "detection_roi_y_min": 0.05,
        "detection_roi_x_max": 0.92,
        "detection_roi_y_max": 0.50,
        # 넓은 영역을 40% 폭 타일로 훑어 먼 신호의 입력 픽셀을 보존한다.
        "detection_tile_width_ratio": 0.40,
        "confidence_threshold": 0.12,
        "tracking_confidence_threshold": 0.07,
        "tracking_max_missed_frames": 5,
        "tracking_maximum_center_shift_ratio": 0.80,
        "tracking_minimum_size_similarity": 0.40,
        "bbox_smoothing_current_weight": 0.65,
        "template_tracking_enabled": True,
        # fresh YOLO가 흔들려도 고득점 고정-template은 약 4~5초 유지한다.
        "template_tracking_max_age_frames": 120,
        "template_tracking_max_consecutive_failures": 3,
        "template_tracking_context_scale": 1.8,
        "template_tracking_search_scale": 2.5,
        "template_tracking_minimum_score": 0.60,
        "template_tracking_maximum_center_shift_ratio": 0.75,
        "minimum_box_area": 24,
        "minimum_box_width_height_ratio": 0.80,
        "red_hue_upper": 25,
        "red_hue_high_lower": 165,
        "minimum_color_saturation": 45,
        "minimum_color_value": 60,
        "vote_window": 5,
        "minimum_red_votes": 3,
        "minimum_green_votes": 3,
        "minimum_depth_m": 0.30,
        "maximum_depth_m": 10.0,
        "minimum_depth_valid_ratio": 0.10,
        "minimum_depth_valid_pixels": 80,
        "stopline_detection_enabled": True,
        "stopline_roi_x_min": 0.08,
        "stopline_roi_y_min": 0.48,
        "stopline_roi_x_max": 0.92,
        "stopline_roi_y_max": 0.98,
        "stopline_minimum_value": 145,
        "stopline_maximum_saturation": 90,
        "stopline_minimum_width_ratio": 0.45,
        "stopline_minimum_aspect_ratio": 6.0,
        "stopline_maximum_angle_deg": 12.0,
        "stopline_detection_window": 5,
        "stopline_minimum_detections": 3,
        "stopline_maximum_y_residual_ratio": 0.012,
        "stopline_maximum_y_step_ratio": 0.08,
        "stopline_maximum_backward_step_ratio": 0.005,
        "stopline_depth_inner_width_ratio": 0.50,
        "stopline_depth_band_height_px": 16,
        "stopline_depth_window": 5,
        "stopline_minimum_depth_samples": 3,
        "stopline_minimum_depth_rows": 6,
        "stopline_maximum_row_depth_mad_m": 0.20,
        "stopline_depth_coherence_absolute_tolerance_m": 0.20,
        "stopline_depth_coherence_relative_tolerance": 0.08,
        "stopline_minimum_coherent_pixel_ratio": 0.60,
        "stopline_minimum_inverse_depth_slope_per_px": 0.0001,
        "stopline_maximum_inverse_depth_slope_per_px": 0.02,
        "stopline_maximum_fit_residual_m": 0.25,
        # 0.0은 거리 표시만 수행하고 stop_required를 만들지 않는다.
        "stopline_stop_distance_m": ParameterValue(
            stop_distance,
            value_type=float,
        ),
        "stopline_stop_y_ratio": ParameterValue(
            stop_y_ratio,
            value_type=float,
        ),
        # 출발은 fresh YOLO bbox의 초록 3/5로만 허용한다.
        "resume_on_green": ParameterValue(
            resume_on_green,
            value_type=bool,
        ),
        "resume_on_red_clear": False,
        # interval=2와 로그 주기가 겹쳐 YOLO 프레임만 빠지지 않게 홀수 사용.
        "print_every": 9,
        "show_debug": True,
        "show_auxiliary_debug": False,
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "oak_depth_enabled",
                default_value="true",
                description=(
                    "true=RGB 정렬 depth 측정, false=y 기준 경량 실행"
                ),
            ),
            DeclareLaunchArgument(
                "stopline_stop_distance_m",
                default_value="0.0",
                description="0이면 거리만 표시, 양수이면 적색 정지 임계값(m)",
            ),
            DeclareLaunchArgument(
                "stopline_stop_y_ratio",
                default_value="0.0",
                description=(
                    "0이면 y 조건 비활성, 0~1.10이면 화면 높이 대비 "
                    "정지선 최하단 끝점의 시간 중앙값 임계값"
                    "(1 초과는 화면 아래 외삽)"
                ),
            ),
            DeclareLaunchArgument(
                "resume_on_green",
                default_value="true",
                description=(
                    "true=fresh YOLO 초록 3/5에서 재출발, "
                    "false=정지 래치 자동 해제 없음"
                ),
            ),
            Node(
                package="stack_traffic",
                executable="stack_traffic_node",
                name="stack_traffic_node",
                output="screen",
                parameters=[parameters],
            )
        ]
    )
