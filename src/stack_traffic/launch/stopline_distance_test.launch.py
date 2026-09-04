"""노트북 단독 OAK-D 신호등 + 정지선 거리 진단."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_TRAFFIC_OAK_MXID = "14442C10B167CFD200"


def build_node_parameters():
    """launch와 테스트가 공유하는 stack_traffic 파라미터를 만든다."""
    oak_mxid = LaunchConfiguration("oak_mxid")
    oak_usb_speed = LaunchConfiguration("oak_usb_speed")
    oak_width = LaunchConfiguration("oak_width")
    oak_height = LaunchConfiguration("oak_height")
    oak_fps = LaunchConfiguration("oak_fps")
    oak_depth_enabled = LaunchConfiguration("oak_depth_enabled")
    stopline_model_path = LaunchConfiguration("stopline_model_path")
    stop_distance = LaunchConfiguration("stopline_stop_distance_m")
    stop_y_ratio = LaunchConfiguration("stopline_stop_y_ratio")
    resume_on_green = LaunchConfiguration("resume_on_green")
    show_debug = LaunchConfiguration("show_debug")
    return {
        "camera_backend": "oak",
        "oak_width": ParameterValue(oak_width, value_type=int),
        "oak_height": ParameterValue(oak_height, value_type=int),
        "oak_fps": ParameterValue(oak_fps, value_type=float),
        "oak_mxid": ParameterValue(oak_mxid, value_type=str),
        "oak_usb_speed": ParameterValue(
            oak_usb_speed,
            value_type=str,
        ),
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
        # 현장 검증값을 유지한다. 실제 유효 시간은 처리 FPS에 비례하므로
        # USB2/10fps 통합 후 별도 A/B 없이 프레임 수를 함께 바꾸지 않는다.
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
        "stopline_model_path": ParameterValue(
            stopline_model_path,
            value_type=str,
        ),
        "stopline_yolo_confidence_threshold": 0.35,
        "stopline_yolo_image_size": 640,
        "stopline_roi_x_min": 0.08,
        "stopline_roi_y_min": 0.48,
        "stopline_roi_x_max": 0.92,
        "stopline_roi_y_max": 0.98,
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
        "show_debug": ParameterValue(show_debug, value_type=bool),
        "show_auxiliary_debug": False,
    }


def generate_launch_description():
    parameters = build_node_parameters()
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "oak_mxid",
                default_value=DEFAULT_TRAFFIC_OAK_MXID,
                description=(
                    "교통용 OAK-D MxID. 기본값은 현재 차량의 교통 카메라; "
                    "다른 장치로 시험할 때만 명시적으로 재정의"
                ),
            ),
            DeclareLaunchArgument(
                "oak_usb_speed",
                default_value="high",
                description=(
                    "high=USB2(480M) 강제, super=USB3 상한. "
                    "차량에서는 GNSS 간섭 완화를 위해 high 사용"
                ),
            ),
            DeclareLaunchArgument(
                "oak_width",
                default_value="1280",
                description="OAK RGB 출력 폭(px)",
            ),
            DeclareLaunchArgument(
                "oak_height",
                default_value="720",
                description="OAK RGB 출력 높이(px)",
            ),
            DeclareLaunchArgument(
                "oak_fps",
                default_value="10.0",
                description=(
                    "USB2 차량 프로필은 10fps. 30fps는 USB2 대역폭 초과"
                ),
            ),
            DeclareLaunchArgument(
                "oak_depth_enabled",
                default_value="false",
                description=(
                    "false=검증된 RGB 정지선 y 기준 차량 실행. "
                    "true=저해상도 depth 진단용"
                ),
            ),
            DeclareLaunchArgument(
                "stopline_model_path",
                default_value="",
                description=(
                    "정지선 YOLO .pt 절대경로. 비우면 패키지에 포함된 "
                    "stopline_yolov8s_seg.pt 사용"
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
                    "(1 초과는 화면 아래 외삽). 현장값 0.98은 현재 ROI·"
                    "고정 장착·0.28m/s 이하에서만 검증됐으며 카메라 장착, "
                    "ROI 또는 속도 변경 시 재보정"
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
            DeclareLaunchArgument(
                "show_debug",
                default_value="false",
                description=(
                    "true=OpenCV 진단 창 표시, false=headless 실차 실행"
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
