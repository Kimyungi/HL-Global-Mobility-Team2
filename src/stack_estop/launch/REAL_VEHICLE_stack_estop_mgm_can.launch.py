import os

from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


CONFIRM_TOKEN = 'I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'


def ydlidar_file(*parts):
    """`ydlidar_ros2_driver` 안의 파일 경로 — **저장소 설치본이 1순위**.

    2026-08-29: 기본값이 `~/ydlidar_ws/src/...` 로 박혀 있었는데 이 PC 의 실제
    워크스페이스는 `~/ydlidar_ros2_ws` 라 파일이 없었다. 없으면 드라이버가 뜨자마자
    죽고 `/scan` 이 0Hz 가 되는데 증상은 "go 가 안 통과한다"로만 보인다.
    저장소가 드라이버를 직접 갖고 있으므로 설치본을 기본값으로 쓰고, 외부
    워크스페이스는 옛 세팅 호환용 폴백으로만 남긴다.
    """
    cands = []
    try:
        cands.append(os.path.join(
            get_package_share_directory('ydlidar_ros2_driver'), *parts))
    except PackageNotFoundError:
        pass
    home = os.path.expanduser('~')
    cands += [os.path.join(home, ws, 'src', 'ydlidar_ros2_driver', *parts)
              for ws in ('ydlidar_ros2_ws', 'ydlidar_ws')]
    return next((p for p in cands if os.path.isfile(p)), cands[0])


def validate_real_vehicle_confirmation(context):
    supplied_token = LaunchConfiguration(
        'REAL_VEHICLE_CONFIRM'
    ).perform(context)

    if supplied_token != CONFIRM_TOKEN:
        raise RuntimeError(
            'REAL VEHICLE launch refused. '
            'Set REAL_VEHICLE_CONFIRM:=' + CONFIRM_TOKEN
        )

    # 라이다 파라미터 파일은 여기서 반드시 확인한다. 없으면 드라이버가 뜨자마자
    # 죽고 respawn=True 로 2초마다 되살아나기만 해 /scan 이 영영 0Hz 인데, 화면에는
    # "go 가 안 통과한다"로만 보여 원인이 라이다로 안 보인다 (2026-08-29: 기본값이
    # 없는 워크스페이스를 가리키고 있었다 — ydlidar_file() 주석 참조).
    ydlidar_params = LaunchConfiguration('ydlidar_params').perform(context)
    if not os.path.isfile(ydlidar_params):
        raise RuntimeError(
            f'라이다 파라미터 파일 없음: {ydlidar_params}\n'
            '  ydlidar_ros2_driver 를 빌드했는지 확인하거나 '
            'ydlidar_params:=<경로> 로 직접 지정하세요.')

    return []


def generate_launch_description():
    default_ydlidar_params = ydlidar_file(
        'params', 'Tmini-Plus-SH.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'REAL_VEHICLE_CONFIRM',
            default_value='NOT_CONFIRMED',
        ),

        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
        ),

        DeclareLaunchArgument(
            'ydlidar_params',
            default_value=default_ydlidar_params,
            description='Path to the YDLIDAR parameter YAML file.',
        ),

        DeclareLaunchArgument(
            'laser_yaw_in_base_rad',
            default_value='1.57079632679',
        ),

        DeclareLaunchArgument(
            'dynamic_enabled',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'dynamic_stop_distance_m',
            default_value='1.20',
        ),

        DeclareLaunchArgument(
            'dynamic_tracking_max_distance_m',
            default_value='3.00',
        ),

        OpaqueFunction(
            function=validate_real_vehicle_confirmation
        ),

        LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace='/',
            parameters=[LaunchConfiguration('ydlidar_params'), {
                'port': '/dev/ttyUSB0',
                'baudrate': 230400,
                'lidar_type': 1,
                'intensity_bit': 8,
            }],
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_static_tf',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.02',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '1.57079632679',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser_frame',
            ],
            output='screen',
        ),

        Node(
            package='stack_estop',
            executable='stack_estop_node',
            name='stack_estop_node',
            parameters=[{
                'laser_yaw_in_base_rad': ParameterValue(
                    LaunchConfiguration(
                        'laser_yaw_in_base_rad'
                    ),
                    value_type=float,
                ),
                'dynamic_enabled': ParameterValue(
                    LaunchConfiguration(
                        'dynamic_enabled'
                    ),
                    value_type=bool,
                ),
                'dynamic_stop_distance_m': ParameterValue(
                    LaunchConfiguration(
                        'dynamic_stop_distance_m'
                    ),
                    value_type=float,
                ),
                'dynamic_tracking_max_distance_m': ParameterValue(
                    LaunchConfiguration(
                        'dynamic_tracking_max_distance_m'
                    ),
                    value_type=float,
                ),
            }],
            output='screen',
        ),

        Node(
            package='adas_mgm',
            executable='mgm_node',
            name='mgm_node',
            output='screen',
        ),

        Node(
            package='bridge_dspace',
            executable='can_bridge_node',
            name='can_bridge_node_REAL_VEHICLE',
            parameters=[{
                'can_interface': LaunchConfiguration(
                    'can_interface'
                )
            }],
            output='screen',
        ),
    ])
