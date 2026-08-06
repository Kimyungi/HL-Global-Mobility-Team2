import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


CONFIRM_TOKEN = 'I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'


def validate_real_vehicle_confirmation(context):
    supplied_token = LaunchConfiguration(
        'REAL_VEHICLE_CONFIRM'
    ).perform(context)

    if supplied_token != CONFIRM_TOKEN:
        raise RuntimeError(
            'REAL VEHICLE launch refused. '
            'Set REAL_VEHICLE_CONFIRM:=' + CONFIRM_TOKEN
        )

    return []


def generate_launch_description():
    ydlidar_params = os.path.join(
        '/home/chanmi/ydlidar_ws', 'src', 'ydlidar_ros2_driver',
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
            parameters=[ydlidar_params, {
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
