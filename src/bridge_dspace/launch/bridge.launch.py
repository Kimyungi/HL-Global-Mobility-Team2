"""실기용: can_bridge_node 단독 (dSPACE 실물 CAN 연결).

사전 준비 (PROTOCOL.md):
    sudo ip link set can0 up type can bitrate 1000000
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        Node(
            package='bridge_dspace',
            executable='can_bridge_node',
            parameters=[{
                'can_interface': LaunchConfiguration('can_interface'),
            }],
            output='screen',
        ),
    ])
