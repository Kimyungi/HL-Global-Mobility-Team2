"""실기용: udp_bridge_node 단독 (dSPACE 실물 연결)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dspace_ip', default_value='192.168.1.10'),
        Node(
            package='bridge_dspace',
            executable='udp_bridge_node',
            parameters=[{
                'dspace_ip': LaunchConfiguration('dspace_ip'),
                'tx_port': 50001,
                'rx_port': 50002,
            }],
            output='screen',
        ),
    ])
