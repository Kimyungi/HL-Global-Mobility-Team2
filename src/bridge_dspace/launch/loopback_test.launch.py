"""부트스트래핑 ① — PC 단독 루프백: dummy ref → can_bridge → dSPACE sim → vehicle vector.

사전 준비 (가상 CAN — PROTOCOL.md):
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set vcan0 up

검증: ros2 topic hz /vehicle/vector  (≈100Hz), ros2 topic echo /vehicle/vector (x 증가)
watchdog 검증: dummy_ref_publisher 프로세스 kill → sim 로그에 TIMEOUT, v→0
저수준 확인: candump vcan0  또는  python3 tools/can_dump.py --iface vcan0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='vcan0'),
        Node(
            package='bridge_dspace',
            executable='dspace_sim_node',
            parameters=[{'can_interface': LaunchConfiguration('can_interface')}],
            output='screen',
        ),
        Node(
            package='bridge_dspace',
            executable='can_bridge_node',
            parameters=[{'can_interface': LaunchConfiguration('can_interface')}],
            output='screen',
        ),
        Node(
            package='bridge_dspace',
            executable='dummy_ref_publisher',
            parameters=[{'v_ref': 0.3}],
            output='screen',
        ),
    ])
