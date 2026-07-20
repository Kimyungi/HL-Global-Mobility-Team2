"""부트스트래핑 ① — PC 단독 루프백: dummy ref → bridge → dSPACE sim → vehicle vector.

검증: ros2 topic hz /vehicle/vector  (≈100Hz), ros2 topic echo /vehicle/vector (x 증가)
watchdog 검증: dummy_ref_publisher 프로세스 kill → sim 로그에 TIMEOUT, v→0
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bridge_dspace',
            executable='dspace_sim_node',
            parameters=[{'pc_ip': '127.0.0.1'}],
            output='screen',
        ),
        Node(
            package='bridge_dspace',
            executable='udp_bridge_node',
            parameters=[{'dspace_ip': '127.0.0.1'}],
            output='screen',
        ),
        Node(
            package='bridge_dspace',
            executable='dummy_ref_publisher',
            parameters=[{'v_ref': 0.3}],
            output='screen',
        ),
    ])
