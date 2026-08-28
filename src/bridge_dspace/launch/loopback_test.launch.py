"""부트스트래핑 ① — PC 단독 루프백: dummy ref → can_bridge → dSPACE sim → vehicle vector.

사전 준비 (최초 1회 — vcan0 상시 생성, PROTOCOL.md):
    sudo src/bridge_dspace/tools/can_setup/install.sh --vcan

★ vcan0 는 **MTU 72** 여야 한다 (CAN FD 프레임 크기). install.sh --vcan 이 설정한다.
  MTU 16 인 예전 vcan0 이 남아 있으면 두 노드가 "CAN FD 활성화 실패" 로 죽는다 —
  조용히 classic 으로 떨어지지 않는 것은 의도다 (실기와 다른 포맷으로 통과하면
  루프백이 검증해야 할 것을 안 검증한 셈이 된다). 손으로 고치려면:
    sudo ip link set vcan0 down && sudo ip link set vcan0 mtu 72 && sudo ip link set vcan0 up

검증: ros2 topic hz /vehicle/vector  (≈100Hz), ros2 topic echo /vehicle/vector (x 증가)
watchdog 검증: dummy_ref_publisher 프로세스 kill → sim 로그에 TIMEOUT, v→0
저수준 확인: candump vcan0  또는  python3 tools/can_dump.py --iface vcan0
             (can_dump 은 각 줄에 STD/FD/FD-BRS 를 찍는다)
classic 대조: ros2 launch ... loopback_test.launch.py can_fd:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # bool 파라미터는 value_type 명시 필수 (launch_parts.py 선례)
    can_fd = ParameterValue(LaunchConfiguration('can_fd'), value_type=bool)
    can_fd_brs = ParameterValue(LaunchConfiguration('can_fd_brs'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='vcan0'),
        DeclareLaunchArgument(
            'can_fd', default_value='true',
            description='실기와 같은 CAN FD 로 루프백 (false = classic 2.0A 대조)'),
        DeclareLaunchArgument('can_fd_brs', default_value='true'),
        Node(
            package='bridge_dspace',
            executable='dspace_sim_node',
            parameters=[{'can_interface': LaunchConfiguration('can_interface'),
                         'can_fd': can_fd, 'can_fd_brs': can_fd_brs}],
            output='screen',
        ),
        Node(
            package='bridge_dspace',
            executable='can_bridge_node',
            parameters=[{'can_interface': LaunchConfiguration('can_interface'),
                         'can_fd': can_fd, 'can_fd_brs': can_fd_brs}],
            output='screen',
        ),
        Node(
            package='bridge_dspace',
            executable='dummy_ref_publisher',
            parameters=[{'v_ref': 0.3}],
            output='screen',
        ),
    ])
