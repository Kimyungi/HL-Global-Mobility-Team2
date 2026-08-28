"""실기용: can_bridge_node 단독 (dSPACE 실물 CAN 연결).

사전 준비 (최초 1회 — 이후 어댑터를 꽂으면 CAN FD 로 자동 up, PROTOCOL.md):
    sudo src/bridge_dspace/tools/can_setup/install.sh

와이어 포맷 (2026-08-28 Kvaser Leaf v3 + CAN FD 이관):
    기본은 CAN FD (BRS on) — 팀 표준. 프레임 레이아웃·ID·스케일은 classic 과 동일하다.
    classic 으로 되돌려 대조하려면:  ros2 launch bridge_dspace bridge.launch.py can_fd:=false
    dSPACE 가 BRS 를 못 켜는 경우:   ... can_fd_brs:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument(
            'can_fd', default_value='true',
            description='CAN FD 프레임으로 송신 (false = classic 2.0A, A/B 대조용)'),
        DeclareLaunchArgument(
            'can_fd_brs', default_value='true',
            description='데이터 구간 비트레이트 전환(BRS). dSPACE 설정과 일치시킬 것'),
        Node(
            package='bridge_dspace',
            executable='can_bridge_node',
            parameters=[{
                'can_interface': LaunchConfiguration('can_interface'),
                # bool 파라미터는 value_type 을 명시해야 한다 — 안 하면 "true" 가
                # 문자열로 넘어가 노드가 타입 불일치로 죽는다 (launch_parts.py 선례)
                'can_fd': ParameterValue(LaunchConfiguration('can_fd'), value_type=bool),
                'can_fd_brs': ParameterValue(
                    LaunchConfiguration('can_fd_brs'), value_type=bool),
            }],
            output='screen',
        ),
    ])
