"""실물 하드웨어(LiDAR + CAN) 회피 테스트 + 로깅.

구성: ydlidar 드라이버 → stack_avoid_node → avoid_viz + RViz
      + bridge_dspace(can0, /vehicle/vector RX) + rosbag record.
Ctrl+C 한 번으로 전체(드라이버·노드·rviz·브리지·rosbag) 정리.

실행은 tools/run_avoid_hw_test.sh 사용(워크스페이스 소싱 + can0 활성화 + 타임스탬프 로그).
직접:  ros2 launch stack_avoid avoid_hw_log.launch.py bag_dir:=/경로/bag can:=true rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('stack_avoid')
    params = os.path.join(pkg, 'config', 'params.yaml')
    rviz_cfg = os.path.join(pkg, 'config', 'avoid_test.rviz')
    ydlidar_launch = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')

    bag_dir = LaunchConfiguration('bag_dir')
    use_can = LaunchConfiguration('can')
    use_rviz = LaunchConfiguration('rviz')

    # 로그로 남길 토픽 (스캔·회피출력·마커·TF·CAN RX·노드로그)
    bag_topics = [
        '/scan', '/scan_front', '/perception/avoid', '/avoid_markers',
        '/tf', '/tf_static', '/vehicle/vector', '/rosout',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('bag_dir', default_value='avoid_hw_bag',
                              description='rosbag 출력 경로'),
        DeclareLaunchArgument('can', default_value='true',
                              description='CAN 브리지(can0) 실행 여부'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # 1) 실 LiDAR 드라이버 → /scan
        IncludeLaunchDescription(PythonLaunchDescriptionSource(ydlidar_launch)),

        # 2) 회피 노드 (방향 270 고정) + 시각화
        Node(package='stack_avoid', executable='stack_avoid_node', name='stack_avoid_node',
             output='screen', parameters=[params]),
        Node(package='stack_avoid', executable='avoid_viz', name='avoid_viz', output='screen',
             parameters=[{'lidar_x_m': 0.76, 'vehicle_width_m': 0.62, 'lateral_margin_m': 0.15,
                          'detect_range_m': 3.0, 'offset_max_m': 1.0, 'roi_angle_deg': 180.0}]),

        # 3) CAN 브리지 (dSPACE RX → /vehicle/vector). dSPACE 미연결이어도 기동은 됨.
        Node(package='bridge_dspace', executable='can_bridge_node', name='can_bridge_node',
             output='screen', parameters=[{'can_interface': 'can0'}],
             condition=IfCondition(use_can)),

        # 4) RViz
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='log', condition=IfCondition(use_rviz)),

        # 5) rosbag 기록 (로그)
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_dir] + bag_topics,
            output='screen'),
    ])
