"""라이다 단독 회피주행 폐루프 시뮬레이션 — 실제 stack_avoid 로직 검증.

  ros2 launch stack_avoid avoid_sim.launch.py
  ros2 launch stack_avoid avoid_sim.launch.py obstacles:="3.0,0.3; 5.0,-0.3" start_x:=0.0

구성: avoid_drive_sim(/scan+차량적분) → stack_avoid_node(/perception/avoid)
      → avoid_viz(/avoid_markers) + RViz(map 프레임).
장애물은 map 프레임 고정 위치, 차량이 그 앞에서 출발해 회피하며 지나감.
범위 내 장애물이 사라지면 정지.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('stack_avoid')
    params = os.path.join(pkg, 'config', 'params.yaml')
    rviz_cfg = os.path.join(pkg, 'config', 'avoid_sim.rviz')

    obstacles = LaunchConfiguration('obstacles')
    start_x = LaunchConfiguration('start_x')
    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('obstacles', default_value='3.0,0.3',
                              description='장애물 map 좌표 "x1,y1; x2,y2" (m)'),
        DeclareLaunchArgument('start_x', default_value='0.0',
                              description='차량 시작 x (map, m)'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # 폐루프 시뮬레이터 (스캔 생성 + 차량 적분 + 정지판단)
        Node(package='stack_avoid', executable='avoid_drive_sim', name='avoid_drive_sim',
             output='screen',
             parameters=[{
                 'obstacles': obstacles, 'start_x': start_x,
                 'target_speed_mps': 0.5, 'wheelbase_m': 0.595, 'max_steer_deg': 27.3,
                 'vehicle_width_m': 0.62, 'vehicle_length_m': 0.85,
                 'lidar_x_m': 0.76, 'forward_angle_deg': 270.0,
                 'detect_range_m': 3.0, 'fov_half_deg': 90.0, 'dt': 0.05,
                 'lookahead_m': 0.7,   # 실차 MPC 근사 추종. 작을수록 목표 오프셋에 빨리 붙음
             }]),

        # 실제 회피 노드 (방향 270 고정)
        Node(package='stack_avoid', executable='stack_avoid_node', name='stack_avoid_node',
             output='screen', parameters=[params]),

        # 회피 출력 시각화 (corridor/FOV/목표점, base_link)
        Node(package='stack_avoid', executable='avoid_viz', name='avoid_viz', output='screen',
             parameters=[{'lidar_x_m': 0.76, 'vehicle_width_m': 0.62, 'lateral_margin_m': 0.15,
                          'detect_range_m': 3.0, 'offset_max_m': 1.0, 'roi_angle_deg': 180.0}]),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='log',
             condition=IfCondition(use_rviz)),
    ])
