"""회피 로직 테스트 (합성 스캔) — 하드웨어 없이 전체 체인 + RViz 시각화.

  ros2 launch stack_avoid avoid_test.launch.py
  ros2 launch stack_avoid avoid_test.launch.py obstacles:="2.0,0.4; 2.0,-0.4"
  ros2 launch stack_avoid avoid_test.launch.py rviz:=false

구성: fake_scan(/scan) → stack_avoid_node(/perception/avoid) → avoid_viz(/avoid_markers) + RViz.
장애물은 런타임에도 변경: ros2 param set /fake_scan obstacles "1.5,0.2; 1.5,-0.2"
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
    rviz_cfg = os.path.join(pkg, 'config', 'avoid_test.rviz')

    obstacles = LaunchConfiguration('obstacles')
    use_rviz = LaunchConfiguration('rviz')
    use_fake = LaunchConfiguration('fake')   # true=합성스캔 / false=실라이다(/scan 외부 공급)

    # fake_scan / avoid_viz 는 params.yaml 실측값과 동일하게 맞춘다
    common = {
        'lidar_x_m': 0.76,
        'forward_angle_deg': 270.0,
    }

    return LaunchDescription([
        DeclareLaunchArgument('obstacles', default_value='2.0,0.0',
                              description='장애물 vehicle 좌표 "x1,y1; x2,y2" (m)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='RViz 동시 실행 여부'),
        DeclareLaunchArgument('fake', default_value='true',
                              description='true=합성스캔 / false=실라이다(드라이버 별도 실행)'),

        # 1) 합성 스캔 (fake:=false 면 실라이다 드라이버가 /scan 공급 — 이 노드 비활성)
        Node(package='stack_avoid', executable='fake_scan', name='fake_scan',
             output='screen', condition=IfCondition(use_fake),
             parameters=[dict(common, obstacles=obstacles, rate_hz=10.0)]),

        # 2) 회피 노드 (실측 파라미터)
        Node(package='stack_avoid', executable='stack_avoid_node', name='stack_avoid_node',
             output='screen', parameters=[params]),

        # 3) 회피 출력 시각화
        Node(package='stack_avoid', executable='avoid_viz', name='avoid_viz',
             output='screen',
             parameters=[dict(common,
                              vehicle_width_m=0.62, lateral_margin_m=0.15,
                              detect_range_m=3.0, offset_max_m=1.0, roi_angle_deg=180.0)]),

        # 4) RViz
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='log',
             condition=IfCondition(use_rviz)),
    ])
