"""One read-only RViz window. Does not start a controller, sensor, CAN or recorder."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('adas_mgm'))
    parking = Path(get_package_share_directory('stack_parking')) / 'config/parking_params.yaml'
    config = yaml.safe_load(parking.read_text())
    params = next(iter(config.values()))['ros__parameters']['vehicle']
    display = Node(package='adas_mgm', executable='integration_view.py', output='screen',
                   parameters=[{'vehicle_front_m': float(params['front_m']),
                                'vehicle_rear_m': float(params['rear_m']),
                                'vehicle_width_m': float(params['width_m'])}])
    rviz = Node(package='rviz2', executable='rviz2', name='integration_v2_rviz',
                arguments=['-d', str(share/'config/integration_v2.rviz')], output='screen')
    return LaunchDescription([display, rviz,
        RegisterEventHandler(OnProcessExit(target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason='Unified RViz window closed'))])),
        RegisterEventHandler(OnProcessExit(target_action=display,
            on_exit=[EmitEvent(event=Shutdown(reason='Visualization bridge exited'))]))])
