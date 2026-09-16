"""MGM only: no drivers/CAN; all driving topics use the /integration_v2 prefix."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    topics = (
        '/operator/go', '/operator/start_session', '/operator/stop',
        '/operator/cancel_mission', '/parking/mission_command',
        '/adas/mgm_state', '/adas/target_ref', '/perception/lane_path',
        '/perception/lane_camera', '/perception/traffic_camera', '/perception/gps_path', '/perception/avoid', '/perception/parking',
        '/perception/traffic_stop', '/perception/estop', '/bridge/can_health',
        '/vehicle/vector', '/avoid_v2/plan', '/adas/traffic_zone_enabled', '/planning/estop_recovery',
    )
    params = Path(get_package_share_directory('adas_mgm')) / 'config/params.yaml'
    return LaunchDescription([Node(
        package='adas_mgm', executable='mgm_node', name='mgm_node',
        parameters=[str(params), {
            'backend': 'core', 'base_state_machine_enabled': True,
            'wait_go': True, 'escape_after_cycles': 0,
        }],
        remappings=[(topic, '/integration_v2' + topic) for topic in topics],
        output='screen',
    )])
