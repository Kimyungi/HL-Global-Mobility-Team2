"""OAK central-crop perception diagnostics matching the training dataset."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    source = Path(get_package_share_directory('stack_traffic')) / 'launch' / 'stopline_distance_test.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(source)),
            launch_arguments={
                'central_traffic_only': 'true',
                'model_path': LaunchConfiguration('model_path'),
            }.items(),
        ),
    ])
