"""Explicit LiDAR E-stop excluded test entry; all other v2 guards remain."""
import importlib.util
from pathlib import Path


def generate_launch_description():
    path = Path(__file__).with_name('REAL_VEHICLE_integration_v2.launch.py')
    spec = importlib.util.spec_from_file_location('integration_v2_vehicle_entry', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.build_launch_description(lidar_estop_enabled=False)
