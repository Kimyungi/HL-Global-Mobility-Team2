"""PR117 obstacle course using the shared v2 hardware, safety and view stack."""
import importlib.util
from pathlib import Path


def generate_launch_description():
    source = Path(__file__).resolve().with_name('REAL_VEHICLE_integration_v2_drive.launch.py')
    spec = importlib.util.spec_from_file_location('v2_obstacle_common', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description(route_profile='obstacle')
