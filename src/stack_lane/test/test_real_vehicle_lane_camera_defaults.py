import importlib.util
import os
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue


def load_real_vehicle_launch_module():
    launch_path = (
        Path(__file__).parents[2]
        / "adas_mgm"
        / "launch"
        / "REAL_VEHICLE_lane_gps_can.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "real_vehicle_lane_gps_can_launch",
        launch_path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestRealVehicleLaneCameraDefaults(unittest.TestCase):
    def test_launch_wires_usb2_and_10fps_into_stack_lane(self):
        os.environ["ROS_LOG_DIR"] = "/tmp/stack_lane_launch_test_logs"
        module = load_real_vehicle_launch_module()
        with patch.object(
            module,
            "get_package_share_directory",
            return_value="/tmp/fake_adas_mgm_share",
        ):
            description = module.generate_launch_description()
        context = LaunchContext()
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)

        parameters = {
            key: (
                value.evaluate(context)
                if isinstance(value, ParameterValue)
                else value.perform(context)
            )
            for key, value in module.build_lane_camera_parameters().items()
        }

        self.assertEqual(parameters["camera_mxid"], "14442C105157D3D200")
        self.assertEqual(parameters["camera_fps"], 10)
        self.assertEqual(parameters["usb_speed"], "high")


if __name__ == "__main__":
    unittest.main()
