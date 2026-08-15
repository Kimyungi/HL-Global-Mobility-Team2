import importlib.util
import os
import unittest
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue


def load_launch_module():
    launch_path = (
        Path(__file__).parents[1]
        / "launch"
        / "stopline_distance_test.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "stack_traffic_stopline_launch",
        launch_path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestLaunchDefaults(unittest.TestCase):
    def test_safe_headless_field_defaults(self):
        os.environ["ROS_LOG_DIR"] = "/tmp/stack_traffic_test_ros_logs"
        module = load_launch_module()
        description = module.generate_launch_description()
        context = LaunchContext()

        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)

        parameters = {
            key: (
                value.evaluate(context)
                if isinstance(value, ParameterValue)
                else value
            )
            for key, value in module.build_node_parameters().items()
        }

        self.assertEqual(
            parameters["oak_mxid"],
            module.DEFAULT_TRAFFIC_OAK_MXID,
        )
        self.assertEqual(parameters["oak_usb_speed"], "high")
        self.assertEqual(parameters["oak_fps"], 10.0)
        self.assertEqual(parameters["oak_width"], 1280)
        self.assertEqual(parameters["oak_height"], 720)
        self.assertFalse(parameters["oak_depth_enabled"])
        self.assertTrue(parameters["resume_on_green"])
        self.assertFalse(parameters["show_debug"])
        self.assertEqual(parameters["stopline_stop_y_ratio"], 0.0)


if __name__ == "__main__":
    unittest.main()
