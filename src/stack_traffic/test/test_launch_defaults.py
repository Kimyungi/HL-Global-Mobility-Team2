import importlib.util
import os
import unittest
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
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
        description = load_launch_module().generate_launch_description()
        context = LaunchContext()
        node_action = None

        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
            elif isinstance(entity, Node):
                node_action = entity

        self.assertIsNotNone(node_action)
        parameters = {}
        for key_substitutions, value in node_action._Node__parameters[0].items():
            key = perform_substitutions(context, list(key_substitutions))
            parameters[key] = (
                value.evaluate(context)
                if isinstance(value, ParameterValue)
                else value
            )

        self.assertEqual(
            parameters["oak_mxid"],
            "14442C10B167CFD200",
        )
        self.assertTrue(parameters["resume_on_green"])
        self.assertFalse(parameters["show_debug"])
        self.assertEqual(parameters["stopline_stop_y_ratio"], 0.0)


if __name__ == "__main__":
    unittest.main()
