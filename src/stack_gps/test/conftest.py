"""Test this checkout before any older sourced ROS workspace package."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
