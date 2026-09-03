#!/usr/bin/env python3
"""Standalone entry point for the parallel-parking test node.

Prefer the complete launch command in normal testing:

    ros2 launch stack_parking parallel_parking_test.launch.py enable_control:=true
"""

from __future__ import annotations

from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from stack_parking.parallel_parking_node import main  # noqa: E402


if __name__ == '__main__':
    main()
