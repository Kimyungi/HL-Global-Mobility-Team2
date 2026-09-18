"""Explicit LiDAR E-stop excluded test entry; all other v2 guards remain."""
import importlib.util
from pathlib import Path


def generate_launch_description():
    raise RuntimeError('Retired v2 launcher excluded; use scripts/v2 prepare and RUN_BOOK_HALLA_FINAL.md')
