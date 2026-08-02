#!/usr/bin/env python3
"""Stable entry point for the Phase 3 ROS 2 HIL CSV logger."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
IMPLEMENTATION = ROOT / "ADAS0.0.2/SoC/tests/navigation_validation/data_logger.py"
sys.path.insert(0, str(IMPLEMENTATION.parent))
runpy.run_path(str(IMPLEMENTATION), run_name="__main__")

