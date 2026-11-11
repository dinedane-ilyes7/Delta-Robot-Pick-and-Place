"""Compatibility settings for the CAD-v2 kinematics module.

The source archive did NOT contain analysis/params.py, although
analysis/kinematics.py imports it. This small compatibility file was reconstructed
from config/robot.yaml and documented design workspace numbers; it is NOT
an original code file from the project source archive.
"""
from pathlib import Path
import numpy as np
import yaml

config_path = Path(__file__).resolve().parents[1] / "config" / "robot.yaml"
with config_path.open(encoding="utf-8") as f:
    robot = yaml.safe_load(f)
PHI = np.deg2rad(robot["geometry"]["phi_deg"])
# Optional task-point sampling defaults, inferred from CAD README's 120 mm radius
# and the reported ~-215..-300 mm working-Z examples; validate before scientific use.
TASK_RADIUS = 0.120
TASK_STROKE = 0.085
