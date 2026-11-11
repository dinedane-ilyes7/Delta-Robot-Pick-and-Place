"""Source-only checks for the CAD v2 Delta kinematics.

These tests check mathematical consistency and do not require MuJoCo, CadQuery,
stepper motors, or a physical robot.
"""
from pathlib import Path
import sys
import unittest

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis"))

from kinematics import Delta  # noqa: E402


class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with (ROOT / "config" / "robot.yaml").open(encoding="utf-8") as f:
            g = yaml.safe_load(f)["geometry"]
        cls.robot = Delta(g["R"], g["r"], g["L1"], g["L2"])

    def test_center_reference_pose(self):
        reference = np.array([0.0, 0.0, -0.240])
        angles, valid = self.robot.ik(reference)
        self.assertTrue(bool(valid))
        np.testing.assert_allclose(angles, angles[0], atol=1e-10)
        recovered = self.robot.fk(angles)
        np.testing.assert_allclose(recovered, reference, atol=1e-8)

    def test_multiple_reachable_poses(self):
        poses = [
            np.array([0.0, 0.0, -0.240]),
            np.array([0.030, 0.015, -0.245]),
            np.array([-0.025, 0.020, -0.260]),
        ]
        for target in poses:
            with self.subTest(target=target.tolist()):
                angles, valid = self.robot.ik(target)
                self.assertTrue(bool(valid))
                np.testing.assert_allclose(self.robot.fk(angles), target, atol=1e-7)


if __name__ == "__main__":
    unittest.main()
