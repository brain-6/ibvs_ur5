#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

from ibvs_math import feature_position_and_jacobian  # noqa: E402


class IBVSMathTest(unittest.TestCase):
    def test_geometric_jacobian_matches_central_difference(self):
        configurations = [
            np.zeros(6),
            np.array([0.4, -1.1, 1.0, -0.7, 0.6, -0.2]),
            np.array([-0.8, -1.7, 1.4, 0.5, -0.9, 1.2]),
        ]
        epsilon = 1e-7

        for q in configurations:
            _, jacobian = feature_position_and_jacobian(q)
            numeric = np.empty((3, 6))

            for joint in range(6):
                step = np.zeros(6)
                step[joint] = epsilon

                p_plus, _ = feature_position_and_jacobian(q + step)
                p_minus, _ = feature_position_and_jacobian(q - step)
                numeric[:, joint] = (p_plus - p_minus) / (2.0 * epsilon)

            np.testing.assert_allclose(jacobian, numeric, atol=1e-8)

    def test_zero_pose_is_expressed_in_ros_base_link(self):
        position, _ = feature_position_and_jacobian(np.zeros(6))
        self.assertGreater(position[0], 0.0)
        self.assertGreater(position[1], 0.0)


if __name__ == '__main__':
    unittest.main()
