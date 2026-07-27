#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

from ibvs_math import (  # noqa: E402
    feature_position_and_jacobian,
    scale_to_max_abs,
    scale_to_norm,
    damped_pseudoinverse,
)

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

    def test_limits_preserve_direction(self):
        vector = np.array([3.0, -4.0, 1.0])
        norm_limited = scale_to_norm(vector, 0.5)
        peak_limited = scale_to_max_abs(vector, 0.2)

        self.assertAlmostEqual(np.linalg.norm(norm_limited), 0.5)
        self.assertAlmostEqual(np.max(np.abs(peak_limited)), 0.2)
        np.testing.assert_allclose(
            norm_limited / norm_limited[0], vector / vector[0])
        np.testing.assert_allclose(
            peak_limited / peak_limited[0], vector / vector[0])

    def test_damped_pseudoinverse_suppresses_weak_direction(self):
        jacobian = np.array([
            [1.0, 0.0],
            [0.0, 1e-6],
        ])
        damping = 0.01

        inverse = damped_pseudoinverse(jacobian, damping)
        expected = np.array([
            [1.0 / (1.0 + damping ** 2), 0.0],
            [0.0, 1e-6 / (1e-12 + damping ** 2)],
        ])
        np.testing.assert_allclose(inverse, expected)

        q_dot = inverse @ np.array([1.0, 1.0])
        self.assertTrue(np.isfinite(q_dot).all())
        self.assertLess(abs(q_dot[1]), 0.1)

if __name__ == '__main__':
    unittest.main()
