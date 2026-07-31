#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

from analyze_ibvs_log import (  # noqa: E402
    analyze_log_text,
    simplify_path,
)


class AnalyzeIBVSLogTest(unittest.TestCase):
    def test_successful_static_run_metrics(self):
        text = """
[INFO] [100.0] [ibvs_controller]: err=[ -4.0, 3.0] qd_max=0.100 sigma_min=0.020
[INFO] [101.0] [ibvs_controller]: err=[ -3.0, 3.0] qd_max=0.200 sigma_min=0.010
[INFO] [102.0] [ibvs_controller]: err=[ -2.0, 2.0] qd_max=0.150 sigma_min=0.030
[INFO] [103.0] [ibvs_controller]: Dead zone (err=2.8px). Holding.
[WARN] [99.0] [ibvs_controller]: Waiting for initial target; holding position.
"""
        result = analyze_log_text(text)

        self.assertTrue(result['success'])
        self.assertEqual(result['sample_count'], 3)
        self.assertAlmostEqual(result['initial_error_px'], 5.0)
        self.assertAlmostEqual(
            result['last_control_error_px'], 2.0 ** 0.5 * 2.0)
        self.assertAlmostEqual(result['convergence_time_sec'], 3.0)
        self.assertAlmostEqual(result['deadzone_error_px'], 2.8)
        self.assertAlmostEqual(
            result['raw_path_length_px'], 1.0 + 2.0 ** 0.5)
        self.assertAlmostEqual(
            result['straight_displacement_px'], 5.0 ** 0.5)
        self.assertAlmostEqual(result['path_efficiency'], 1.0)
        self.assertAlmostEqual(
            result['peak_commanded_joint_speed'], 0.2)
        self.assertAlmostEqual(result['minimum_sigma'], 0.01)
        self.assertEqual(result['initial_target_wait_warnings'], 1)

    def test_stale_input_marks_run_unsuccessful(self):
        text = """
[INFO] [10.0] [ibvs_controller]: err=[ -2.0, 2.0] qd_max=0.100 sigma_min=0.020
[INFO] [11.0] [ibvs_controller]: Dead zone (err=2.8px). Holding.
[WARN] [12.0] [ibvs_controller]: Moving target stale or missing; holding position.
"""
        result = analyze_log_text(text)

        self.assertFalse(result['success'])
        self.assertEqual(result['moving_target_stale_warnings'], 1)

    def test_pixel_staircase_is_simplified_to_a_straight_path(self):
        staircase = [
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (2.0, 1.0),
            (2.0, 2.0),
        ]

        simplified = simplify_path(staircase, tolerance=1.5)

        self.assertEqual(simplified, [(0.0, 0.0), (2.0, 2.0)])


if __name__ == '__main__':
    unittest.main()
