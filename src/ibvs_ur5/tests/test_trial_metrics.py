#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from trial_metrics import TrialJudge, is_formal_trial


class TrialJudgeTest(unittest.TestCase):
    def make_trial(self):
        judge = TrialJudge(requested_at=0.0)
        judge.command(1.0)
        return judge

    def test_stable_hold_reports_start_not_confirmation(self):
        judge = self.make_trial()
        for tick in range(10, 51):
            judge.observe(tick / 10, 10.0 if tick < 20 else 4.0)
        result = judge.summary()
        self.assertEqual(result['end_reason'], 'converged')
        self.assertEqual(result['convergence_seconds'], 1.0)
        self.assertEqual(result['hold_seconds_observed'], 3.0)
        self.assertEqual(result['final_error_mean_px'], 4.0)
        self.assertEqual(result['stale_intervals'], 0)

    def test_exact_threshold_and_rebound_restart_hold(self):
        judge = self.make_trial()
        # 额外给一个新样本；二进制浮点的 6.1 - 3.1 可能略小于 3。
        for tick in range(10, 63):
            judge.observe(tick / 10, 5.0 if tick == 30 else 4.0)
        self.assertTrue(judge.summary()['geometric_converged'])
        self.assertAlmostEqual(judge.summary()['convergence_seconds'], 2.1)

    def test_no_new_data_cannot_finish_hold_and_final_is_missing(self):
        judge = self.make_trial()
        for tick in range(10, 39):
            judge.observe(tick / 10, 4.0)
        judge.poll(4.1)  # 最后一个值还新鲜，但没有新样本确认三秒保持。
        self.assertIsNone(judge.end_reason)
        judge.poll(5.0)
        judge.poll(6.0)
        self.assertEqual(judge.stale_intervals, 1)
        judge.poll(31.0)
        self.assertFalse(judge.summary()['geometric_converged'])
        self.assertIsNone(judge.summary()['final_error_mean_px'])

    def test_gap_recovery_needs_a_new_full_hold(self):
        judge = self.make_trial()
        for tick in range(10, 31):
            judge.observe(tick / 10, 4.0)
        for tick in range(40, 71):
            judge.observe(tick / 10, 4.0)
        self.assertEqual(judge.summary()['convergence_seconds'], 3.0)
        self.assertEqual(judge.stale_intervals, 1)

    def test_delayed_or_invalid_sample_breaks_hold(self):
        for error, age in ((4.0, 0.6), (float('nan'), 0.0), (4.0, -1.0)):
            with self.subTest(error=error, age=age):
                judge = self.make_trial()
                for tick in range(10, 30):
                    judge.observe(tick / 10, 4.0)
                judge.observe(3.0, error, age)
                self.assertIsNone(judge.hold_start)
                self.assertEqual(judge.invalid_samples, 1)

    def test_hold_must_finish_inside_observation_window(self):
        judge = self.make_trial()
        for tick in range(10, 311):
            judge.observe(tick / 10, 10.0 if tick < 290 else 4.0)
        judge.poll(31.0)
        self.assertEqual(judge.end_reason, 'observation_timeout')
        judge.observe(32.0, 4.0)
        self.assertFalse(judge.summary()['geometric_converged'])

    def test_no_command_is_a_separate_startup_timeout(self):
        judge = TrialJudge(requested_at=100.0)
        judge.observe(110.0, 0.0)
        judge.poll(130.0)
        judge.command(131.0)
        self.assertEqual(judge.end_reason, 'startup_timeout')
        self.assertIsNone(judge.first_command)

    def test_missing_observation_restarts_hold(self):
        judge = self.make_trial()
        for tick in range(10, 30):
            judge.observe(tick / 10, 4.0)
        judge.interrupt()
        for tick in range(30, 61):
            judge.observe(tick / 10, 4.0)
        self.assertEqual(judge.summary()['convergence_seconds'], 2.0)
        self.assertEqual(judge.interruptions, 1)

    def test_clean_control_timeout_remains_a_formal_failed_trial(self):
        self.assertTrue(is_formal_trial('observation_timeout', '', []))
        self.assertTrue(is_formal_trial('converged', '', []))
        self.assertFalse(is_formal_trial('startup_timeout', '', []))
        self.assertFalse(is_formal_trial('converged', ' M controller.py', []))
        self.assertFalse(is_formal_trial('converged', '', ['clock_events']))


if __name__ == '__main__':
    unittest.main()
