#!/usr/bin/env python3
"""Phase 4 的小型判定器：时间由调用者传入，单元测试不依赖 ROS。"""
import math


def is_formal_trial(end_reason, git_status, anomalies):
    """干净版本中正常观测到超时也是有效失败样本，不能只收成功结果。"""
    return (end_reason in ('converged', 'observation_timeout') and
            not git_status and not anomalies)


class TrialJudge:
    def __init__(self, requested_at, threshold=5.0, hold_seconds=3.0,
                 observation_seconds=30.0, freshness_seconds=0.5,
                 startup_seconds=30.0):
        self.requested_at = requested_at
        self.threshold = threshold
        self.hold_seconds = hold_seconds
        self.observation_seconds = observation_seconds
        self.freshness_seconds = freshness_seconds
        self.startup_seconds = startup_seconds
        self.first_command = None
        self.hold_start = None
        self.last_valid = None
        self.ended_at = None
        self.end_reason = None
        self.samples = []
        self.stale_intervals = 0
        self.invalid_samples = 0
        self.interruptions = 0
        self._stale = False

    def command(self, now):
        if self.first_command is not None or self.end_reason is not None:
            return
        if now - self.requested_at >= self.startup_seconds:
            self.finish(now, 'startup_timeout')
        else:
            self.first_command = now

    def interrupt(self):
        """例如诊断消息丢失：不能跨越未知区间累计保持。"""
        if self.first_command is not None and self.end_reason is None:
            self.hold_start = None
            self.interruptions += 1

    def _check_gap(self, now):
        if self.first_command is None:
            return
        reference = self.last_valid if self.last_valid is not None else self.first_command
        if now - reference > self.freshness_seconds:
            self.hold_start = None
            if not self._stale:
                self.stale_intervals += 1
            self._stale = True

    def observe(self, now, error, sample_age=0.0):
        if self.first_command is None or self.end_reason is not None:
            return
        self._check_gap(now)
        if now - self.first_command > self.observation_seconds:
            self.finish(now, 'observation_timeout')
            return
        if (not math.isfinite(error) or error < 0 or
                not math.isfinite(sample_age) or
                not 0 <= sample_age <= self.freshness_seconds):
            self.hold_start = None
            self.invalid_samples += 1
            return
        self.last_valid = now
        self._stale = False
        self.samples.append((now, error))
        if error >= self.threshold:  # 严格小于 5；等于 5 也会中断保持。
            self.hold_start = None
            return
        if self.hold_start is None:
            self.hold_start = now
        # 只在新样本到达时确认成功，定时器不能靠旧值补足三秒。
        if now - self.hold_start >= self.hold_seconds:
            self.finish(now, 'converged')

    def poll(self, now):
        if self.end_reason is not None:
            return
        if self.first_command is None:
            if now - self.requested_at >= self.startup_seconds:
                self.finish(now, 'startup_timeout')
            return
        self._check_gap(now)
        if now - self.first_command >= self.observation_seconds:
            self.finish(now, 'observation_timeout')

    def finish(self, now, reason):
        if self.end_reason is None:
            self.ended_at = now
            self.end_reason = reason

    def summary(self):
        converged = self.end_reason == 'converged'
        final = []
        if (self.ended_at is not None and self.last_valid is not None and
                self.ended_at - self.last_valid <= self.freshness_seconds):
            final = [(t, e) for t, e in self.samples
                     if self.ended_at - self.hold_seconds <= t <= self.ended_at]
        return {
            'end_reason': self.end_reason,
            'geometric_converged': converged,
            'first_command_mono': self.first_command,
            'hold_start_mono': self.hold_start,
            'ended_at_mono': self.ended_at,
            'startup_seconds': (None if self.first_command is None else
                                self.first_command - self.requested_at),
            'convergence_seconds': (self.hold_start - self.first_command
                                    if converged else None),
            'hold_seconds_observed': (self.ended_at - self.hold_start
                                      if converged else None),
            'stale_intervals': self.stale_intervals,
            'invalid_samples': self.invalid_samples,
            'interrupted_intervals': self.interruptions,
            'final_error_mean_px': (sum(e for _, e in final) / len(final)
                                    if final else None),
            'final_error_max_px': max((e for _, e in final), default=None),
            'final_window_samples': len(final),
            'final_window_span_seconds': final[-1][0] - final[0][0] if final else None,
        }
