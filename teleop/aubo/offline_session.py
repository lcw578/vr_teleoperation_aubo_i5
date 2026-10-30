"""Offline command/session checks. No real SDK or Quest imports.

Only FakePositionBackend can run this preparation harness. Hardware stop,
threading and controller readiness must be implemented/verified separately.
"""
from dataclasses import dataclass
import math
import time

from .feedback import FeedbackCache, joints


@dataclass(frozen=True)
class SessionPolicy:
    # Required explicit values, not guessed AUBO defaults.
    joint_min_rad: tuple
    joint_max_rad: tuple
    velocity_rad_s: tuple
    feedback_max_age_s: float
    input_max_age_s: float
    command_max_age_s: float

    def __post_init__(self):
        lo, hi, vel = map(joints, (self.joint_min_rad, self.joint_max_rad, self.velocity_rad_s))
        if any(a >= b for a, b in zip(lo, hi)) or any(v <= 0 for v in vel):
            raise ValueError("Explicit ordered ranges and positive velocities required")
        for age in (self.feedback_max_age_s, self.input_max_age_s, self.command_max_age_s):
            if not math.isfinite(age) or age <= 0:
                raise ValueError("Explicit positive age limits required")
        object.__setattr__(self, "joint_min_rad", lo)
        object.__setattr__(self, "joint_max_rad", hi)
        object.__setattr__(self, "velocity_rad_s", vel)


class FakePositionBackend:
    """Instant fake target acceptance, with manually supplied fake measurement."""
    source = "fake_only"

    def __init__(self):
        self.submitted = []
        self.stop_requests = 0
        self.fail_next = None
        self.fail_stop_next = None

    def send_position(self, q_rad):
        self.submitted.append(q_rad)
        failure, self.fail_next = self.fail_next, None
        if failure:
            raise failure  # Can model submission followed by lost response.
        return 0

    def stop(self):
        self.stop_requests += 1  # No claim about physical stop effectiveness.
        failure, self.fail_stop_next = self.fail_stop_next, None
        if failure:
            raise failure


@dataclass(frozen=True)
class Candidate:
    q_rad: tuple
    generated_monotonic_s: float
    session: int


class OfflinePositionSession:
    def __init__(self, backend, policy, *, clock=time.monotonic, cache=None):
        if type(backend) is not FakePositionBackend:
            raise TypeError("This harness accepts only FakePositionBackend; real motion unavailable")
        self.backend, self.policy, self.clock = backend, policy, clock
        self.cache = cache if cache is not None else FeedbackCache(clock)
        self.engaged = False
        self.require_release = True
        self.fault = None
        self.session = 0
        self.pending = None
        self.last_reference = None
        self.last_submit_s = None
        self.last_input_s = None
        self.last_candidate_s = None
        self.attempts = []
        self.stop_error = None

    def _request_stop(self):
        try:
            self.backend.stop()
        except Exception as exc:
            self.stop_error = type(exc).__name__
            self.fault = "fake_stop_failed"
            self.require_release = True

    def _fail(self, reason):
        active = self.engaged
        self.engaged = False
        self.require_release = True
        self.pending = None
        self.session += 1
        self.fault = reason
        if active:
            self._request_stop()

    def revoke(self, reason):
        """External input/model failure; revoke before requesting fake stop."""
        self._fail(reason)

    def require_feedback(self):
        sample = self.cache.require_recent(self.policy.feedback_max_age_s)
        if any(not lo <= q <= hi for q, lo, hi in zip(
                sample.q_rad, self.policy.joint_min_rad, self.policy.joint_max_rad)):
            raise RuntimeError("measured_position_outside_working_range")
        return sample

    def input(self, *, grip, received_monotonic_s, tracking_valid):
        now = self.clock()
        try:
            stamp = float(received_monotonic_s)
        except (TypeError, ValueError, OverflowError):
            self._fail("input_invalid_or_expired")
            return
        if (type(grip) is not bool or tracking_valid is not True or not math.isfinite(stamp) or stamp > now or
                now - stamp > self.policy.input_max_age_s or
                (self.last_input_s is not None and stamp < self.last_input_s)):
            self._fail("input_invalid_or_expired")
            return
        self.last_input_s = stamp
        if not grip:
            was_active = self.engaged
            self.engaged = False
            self.pending = None
            self.session += 1
            self.require_release = self.stop_error is not None
            if was_active:
                self._request_stop()
            return
        if self.require_release or self.stop_error:
            return
        if self.engaged:
            self.check()
            return
        try:
            sample = self.require_feedback()
        except RuntimeError as exc:
            self._fail(str(exc))
            return
        self.session += 1
        self.last_reference = sample.q_rad  # Anchor from fake measurement, not old target.
        self.last_submit_s = now
        self.last_candidate_s = None
        self.pending = None
        self.fault = None
        self.engaged = True

    def check(self):
        if not self.engaged:
            return False
        now = self.clock()
        if self.last_input_s is None or now - self.last_input_s > self.policy.input_max_age_s:
            self._fail("input_timeout")
            return False
        try:
            self.require_feedback()
        except RuntimeError as exc:
            self._fail(str(exc))
            return False
        return True

    def offer(self, q_rad, *, generated_monotonic_s, session):
        if not self.check() or session != self.session:
            return False
        now = self.clock()
        try:
            stamp = float(generated_monotonic_s)
        except (TypeError, ValueError, OverflowError):
            self._fail("candidate_expired")
            return False
        if not math.isfinite(stamp) or stamp > now or now - stamp > self.policy.command_max_age_s:
            self._fail("candidate_expired")
            return False
        try:
            q = joints(q_rad)
        except (TypeError, ValueError, OverflowError):
            self._fail("invalid_candidate")
            return False
        if any(not lo <= x <= hi for x, lo, hi in zip(
                q, self.policy.joint_min_rad, self.policy.joint_max_rad)):
            self._fail("candidate_outside_working_range")
            return False
        if self.last_candidate_s is not None and stamp <= self.last_candidate_s:
            return False
        self.pending = Candidate(q, stamp, session)  # A single latest slot, no queue.
        self.last_candidate_s = stamp
        return True

    def dispatch(self):
        if not self.check() or self.pending is None:
            return False
        candidate, self.pending = self.pending, None
        now = self.clock()
        if candidate.session != self.session or now - candidate.generated_monotonic_s > self.policy.command_max_age_s:
            self._fail("candidate_expired")
            return False
        dt = now - self.last_submit_s
        # Do not multiply a long stall into permission for a large jump.
        if dt <= 0 or dt > self.policy.command_max_age_s:
            self._fail("submission_period_invalid")
            return False
        if any(abs(q - prev) > speed * dt + 1e-12 for q, prev, speed in zip(
                candidate.q_rad, self.last_reference, self.policy.velocity_rad_s)):
            self._fail("submission_step_exceeds_explicit_velocity_budget")
            return False
        attempt = {"source": "fake_only", "q_submitted_rad": candidate.q_rad,
                   "submitted_monotonic_s": now, "session": self.session,
                   "return_code": None, "outcome": "response_unknown"}
        self.attempts.append(attempt)
        try:
            rc = self.backend.send_position(candidate.q_rad)
            attempt["return_code"] = rc
            if rc != 0:
                attempt["outcome"] = "fake_rejected"
                self._fail("submission_failed")
                return False
        except Exception as exc:
            attempt["exception_type"] = type(exc).__name__
            self._fail("submission_response_unknown")
            return False
        attempt["outcome"] = "fake_accepted"
        self.last_reference = candidate.q_rad
        self.last_submit_s = now
        return True

    def disconnect(self):
        self._fail("disconnected")
        self.last_input_s = None
        self.cache.invalidate("disconnected", source="fake_only")
