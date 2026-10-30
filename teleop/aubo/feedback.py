"""Host-received feedback; timestamps do not claim controller sampling time."""
from dataclasses import dataclass
import math
import threading
import time

_RECEIVED_NOW = object()


class OlderFeedback(ValueError):
    """An older packet was discarded; a newer valid sample is preserved."""


def joints(value):
    q = tuple(float(x) for x in value)
    if len(q) != 6 or not all(math.isfinite(x) for x in q):
        raise ValueError("Expected six finite joint positions in radians")
    return q


@dataclass(frozen=True)
class Feedback:
    q_rad: tuple | None
    received_monotonic_s: float
    source: str
    valid: bool
    return_code: int | None
    reason: str | None = None
    controller_sample_time: None = None


class FeedbackCache:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self._lock = threading.Lock()
        self._latest = None

    def update(self, q_rad, *, source, return_code=0, received_monotonic_s=_RECEIVED_NOW):
        now = self.clock()
        try:
            stamp = now if received_monotonic_s is _RECEIVED_NOW else float(received_monotonic_s)
        except (TypeError, ValueError, OverflowError):
            self.invalidate("invalid_feedback_timestamp", source=source)
            raise ValueError("Unparseable host reception timestamp") from None
        if not math.isfinite(stamp) or stamp > now:
            self.invalidate("invalid_feedback_timestamp", source=source)
            raise ValueError("Invalid/future host reception timestamp")
        q, reason = None, None
        if return_code != 0:
            reason = "sdk_query_failed"
        else:
            try:
                q = joints(q_rad)
            except (TypeError, ValueError, OverflowError):
                reason = "invalid_joint_feedback"
        sample = Feedback(q, stamp, source, reason is None, return_code, reason)
        with self._lock:
            if self._latest and stamp < self._latest.received_monotonic_s:
                raise OlderFeedback("Out-of-order host reception timestamp; newer sample preserved")
            self._latest = sample
        return sample

    def invalidate(self, reason, *, source="host"):
        with self._lock:
            self._latest = Feedback(None, self.clock(), source, False, None, reason)

    def latest(self):
        with self._lock:
            return self._latest

    def require_recent(self, max_age_s):
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("Feedback age limit must be explicitly positive")
        sample = self.latest()
        if sample is None or not sample.valid:
            raise RuntimeError(sample.reason if sample else "feedback_missing")
        age = self.clock() - sample.received_monotonic_s
        if age < 0 or age > max_age_s:
            raise RuntimeError("feedback_reception_expired")
        return sample
