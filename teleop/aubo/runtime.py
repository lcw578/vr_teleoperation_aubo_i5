"""Single-owner offline runtime for rehearsing the future AUBO entry.

This module deliberately has no SDK motion path.  It serializes transport
events into the existing author core plus the fake-only coordinator, keeps
motion frames latest-only, preserves input faults, and runs a separate
watchdog check.  A real follower must implement the same lifecycle after the
controller's FollowMode and stop semantics are verified.
"""
from collections import deque
import math
import threading
import time

from .input_timing import InputTiming


class OfflineRuntime:
    """Thread-safe transport adapter around ``AuboShadowCoordinator``.

    The receiver side never calls IK.  ``step`` is the sole owner of the
    coordinator and fake dispatch.  This mirrors the intended hardware shape:
    network/callback threads publish state, while one control owner advances
    the teleoperator and sends at most one latest target.
    """

    def __init__(self, shadow, *, control_dt, clock=time.monotonic, dispatch_fake=True, expected_reference_id=None):
        if not math.isfinite(control_dt) or control_dt <= 0:
            raise ValueError("Explicit positive control period required")
        self.expected_reference_id = expected_reference_id
        self.shadow = shadow
        self.control_dt = float(control_dt)
        self.clock = clock
        self.dispatch_enabled = bool(dispatch_fake)
        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_frame_received = None
        self._input_faults = deque(maxlen=8)
        self._release_frame = None
        self._last_received = None
        self._last_epoch = None
        self._last_step = None
        self._closed = False
        self._input_diagnostics = dict(received=0, accepted=0, rejected=0,
            last_error=None, right_present=False, grip=None, button_count=0, axis_count=0)
        self._last_valid_received = None
        self.input_timing = InputTiming()

    def submit_xr_frame(self, frame, *, received_monotonic_s=None):
        """Publish one frame; motion frames are latest-only.

        Malformed frames are retained as fault events instead of being silently
        replaced by a later valid frame.  This method does not call the IK.
        """
        if self._closed:
            return False
        stamp = self.clock() if received_monotonic_s is None else received_monotonic_s
        self.input_timing.received(frame, stamp)
        if (self.expected_reference_id is not None and
                (not isinstance(frame,dict) or frame.get("reference_id") != self.expected_reference_id)):
            self.submit_fault("workspace_alignment_reference_mismatch")
            with self._lock:
                self._input_diagnostics["rejected"] += 1
                self._input_diagnostics["last_error"] = "workspace_alignment_reference_mismatch"
            return False
        right = (frame.get("controllers") or {}).get("right") if isinstance(frame, dict) and isinstance(frame.get("controllers"), dict) else None
        buttons = right.get("buttons") if isinstance(right, dict) else None
        axes = right.get("axes") if isinstance(right, dict) else None
        with self._lock:
            self._input_diagnostics["received"] += 1
            self._input_diagnostics.update(right_present=isinstance(right, dict), grip=None,
                button_count=len(buttons) if isinstance(buttons, list) else 0,
                axis_count=len(axes) if isinstance(axes, list) else 0)
        from .shadow import normalize_xr_frame
        try:
            frame = normalize_xr_frame(frame)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            with self._lock:
                self._input_diagnostics["rejected"] += 1
                self._input_diagnostics["last_error"] = (
                    "right_controller_missing" if not isinstance(right, dict) else type(exc).__name__ + ": " + str(exc)[:160])
            self.submit_fault("invalid_xr_frame")
            return False
        try:
            stamp = float(stamp)
            valid_stamp = math.isfinite(stamp) and stamp <= self.clock()
        except (TypeError, ValueError, OverflowError):
            valid_stamp = False
        with self._lock:
            if not valid_stamp:
                self._input_faults.append("invalid_input_reception_timestamp")
                self._latest_frame = None
                self._latest_frame_received = None
                return False
            if self._last_received is not None and stamp < self._last_received:
                self._input_faults.append("out_of_order_input_reception")
                return False
            self._last_received = stamp
            self._last_valid_received = stamp
            self._input_diagnostics["accepted"] += 1
            self._input_diagnostics["grip"] = frame["controllers"]["right"]["buttons"][1]["p"]
            if self._last_epoch is not None and frame["epoch"] != self._last_epoch:
                self._input_faults.append("reference_changed")
                self._release_frame = None
            self._last_epoch = frame["epoch"]
            if not frame["controllers"]["right"]["buttons"][1]["p"]:
                self._release_frame = (frame, stamp)
            self._latest_frame, self._latest_frame_received = frame, stamp
        return True

    def submit_fault(self, reason):
        """Transport threads only publish faults; control owner resets the core."""
        with self._lock:
            self._input_faults.append(str(reason))
            self._latest_frame = self._latest_frame_received = None
            self._release_frame = None

    def submit_feedback(self, q_rad, *, received_monotonic_s=None, source="fake_only"):
        """Publish feedback through the thread-safe coordinator cache."""
        if self._closed:
            return False
        stamp = self.clock() if received_monotonic_s is None else received_monotonic_s
        self.shadow.feedback(q_rad, received_monotonic_s=stamp, source=source)
        return True

    def watchdog_tick(self):
        """Check input/feedback without advancing IK or dispatching motion."""
        if self._closed:
            return False
        with self._lock:
            faults = list(self._input_faults)
            self._input_faults.clear()
            received = self._latest_frame_received
        if faults:
            self.shadow.reject_input(faults[-1])
            # Do not use any frame received before this fault was consumed as
            # a recovery release. A fresh release must arrive on a later tick.
            with self._lock:
                self._latest_frame = self._latest_frame_received = None
                self._release_frame = None
        elif (self.shadow.session.engaged and received is not None and
              self.clock() - received > self.shadow.session.policy.input_max_age_s):
            self.shadow.reject_input("input_timeout")
        return self.shadow.watchdog_tick()

    def step(self, *, joint_target=None):
        """Advance one control tick and dispatch only when the period allows."""
        if self._closed:
            return None
        self.watchdog_tick()
        with self._lock:
            if self._release_frame is not None:
                frame, received = self._release_frame
                self._release_frame = None
                if self._latest_frame is frame:
                    self._latest_frame = self._latest_frame_received = None
            else:
                frame, received = self._latest_frame, self._latest_frame_received
                self._latest_frame = self._latest_frame_received = None
        if frame is not None:
            if self.clock() - received > self.shadow.session.policy.input_max_age_s:
                self.shadow.reject_input("input_invalid_or_expired")
            else:
                self.shadow.accept_xr_frame(frame, received_monotonic_s=received)
                self.input_timing.processed(self.shadow.last_input_event)
        candidate = self.shadow.tick(joint_target=joint_target) if joint_target is not None else self.shadow.tick()
        if candidate is None:
            self._last_step = self.clock()
            return None
        if not self.dispatch_enabled:
            self._last_step = self.clock()
            return candidate
        # Do not dispatch in the same instant as engagement.  The candidate
        # remains in the single pending slot and will be considered next tick.
        last_submit = self.shadow.session.last_submit_s
        now = self.clock()
        if last_submit is None or now - last_submit < self.control_dt:
            self._last_step = now
            return candidate
        self.shadow.dispatch_fake()
        self._last_step = now
        return candidate

    def status(self):
        session = self.shadow.session
        with self._lock:
            diagnostics = dict(self._input_diagnostics,
                valid_frame_age_s=None if self._last_valid_received is None else self.clock() - self._last_valid_received)
        return {
            "closed": self._closed,
            "engaged": session.engaged,
            "fault": session.fault,
            "require_release": session.require_release,
            "pending": session.pending is not None,
            "session": session.session,
            "last_submit_monotonic_s": session.last_submit_s,
            "last_step_monotonic_s": self._last_step,
            "backend": "fake_only" if self.dispatch_enabled else "candidate_only",
            "motion_available": False,
            "xr_input": diagnostics,
            "input_timing": self.input_timing.snapshot(),
        }

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.shadow.close()
