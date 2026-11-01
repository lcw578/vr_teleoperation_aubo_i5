"""Supervised worker; serial motion owner and optional independent state owner.

Independent watchdog revokes local permission while SDK is blocked. It cannot
interrupt the request or deliver an independent physical stop. Such a timeout
is latched, never automatically recovered, and requires the operator's stop.
"""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import os
import sys
import threading
import time
from collections import deque

from .feedback import FeedbackCache, joints
from .j6_trial import checked_snapshot, require_snapshot
from .probe import record_ok
from .readonly import load_native
from .state_query import StateQueryWorker


class ObservationPending(RuntimeError):
    """Known observation aged out; never usable for arm, send or stop evidence."""

    def __init__(self, message, snapshot):
        super().__init__(message)
        self.snapshot = snapshot


def load_follow_native(runtime=None):
    root = Path(runtime or os.environ.get("AUBO_RUNTIME_DIR", "/data/VR_teleoperation_runtime"))
    readonly = load_native(root)
    manifest = json.loads((root / "follow-build.json").read_text())
    library, binary = Path(manifest["release_library"]), Path(manifest["module_path"])
    if (library.resolve() != Path(readonly._isolated_sdk_handle._name).resolve() or
            hashlib.sha256(library.read_bytes()).hexdigest() != manifest["release_sha256"] or
            hashlib.sha256(binary.read_bytes()).hexdigest() != manifest["module_sha256"]):
        raise RuntimeError("Follower binary/SDK differs from recorded build")
    name = "_aubo_follow"
    module = sys.modules.get(name)
    if module is None:
        spec = importlib.util.spec_from_file_location(name, binary)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        sys.modules[name] = module
    elif Path(module.__file__).resolve() != binary.resolve():
        raise RuntimeError("A different follower binary is already loaded")
    return module


class J6FollowWorker:
    streaming_history_capacity = 4096
    period_s = .1                  # Legacy J6 cadence
    six_axis_period_s = .005       # Passthrough: 200 Hz requested
    six_axis_follow_period_s = .05 # Follow: restore previously tested 20 Hz
    send_max_gap_s = .65           # Existing gap budget retained; no catch-up
    speed_deg_s = 10.              # Legacy J6 trial only; not a six-axis setting
    client_acceleration_deg_s2 = 5.
    following_error_limit_deg = None  # Diagnostic difference only, no host lag cutoff
    input_max_age_s = 1.0         # Quest input gap only; Grip release revokes immediately
    candidate_max_age_s = .1
    sdk_budget_s = .5
    feedback_max_age_s = .2        # Reception age is separate from call duration
    idle_query_period_s = .05
    active_push_query_period_s = .1  # Host scheduling choice, not a cabinet rating
    # Experimental completion allowance for the blocking slow-stop operation.
    # This is separate from command/query latency and is not a vendor guarantee.
    stop_completion_budget_s = 2.
    stop_feedback_budget_s = 2.

    def __init__(self, client, anchor, *, j6_range_rad, clock=time.monotonic, diagnostics=False,
                 joint_ranges_rad=None, waypoint_feedback=False, sdk_profile_readback=None,
                 command_mode="follow", follow_hz=20., state_observer=None):
        self.client, self.anchor, self.clock = client, joints(anchor), clock
        self.six_axis = joint_ranges_rad is not None
        if command_mode not in ("follow", "passthrough") or (command_mode == "passthrough" and not self.six_axis):
            raise ValueError("Passthrough requires explicit six-axis scope")
        self.command_mode = command_mode
        if follow_hz not in (20., 25., 50., 200.) or (follow_hz != 20. and (not self.six_axis or command_mode != "follow")):
            raise ValueError("Follow rate requires six-axis follow and 20, 25, 50 or 200 Hz")
        if self.six_axis:
            self.period_s = (self.six_axis_period_s if command_mode == "passthrough"
                             else 1./follow_hz)
        self.waypoint_feedback_enabled = waypoint_feedback
        self.parallel_state_query = state_observer is not None
        if self.parallel_state_query and not (self.six_axis and waypoint_feedback and command_mode == "follow"):
            raise ValueError("Independent state query requires six-axis Follow with waypoint feedback")
        self._engagement_barrier = None
        self.joint_ranges_rad = None
        if self.six_axis:
            bounds = tuple(tuple(float(v) for v in pair) for pair in joint_ranges_rad)
            if (len(bounds) != 6 or any(len(b) != 2 or not all(math.isfinite(v) for v in b)
                    or b[0] >= b[1] or not b[0] <= q <= b[1] for b,q in zip(bounds,self.anchor))):
                raise ValueError("Six valid controller joint ranges containing the measured anchor required")
            self.joint_ranges_rad = bounds
        # Separate from serial request cache: reading a newer push must not let
        # an older snapshot overwrite it or manufacture a new robot-state time.
        self._push_cache = FeedbackCache(clock)
        self._last_push_sequence = 0
        self._feedback_channels = {}
        # Finite opt-in commissioning trace; no disk I/O on the SDK thread.
        self._trace = deque(maxlen=100000 if follow_hz == 200 or self.parallel_state_query else 20000) if diagnostics else None
        self._trace_count = 0
        self._stream_events = None
        self._feedback_error_log_s = None
        self._send_samples = deque(maxlen=2000)
        self._send_metrics_s = None
        self._send_metrics = {}
        self.j6_range_rad = tuple(float(v) for v in j6_range_rad)
        if (len(self.j6_range_rad) != 2 or not all(math.isfinite(v) for v in self.j6_range_rad)
                or self.j6_range_rad[0] >= self.j6_range_rad[1]
                or not self.j6_range_rad[0] <= self.anchor[5] <= self.j6_range_rad[1]):
            raise ValueError("Explicit valid controller J6 range containing the measured anchor required")
        # Match the native offset conversion at an exact controller boundary.
        self.offset_range_deg = tuple((v-self.anchor[5])/math.radians(1.) for v in self.j6_range_rad)
        self.cache = FeedbackCache(clock)
        self._lock = threading.RLock()
        self._quit = threading.Event()
        self._watch_quit = threading.Event()
        self._wake = threading.Event()
        self._thread = threading.Thread(target=self._run, name="aubo-follow-sdk", daemon=True)
        self._watch = threading.Thread(target=self._watchdog, name="aubo-follow-watchdog", daemon=True)
        self._wanted = None
        self._input_s = None
        self._active_session = None
        self._active_source = None
        self._may_move = False
        self._stop_needed = False
        self._fatal = False
        self._unknown = False
        self._inflight = None
        self._last_send_s = None
        self._last_sent_offset = None
        self._last_sent_q = None
        self._home_velocity = (0.,)*6
        self._last_observed_s = None
        self._robot_state = None
        self.recorder = None  # Optional bounded handoff; never a filesystem writer.
        self._status = dict(state="idle", fault=None, submitted=0, last_attempt=None,
            last_stop=None, last_snapshot=None, last_observation_attempt=None,
            feedback_ready=False, stop_confirmed=False,
            stop_confirmed_monotonic_s=None, stop_requires_post_confirm_release=True,
            first_stop_trigger=None, first_fatal=None,
            independent_physical_stop=False, serial_sdk_calls=True, sdk_connections=1,
            state_query_mode="serial", state_query_mode_label="串行查询（单 SDK 连接）",
            offset_range_deg=list(self.offset_range_deg),
            joint_range_deg=[math.degrees(v) for v in self.j6_range_rad],
            range_source="controller_joint_ranges", speed_deg_s=None if self.six_axis else self.speed_deg_s,
            sdk_client_speed_deg_s=None if self.six_axis else self.speed_deg_s,
            sdk_profile_policy="sdk_unmodified" if self.six_axis else "legacy_j6_explicit",
            sdk_profile_writes=not self.six_axis,
            sdk_profile_readback=sdk_profile_readback,
            target_policy="direct_ik" if self.six_axis else "j6_rate_limited",
            host_target_rate_limit_deg_s=None if self.six_axis else self.speed_deg_s,
            client_acceleration_deg_s2=None if self.six_axis else self.client_acceleration_deg_s2,
            following_error_limit_deg=self.following_error_limit_deg,
            requested_frequency_hz=1./self.period_s, send_max_gap_s=self.send_max_gap_s,
            recording=False, diagnostics_enabled=diagnostics,
            execution_scope="six_axis" if self.six_axis else "j6_only",
            joint_ranges_deg=None if not self.six_axis else [[math.degrees(v) for v in b] for b in self.joint_ranges_rad])
        self._status.update(stop_completion_budget_s=self.stop_completion_budget_s,
                            stop_feedback_budget_s=self.stop_feedback_budget_s,
                            sdk_budget_s=self.sdk_budget_s, feedback_max_age_s=self.feedback_max_age_s)
        self._status.update(state_query_policy="separate_active_push_cadence",
            active_state_query_period_s=(self.active_push_query_period_s
                if self.six_axis and self.waypoint_feedback_enabled else self.idle_query_period_s),
            idle_state_query_period_s=self.idle_query_period_s)
        self._status.update(command_mode=command_mode,
            sdk_profile_controls_passthrough=False if command_mode == "passthrough" else None)
        self.state_query = (StateQueryWorker(state_observer,
            on_failure=lambda reason:self.revoke(reason, fatal=True),
            is_active=lambda:self.status()["state"] == "active", diagnose=self._diagnose, clock=clock,
            active_period_s=self.active_push_query_period_s, idle_period_s=self.idle_query_period_s,
            budget_s=self.sdk_budget_s) if self.parallel_state_query else None)
        if self.parallel_state_query:
            self._status.update(state_query_policy="independent_sdk_connection",
                state_query_mode="parallel", state_query_mode_label="并行查询（双 SDK 连接）",
                state_query_schedule="start_to_start_single_inflight_no_catchup",
                serial_sdk_calls=False, motion_sdk_calls_serial=True, sdk_connections=2)

    def start(self):
        if self.state_query:
            self.state_query.start()
        self._thread.start()
        self._watch.start()

    def _diagnose(self, event, **values):
        with self._lock:
            if self._trace is not None:
                self._trace_count += 1
                row = dict(values, event=event, host_monotonic_s=self.clock(),
                           event_sequence=self._trace_count)
                self._trace.append(row)
                if self._stream_events is not None:
                    self._stream_events.append(row)

    def enable_diagnostic_stream(self, capacity=8192):
        """Attach one bounded consumer before starting workers; no disk I/O here."""
        with self._lock:
            if self._trace is None or self._stream_events is not None:
                raise RuntimeError("Diagnostic stream requires an enabled, unattached trace")
            # The writer owns the full log. Retaining another 100k nested
            # Python records makes cyclic-GC scans stall the control thread.
            self._trace = deque(self._trace, maxlen=self.streaming_history_capacity)
            self._stream_events = deque(maxlen=capacity)

    def drain_diagnostic_stream(self, limit=256):
        with self._lock:
            if self._stream_events is None:
                return []
            return [self._stream_events.popleft()
                    for _ in range(min(limit, len(self._stream_events)))]

    def note_feedback_failure(self, exc):
        """Keep the exact error even when the finite trace is disabled."""
        with self._lock:
            now = self.clock()
            previous = self._status.get("last_feedback_error")
            detail = dict(host_monotonic_s=now, error_type=type(exc).__name__,
                          error=str(exc), read=self._status.get("last_feedback_read"))
            self._status["last_feedback_error"] = detail
            if (previous is None or previous["error"] != detail["error"] or
                    self._feedback_error_log_s is None or now-self._feedback_error_log_s >= .1):
                self._feedback_error_log_s = now
                self._diagnose("push_refresh_failed", **detail)

    def diagnostic_report(self):
        with self._lock:
            events = list(self._trace) if self._trace is not None else []
            return dict(enabled=self._trace is not None, events=events,
                        total_events=self._trace_count, dropped_events=self._trace_count-len(events),
                        controller_sample_time=None,
                        notes="Host query/reception times; command/feedback samples are not simultaneous",
                        final_status=self.status())

    def note_input(self, stamp):
        with self._lock:
            self._input_s = stamp

    def project_j6(self, angle):
        """Clamp the author's absolute angle to the current controller range."""
        return min(self.j6_range_rad[1], max(self.j6_range_rad[0], angle))

    def project_target(self, q):
        q = joints(q)
        if not self.six_axis:
            return self.anchor[:5]+(self.project_j6(q[5]),)
        return tuple(min(b[1],max(b[0],v)) for b,v in zip(self.joint_ranges_rad,q))

    def _check_measured(self, q):
        if self.six_axis:
            if any(not lo-math.radians(.1) <= v <= hi+math.radians(.1)
                   for v,(lo,hi) in zip(q,self.joint_ranges_rad)):
                raise RuntimeError("measured_motion_outside_controller_ranges")
        elif any(abs(q[i]-self.anchor[i]) > math.radians(.1) for i in range(5)) or not (
                self.j6_range_rad[0]-math.radians(.1) <= q[5] <= self.j6_range_rad[1]+math.radians(.1)):
            raise RuntimeError("measured_motion_outside_j6_envelope")

    def measured_feedback(self):
        """Nonblocking push memory read; does not refresh SDK robot-state data."""
        if not self.waypoint_feedback_enabled:
            return self.cache.require_recent(self.feedback_max_age_s)
        read_started = self.clock()
        try:
            if callable(getattr(type(self.client), "waypoint_feedback_since", None)):
                raw = self.client.waypoint_feedback_since(self._last_push_sequence)
            else:
                raw = self.client.waypoint_feedback()
        finally:
            read_finished = self.clock()
            with self._lock:
                self._status["last_feedback_read"] = dict(
                    started_monotonic_s=read_started, finished_monotonic_s=read_finished,
                    duration_s=read_finished-read_started, sequence=None,
                    callback_received_monotonic_s=None)
        with self._lock:
            self._feedback_channels = raw.get("channels", {}) if isinstance(raw, dict) else {}
        samples = raw["samples"]
        if not samples:
            self._push_cache.invalidate("waypoint_feedback_missing")
            raise RuntimeError("waypoint_feedback_missing")
        frame = samples[-1]
        sequence = frame["sequence"]
        with self._lock:
            self._status["last_feedback_read"] = dict(
                self._status["last_feedback_read"], sequence=sequence,
                callback_received_monotonic_s=frame["host_received_monotonic_s"])
        if type(sequence) is not int or sequence <= 0:
            raise RuntimeError("invalid_push_sequence")
        if sequence < self._last_push_sequence:
            raise RuntimeError("push_sequence_regressed")
        if sequence > self._last_push_sequence:
            previous_sequence = self._last_push_sequence
            previous_sample = self._push_cache.latest()
            q = joints(frame["q_rad"])
            self._check_measured(q)
            sample = self._push_cache.update(q, source="sdk_waypoint_push",
                received_monotonic_s=frame["host_received_monotonic_s"])
            self._last_push_sequence = sequence
            self._diagnose("push_observed", sequence=sequence,
                received_monotonic_s=sample.received_monotonic_s, q_rad=list(q),
                samples=[s for s in samples if s["sequence"] > previous_sequence] if self._trace is not None else None,
                previous_sequence=previous_sequence,
                previous_received_monotonic_s=(previous_sample.received_monotonic_s
                    if previous_sample else None),
                read_started_monotonic_s=read_started,
                read_finished_monotonic_s=read_finished,
                read_duration_s=read_finished-read_started,
                sdk_inflight=self.status()["sdk_inflight"])
        return self._push_cache.require_recent(self.feedback_max_age_s)

    def offer(self, candidate, *, control_source='teleop'):
        # Candidate is the author's output; the backend only bounds submission.
        try:
            q = joints(candidate.q_rad)
            stamp = float(candidate.generated_monotonic_s)
            valid = (control_source in ('teleop', 'home_return') and
                     (control_source != 'home_return' or (self.six_axis and self.command_mode == 'follow')) and
                     math.isfinite(stamp) and 0 <= self.clock() - stamp <= self.candidate_max_age_s)
        except (ValueError, TypeError, OverflowError):
            valid = False
        if not valid:
            self.revoke("invalid_or_expired_candidate", fatal=True)
            return False
        with self._lock:
            if self._quit.is_set() or self._fatal or self._unknown or self._stop_needed:
                return False
            self._wanted = (q, stamp, candidate.session, control_source)
        self._wake.set()
        return True

    def revoke(self, reason, *, fatal=False):
        # Native revoke is atomic, has no SDK call and takes no I/O mutex.
        self.client.revoke()
        with self._lock:
            # Preserve the initiating host event before secondary feedback or
            # transport failures overwrite the current reason.
            now = self.clock()
            event = dict(reason=str(reason), host_monotonic_s=now,
                prior_state=self._status["state"],
                feedback_age_s=None if self._last_observed_s is None else now-self._last_observed_s,
                input_age_s=None if self._input_s is None else now-self._input_s,
                candidate_age_s=None if self._wanted is None else now-self._wanted[1],
                last_send_age_s=None if self._last_send_s is None else now-self._last_send_s,
                state_observer=self.state_query.status() if self.state_query else None,
                feedback_read=self._status.get("last_feedback_read"),
                feedback_error=self._status.get("last_feedback_error"),
                sdk_inflight=dict(self._inflight) if self._inflight else None)
            observation = self._status["last_observation_attempt"]
            if observation is not None:
                event["reception_age_s"] = {key:now-observation["snapshot"][key]["host_received_monotonic_s"]
                    for key in ("joints", "state")}
            if fatal and self._status["first_fatal"] is None:
                self._status["first_fatal"] = event
            # Once any non-clutch fault occurs, later released frames cannot
            # turn recovery back into a normal clutch stop.
            # Also retain faults arriving just after confirmation, before the
            # runtime has consumed the stop gate. Being at rest does not make
            # a disconnected/invalid input safe to recover as a normal release.
            if fatal or reason != "grip_released":
                self._status["stop_requires_post_confirm_release"] = True
            if self._may_move and not self._stop_needed and self._status["first_stop_trigger"] is None:
                self._status["first_stop_trigger"] = event
                self._diagnose("stop_requested", trigger=event,
                    last_sent_q_rad=None if self._last_sent_q is None else list(self._last_sent_q))
                if self.recorder:
                    self.recorder.record('hardware_event', reason=str(reason), trigger=event)
                    if reason in ('grip_released', 'input_timeout'):
                        self.recorder.pause_episode(str(reason), stamp=now)
            self._wanted = None
            self._engagement_barrier = None
            self._fatal |= fatal
            if fatal:
                self._status["feedback_ready"] = False
            self._status["fault"] = (self._status["first_fatal"]["reason"]
                if self._status["first_fatal"] else reason)
            if self._may_move and not self._unknown and not self._stop_needed:
                self._stop_needed = True
                self._status["state"] = "stop_requested"
            elif fatal and not self._may_move and not self._unknown:
                self._status["state"] = "fault"
        self._wake.set()

    def status(self):
        with self._lock:
            now = self.clock()
            if self._send_metrics_s is None or now-self._send_metrics_s >= 1.:
                samples = [s for s in self._send_samples if now-s["started_s"] <= 10.]
                gaps = [b["started_s"]-a["started_s"] for a,b in zip(samples,samples[1:])
                        if a["session"] == b["session"] and b["started_s"] > a["started_s"]]
                def distribution(values):
                    ordered = sorted(values)
                    return (dict(count=len(ordered), mean_ms=sum(ordered)/len(ordered)*1000,
                        p50_ms=ordered[(len(ordered)-1)//2]*1000,
                        p95_ms=ordered[math.ceil(.95*len(ordered))-1]*1000,
                        p99_ms=ordered[math.ceil(.99*len(ordered))-1]*1000,
                        max_ms=ordered[-1]*1000) if ordered else dict(count=0))
                self._send_metrics = dict(window_s=10., updated_monotonic_s=now,
                    scope="completed_sdk_calls; intervals_within_same_engagement; not_controller_servo_rate",
                    calls=len(samples), accepted=sum(s["accepted"] for s in samples),
                    actual_within_engagement_hz=len(gaps)/sum(gaps) if gaps else None,
                    send_interval=distribution(gaps), sdk_call=distribution([s["duration_s"] for s in samples]),
                    intervals_over_20ms=sum(g>.020 for g in gaps),
                    intervals_over_requested_period=sum(g>self.period_s for g in gaps),
                    intervals_over_twice_requested_period=sum(g>2*self.period_s for g in gaps),
                    intervals_over_40ms=sum(g>.040 for g in gaps),
                    intervals_over_100ms=sum(g>.100 for g in gaps))
                self._send_metrics_s = now
            return dict(self._status, fatal=self._fatal, stop_result_unknown=self._unknown,
                input_timeout_s=self.input_max_age_s,
                state_observer=self.state_query.status() if self.state_query else None,
                send_metrics=self._send_metrics,
                sdk_inflight=dict(self._inflight) if self._inflight else None,
                sdk_inflight_age_s=now-self._inflight["started_s"] if self._inflight else None,
                push_sequence=self._last_push_sequence,
                feedback_channels=dict(self._feedback_channels),
                push_feedback_age_s=now-self._push_cache.latest().received_monotonic_s if self._push_cache.latest() else None,
                feedback_age_s=now-self.cache.latest().received_monotonic_s if self.cache.latest() else None)

    def _call(self, name, call, *, budget_s=None):
        budget = budget_s if budget_s is not None else (
            self.stop_completion_budget_s if name == "slow_stop" else self.sdk_budget_s)
        with self._lock:
            self._inflight = dict(operation=name, started_s=self.clock(), budget_s=budget)
            self._diagnose("sdk_call_started", connection="A", **self._inflight)
        try:
            result = call()
        finally:
            with self._lock:
                started = self._inflight["started_s"]
                self._diagnose("sdk_call_finished", connection="A", operation=name,
                               started_s=started, duration_s=self.clock()-started)
                self._inflight = None
        if self.clock() - started > budget:
            reason = {"slow_stop": "stop_completion_over_budget",
                      "stop_snapshot": "stop_confirmation_timeout"}.get(name, "sdk_call_over_budget")
            self.revoke(reason, fatal=True)
        return result

    def _observe(self, *, stop_deadline=None):
        # After slow-stop succeeds, use the remaining confirmation window for
        # read-only I/O. Motion queries retain their separate SDK-call budget.
        remaining = None if stop_deadline is None else stop_deadline-self.clock()
        if remaining is not None and remaining <= 0:
            raise RuntimeError("stop_confirmation_timeout")
        snap = self._call("snapshot" if stop_deadline is None else "stop_snapshot",
                          lambda: checked_snapshot(self.client), budget_s=remaining)
        now = self.clock()
        ages = {key:now-float(snap[key]["host_received_monotonic_s"])
                for key in ("joints", "state")}
        with self._lock:
            self._status["last_observation_attempt"] = dict(snapshot=snap,
                checked_monotonic_s=now, reception_age_s=ages)
        if self.parallel_state_query and snap.get("observation_pending"):
            raise ObservationPending("Parallel observation pending: "
                f"joint_age_s={ages['joints']:.6f}, state_age_s={ages['state']:.6f}", snap)
        # The legacy snapshot validator accepts only motion-permitted states.
        # During parallel stop confirmation, Paused/Resumed are valid observed
        # states to keep polling, but can never confirm stopped or authorize send.
        if (self.parallel_state_query and stop_deadline is not None and
                snap.get("validation_error", {}).get("message") == "Unexpected controller state" and
                record_ok(snap["joints"]) and record_ok(snap["state"]) and
                type(snap["state"]["data"]) is int and snap["state"]["data"] in (2, 3)):
            snap.pop("validation_error")
        with self._lock:
            self._status["last_snapshot"] = snap
        # checked_snapshot's legacy 200 ms diagnostic belongs to the earlier
        # micro-trial entry points. Preserve it, but enforce this worker's
        # explicitly configured call budget here.
        snap["sdk_observation_budget_s"] = self.sdk_budget_s
        snap["sdk_observation_budget_exceeded"] = snap["host_snapshot_elapsed_s"] > self.sdk_budget_s
        require_snapshot(snap, enforce_budget=False)
        if stop_deadline is None and snap["sdk_observation_budget_exceeded"]:
            raise RuntimeError("Snapshot exceeded experimental 500 ms observation budget")
        if stop_deadline is not None:
            now = self.clock()
            if now >= stop_deadline:
                raise RuntimeError("stop_confirmation_timeout")
        # A longer call allowance never refreshes the reception timestamp of
        # old data, including during stop confirmation.
        now = self.clock()
        if any(not 0 <= now-float(snap[key]["host_received_monotonic_s"]) <= self.feedback_max_age_s
               for key in ("joints", "state")):
            if self.parallel_state_query:
                raise ObservationPending("Parallel feedback expired during snapshot processing", snap)
            raise RuntimeError("stop_feedback_reception_expired" if stop_deadline is not None
                               else "feedback_reception_expired")
        q = joints(snap["joints"]["data"])
        self._check_measured(q)
        self.cache.update(q, source=snap["joints"].get("source", "aubo_follow_snapshot"),
            received_monotonic_s=snap["joints"]["host_received_monotonic_s"])
        self._last_observed_s = self.clock()
        self._robot_state = snap["state"]["data"]
        with self._lock:
            self._status["feedback_ready"] = True
        self._diagnose("feedback", snapshot=snap, phase=self._status["state"])
        return q, snap["state"]["data"]

    def _handle_pending_observation(self, pending):
        """Freshness loss revokes motion without invalidating a healthy channel.

        A fresh non-stopped state still needs stop handling even if joints aged
        out. Never retain an intent across this gap or use it as stopped proof.
        """
        with self._lock:
            first_wait = self._status["feedback_ready"]
            self._status["feedback_ready"] = False
            if self._stop_needed:
                return  # A concurrent release already owns the stop deadline.
            if self._may_move:
                ages = [self.clock()-pending.snapshot[key]["host_received_monotonic_s"]
                        for key in ("joints", "state")]
                if not all(math.isfinite(age) and age >= 0 for age in ages):
                    raise pending  # Invalid/future samples remain a fault.
                self.revoke("feedback_reception_expired")
                return  # Freshness stops motion; it does not kill observation.
            state = pending.snapshot["state"]
            fresh_state = (record_ok(state) and type(state["data"]) is int and
                state["data"] in (0, 1, 2, 3) and
                0 <= self.clock()-state["host_received_monotonic_s"] <= self.feedback_max_age_s)
            if fresh_state and state["data"] != 0:
                self._may_move = True
                self.revoke("unexpected_motion_while_idle", fatal=True)
                return
            # Leave the cache's real reception timestamp intact. Invalidating
            # it with host 'now' would reject a recovering push received just
            # before that invalidation as out of order. Readiness and native
            # permission prevent this old sample from authorizing any motion.
            self.revoke("parallel_observation_pending")
            if first_wait:
                self._diagnose("idle_observation_wait",
                    observation=self._status["last_observation_attempt"])

    def watchdog_tick(self):
        with self._lock:
            inflight = self._inflight
            active = self._may_move and not self._stop_needed and not self._unknown
            wanted = self._wanted
            input_s = self._input_s
        observation = self.state_query.status() if self.state_query else None
        # A B result can be published while status() is read. Validate the
        # copied records against a later clock sample, never an earlier one.
        now = self.clock()
        if self.state_query:
            query = observation["sdk_inflight"]
            if query and now-query["started_s"] > query["budget_s"]:
                self.state_query.fail("query_over_budget")
            raw = observation["last_result"]
            if active:
                age = None if raw is None else now-raw["host_received_monotonic_s"]
                if age is None or not math.isfinite(age) or age < 0:
                    self.state_query.fail("state_feedback_invalid")
                elif age > self.feedback_max_age_s:
                    with self._lock:
                        self._status["feedback_ready"] = False
                    self.revoke("state_reception_expired")
        if inflight and now-inflight["started_s"] > inflight["budget_s"]:
            reason = {"slow_stop": "stop_completion_over_budget_operator_stop_required",
                      "stop_snapshot": "stop_confirmation_timeout_operator_stop_required"}.get(
                          inflight["operation"], "sdk_call_over_budget_operator_stop_required")
            self.revoke(reason, fatal=True)
            with self._lock:
                self._status["operator_stop_required"] = True
        elif active and (input_s is None or now-input_s > self.input_max_age_s):
            self.revoke("input_timeout")
        elif active and wanted and now-wanted[1] > self.candidate_max_age_s:
            self.revoke("candidate_timeout")

    def _watchdog(self):
        while not self._watch_quit.wait(.01):
            self.watchdog_tick()

    def _stop_and_confirm(self):
        with self._lock:
            self._status.update(state="stopping", stop_confirmed=False, stop_confirmed_monotonic_s=None)
        self._diagnose("slow_stop_call_started")
        try:
            result = self._call("slow_stop", self.client.stop)
            with self._lock:
                self._status["last_stop"] = result
            self._diagnose("stop_returned", result=result)
            if not record_ok(result):
                raise RuntimeError("stop_request_failed")
            # A late successful return must still be checked against feedback.
            # Retain the fatal latch; confirmation cannot re-authorize motion.
            if result["duration_s"] > self.stop_completion_budget_s:
                self.revoke("stop_completion_over_budget", fatal=True)
            deadline = self.clock()+self.stop_feedback_budget_s
            # Results of queries started before stop completion cannot confirm it.
            barrier = self.clock() if self.state_query else None
            if self.state_query:
                self.state_query.request_refresh()
            quiet_q = quiet_s = None
            while self.clock() < deadline:
                try:
                    q, state = self._observe(stop_deadline=deadline)
                except ObservationPending:
                    # The old sample is not evidence. Keep the observation
                    # worker alive and await a new one within the same deadline.
                    quiet_q = quiet_s = None
                    with self._lock:
                        self._status["feedback_ready"] = False
                    time.sleep(.005)
                    continue
                now = self.clock()
                joint_stamp = self._status["last_snapshot"]["joints"]["host_received_monotonic_s"]
                if self.state_query and (self._status["last_snapshot"]["state"]["host_started_monotonic_s"] < barrier
                                         or joint_stamp < barrier):
                    time.sleep(.005)
                    continue
                quiet_now = joint_stamp if self.state_query else now
                if state == 0:
                    if quiet_q is not None and max(abs(a-b) for a,b in zip(q,quiet_q)) <= math.radians(.02):
                        if quiet_now-quiet_s >= .1:
                            with self._lock:
                                self._may_move = self._stop_needed = False
                                self._active_session = None
                                self._engagement_barrier = None
                                self._status.update(state="fault" if self._fatal else "idle", stop_confirmed=True,
                                                    stop_confirmed_monotonic_s=now)
                            self._diagnose("stop_confirmed", q_rad=list(q), robot_state=state,
                                quiet_since_monotonic_s=quiet_s,
                                feedback=self._status["last_snapshot"])
                            return
                    else:
                        quiet_q, quiet_s = q, quiet_now
                else:
                    quiet_q = quiet_s = None
                time.sleep(.05)
            raise RuntimeError("stop_confirmation_timeout")
        except Exception as exc:
            self.cache.invalidate("stop_confirmation_failed")
            self._diagnose("stop_confirmation_failed", error=dict(type=type(exc).__name__, message=str(exc)))
            with self._lock:
                self._fatal = self._unknown = True
                self._stop_needed = False  # No automatic retry after an unknown stop outcome.
                self._status.update(state="stop_unknown", fault=str(exc), operator_stop_required=True,
                                    stop_confirmation_error=dict(type=type(exc).__name__,message=str(exc)))

    def _cycle(self):
        with self._lock:
            stop_needed, unknown = self._stop_needed, self._unknown
            wanted, active_session = self._wanted, self._active_session
        if unknown:
            return
        if stop_needed:
            self._stop_and_confirm()
            return
        if self._fatal and not self._may_move:
            return
        # Only continuous six-axis following with independent joint push uses
        # the separate state-query cadence. Startup/re-engagement always query;
        # idle, legacy J6 and stop confirmation retain their previous behavior.
        # Schedule from the state's reception time, not the snapshot completion
        # bookkeeping time. This does not manufacture a newer robot-state stamp.
        needs_observation = True
        if not self.parallel_state_query:
            active_push = (self.six_axis and self.waypoint_feedback_enabled
                           and active_session is not None and wanted is not None)
            query_period = self.active_push_query_period_s if active_push else self.idle_query_period_s
            state_stamp = (self._status["last_snapshot"]["state"]["host_received_monotonic_s"]
                           if self._status["last_snapshot"] is not None else None)
            joint_sample = self.cache.latest()
            joint_age = None if joint_sample is None else self.clock()-joint_sample.received_monotonic_s
            # Only the serial path schedules SDK snapshots here. Parallel
            # scheduling belongs solely to B; A always reads the latest cache.
            needs_observation = (state_stamp is None or self.clock()-state_stamp >= query_period
                or joint_sample is None or not joint_sample.valid
                or not 0 <= joint_age < self.feedback_max_age_s
                or (wanted is not None and active_session is None))
        if needs_observation:
            try:
                q, state = self._observe()
            except ObservationPending as pending:
                self._handle_pending_observation(pending)
                return
        else:
            q, state = self.cache.require_recent(.2).q_rad, self._robot_state
        with self._lock:
            wanted, fatal = self._wanted, self._fatal
            input_s, active_session = self._input_s, self._active_session
        if fatal or wanted is None:
            if not self._may_move and state != 0:
                with self._lock:
                    self._may_move = True
                self.revoke("unexpected_motion_while_idle", fatal=True)
            return
        now = self.clock()
        if (now-wanted[1] > self.candidate_max_age_s or input_s is None or
                now-input_s > self.input_max_age_s):
            self.revoke("intent_expired")
            return
        if active_session is not None and active_session != wanted[2]:
            self.revoke("session_changed")
            return
        if active_session is not None and self._active_source != wanted[3]:
            self.revoke('control_source_changed')
            return
        if active_session is None:
            if self.state_query:
                barrier = self._engagement_barrier
                if barrier is None or barrier[0] != wanted[2]:
                    self._engagement_barrier = (wanted[2], self.clock())
                    self.state_query.request_refresh()
                    return
                if self._status["last_snapshot"]["state"]["host_started_monotonic_s"] < barrier[1]:
                    return
            if state != 0:
                self.revoke("engagement_requires_stopped_feedback", fatal=True)
                return
            token = self.client.revision()
            with self._lock:
                # Mark BEFORE arm so a concurrent input fault requests stop.
                self._may_move = True
                self._status.update(state="arming", stop_confirmed=False, first_stop_trigger=None,
                    stop_confirmed_monotonic_s=None, stop_requires_post_confirm_release=False)
            if not self._call("arm", lambda:self.client.arm(token)):
                self.revoke("native_arm_refused")
                return
            if self.command_mode == "passthrough":
                # Enter only on engagement, never during prepare/inspection.
                # _may_move was set before entry: even a late/failed or revoked
                # entry is followed by exit+stop, rather than a second enter.
                with self._lock:
                    if self._stop_needed or self._fatal or self._wanted is None:
                        return
                entry = self._call("enter_passthrough", lambda:self.client.enter_passthrough(token))
                self._diagnose("passthrough_enter_returned", result=entry)
                with self._lock:
                    self._status["last_enter_passthrough"] = entry
                if not record_ok(entry) or entry["duration_s"] > self.sdk_budget_s:
                    self.revoke("passthrough_entry_failed_or_late", fatal=True)
                    return
            with self._lock:
                if self._stop_needed or self._fatal or self._wanted is None:
                    return
                self._active_session = wanted[2]
                self._active_source = wanted[3]
                self._token = token
                self._last_sent_offset = (q[5]-self.anchor[5])/math.radians(1.)
                self._last_sent_q = q
                self._home_velocity = (0.,)*6
                self._last_send_s = self.clock()
                self._status.update(state="active", fault=None)
            return
        if now-self._last_send_s < self.period_s:
            return
        if now-self._last_send_s > self.send_max_gap_s:
            # A missed cadence is recoverable only after confirmed stop and a
            # new release/press. SDK errors/timeouts remain fatal separately.
            self.revoke("send_period_over_budget")
            return
        projected = self.project_target(wanted[0])
        desired = (projected[5]-self.anchor[5])/math.radians(1.)
        if self.six_axis:
            # Forward latest IK positions without host intermediate targets.
            # Passthrough has no assumed FollowMode profile interpolation.
            target = projected
            offset = desired
            if wanted[3] == 'home_return':
                from .home import bounded_home_step
                # dt 放宽到 3×周期（2026-11-01）：双峰应答下实际间隔 >周期时，
                # 原封顶让目标斜坡只跑名义速度的 ~40%，跟踪差距稳定增长触发
                # 3°→8° 容差。上限仍受 bounded_home_step 内部 0.12s 二次封顶。
                target, home_velocity = bounded_home_step(self._last_sent_q, projected,
                    self._home_velocity, min(now-self._last_send_s, 3*self.period_s))
                offset = (target[5]-self.anchor[5])/math.radians(1.)
        else:
            # Preserve the separate legacy J6 commissioning path.
            step = self.speed_deg_s * min(self.period_s, now-self._last_send_s) * .98
            offset = min(self._last_sent_offset+step, max(self._last_sent_offset-step,desired))
            target = self.anchor[:5]+(self.anchor[5]+math.radians(offset),)
        attempt = dict(generated_q_rad=list(wanted[0]), requested_q_rad=list(target), generated_monotonic_s=wanted[1],
            control_source=wanted[3],
            session=wanted[2], requested_offset_deg=offset, outcome="in_flight_result_unknown",
            host_attempt_monotonic_s=now, previous_offset_deg=self._last_sent_offset,
            feedback_snapshot=self._status["last_snapshot"])
        with self._lock:
            self._status["last_attempt"] = attempt
        if self.recorder:
            attempt['recording_episode_id'] = self.recorder.status()['active_episode']
        self._diagnose("send_attempt", attempt=attempt)
        if self.recorder:
            self.recorder.record('sdk_command', phase='attempt', attempt=attempt)
        try:
            result = self._call("follow", lambda:(self.client.send_joints(target,self._token)
                if self.six_axis else self.client.send_offset(offset,self._token)))
        except Exception as exc:
            with self._lock:
                self._status["last_attempt"] = dict(attempt,
                    outcome="not_submitted_permission_revoked" if str(exc)=="Send permission revoked" else "exception_result_unknown",
                    exception=dict(type=type(exc).__name__,message=str(exc)))
            self._diagnose("send_exception", attempt=self._status["last_attempt"])
            if self.recorder:
                self.recorder.record('sdk_command', phase='exception', attempt=self._status['last_attempt'])
            raise
        completed = dict(attempt, sdk_result=result,
                         outcome=("not_submitted_local_guard" if result.get("sdk_invoked") is False
                                  else "sdk_accepted" if record_ok(result) else "sdk_rejected"))
        with self._lock:
            self._status["last_attempt"] = completed
            self._status["submitted"] += int(result.get("sdk_invoked") is not False)
            if result.get("sdk_invoked") is not False:
                self._send_samples.append(dict(started_s=result["host_started_monotonic_s"],
                    duration_s=result["duration_s"],session=attempt["session"],accepted=record_ok(result)))
        self._diagnose("send_returned", attempt=completed)
        if self.recorder:
            self.recorder.record('sdk_command', phase='returned', attempt=completed)
        if result.get("sdk_invoked") is False:
            pre = result.get("pre_send", {})
            ages = [pre.get("checked_monotonic_s", float("nan"))-pre.get(key, float("nan"))
                    for key in ("joints_received_monotonic_s", "state_received_monotonic_s")]
            if (self.parallel_state_query and result.get("local_refusal_reason") ==
                    "Six-axis feedback/state age or send interval violated" and
                    all(math.isfinite(age) and age >= 0 for age in ages) and
                    any(age > self.feedback_max_age_s for age in ages)):
                self.revoke("feedback_reception_expired")
                return
            self.revoke("native_send_refused: "+result.get("local_refusal_reason", "unknown"), fatal=True)
            return
        if not record_ok(result) or result["duration_s"] > self.sdk_budget_s:
            self.revoke("follow_failed_or_late", fatal=True)
            return
        self._last_send_s = result["host_started_monotonic_s"]
        self._last_sent_offset = offset
        self._last_sent_q = target
        if wanted[3] == 'home_return':
            self._home_velocity = home_velocity
        # Never re-authorize after a call returned following revoke.

    def _wait_timeout_s(self):
        """Wake by the next send deadline without adding a fixed 5 ms delay."""
        with self._lock:
            if self._wanted is not None and self._active_session is not None and self._last_send_s is not None:
                return max(0., min(.005, self._last_send_s+self.period_s-self.clock()))
        return .005

    def _run(self):
        try:
            while not self._quit.is_set():
                try:
                    self._cycle()
                except Exception as exc:
                    with self._lock:
                        cancelled_before_sdk = self._stop_needed and str(exc) == "Send permission revoked"
                    if not cancelled_before_sdk:
                        self.cache.invalidate("follow_worker_failed")
                        self.revoke(type(exc).__name__+": "+str(exc), fatal=True)
                self._wake.wait(self._wait_timeout_s())
                self._wake.clear()
        finally:
            if self._may_move and not self._unknown:
                self._stop_and_confirm()

    def close(self):
        self.revoke("closing")
        self._quit.set()
        self._wake.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=6.)
        query_error = None
        if self.state_query:
            try:
                self.state_query.close()
            except RuntimeError as exc:
                query_error = exc
        self._watch_quit.set()
        if self._watch.ident is not None:
            self._watch.join(timeout=1.)
        if self._thread.is_alive():
            raise RuntimeError("SDK still in flight; operator stop required; no concurrent logout")
        if query_error:
            raise query_error
        if self._unknown:
            raise RuntimeError("Physical stop result unknown; operator stop required")
