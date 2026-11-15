"""Supervised worker; serial motion owner and optional independent state owner.

Independent watchdog revokes local permission while SDK is blocked. It cannot
interrupt the request or deliver an independent physical stop. Safety failures
retain the stop/release gate. A stationary cold-priming deadline alone can recover after confirmed stop and a fresh released input.
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
from .trajectory import JointTrajectory


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
    passthrough_alpha = None       # Replaced by bounded actual-dt trajectory
    priming_max_s = 2.5
    cold_cooldown_s = .5
    holding_idle_s = 20.
    braking_max_s = 3.
    holding_tolerance_rad = math.radians(.1)
    braking_tracking_limit_rad = math.radians(3.)
    hot_state_max_age_s = .5
    hot_joint_max_age_s = .5
    slow_send_s = .25
    slow_send_count = 3
    priming_limit_s = .4
    priming_min_steps = 12
    priming_fast_count = 5
    priming_fast_s = .04
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
    hot_query_period_s = .2
    resume_state_max_age_s = .25
    resume_joint_max_age_s = .12
    active_push_query_period_s = .1  # Host scheduling choice, not a cabinet rating
    # Experimental completion allowance for the blocking slow-stop operation.
    # This is separate from command/query latency and is not a vendor guarantee.
    stop_completion_budget_s = 2.
    stop_feedback_budget_s = 2.

    def __init__(self, client, anchor, *, j6_range_rad, clock=time.monotonic, diagnostics=False,
                 joint_ranges_rad=None, waypoint_feedback=False, sdk_profile_readback=None,
                 command_mode="follow", follow_hz=20., state_observer=None,
                 priming_reanchor_required=False, fixed_tick_batch=False,batch_point_limit=4,
                 wall_clock_batch=False,refined_stream=False,motion_profile=None,buffer_points=None,underrun_points=None):
        self.client, self.anchor, self.clock = client, joints(anchor), clock
        self.six_axis = joint_ranges_rad is not None
        if command_mode not in ("follow", "passthrough") or (command_mode == "passthrough" and not self.six_axis):
            raise ValueError("Passthrough requires explicit six-axis scope")
        if fixed_tick_batch and (command_mode!='passthrough' or state_observer is None):
            raise ValueError('Fixed-tick batches require passthrough and independent feedback')
        if type(batch_point_limit) is not int or not 1<=batch_point_limit<=8:
            raise ValueError('Batch point limit must be 1..8')
        if wall_clock_batch and (not fixed_tick_batch or batch_point_limit<4):
            raise ValueError('Wall clock batches require fixed tick and a 4..8 point maximum')
        self.wall_clock_batch=bool(wall_clock_batch)
        if refined_stream and not wall_clock_batch:raise ValueError('Refined stream requires wall clock batches')
        self.refined_stream=bool(refined_stream)
        self._state_poll_paused=False
        self._underrun=None
        from .motion_profile import motion_profile as resolve_profile
        self.motion_profile=resolve_profile(motion_profile or 'conservative')
        self.buffer_points=(18 if motion_profile is not None else 12) if buffer_points is None else buffer_points
        if type(self.buffer_points) is not int or not 8<=self.buffer_points<=20:
            raise ValueError('Stream buffer must be 8..20 points')
        self.underrun_points=self.buffer_points if underrun_points is None else underrun_points
        if type(self.underrun_points) is not int or self.underrun_points!=self.buffer_points:
            raise ValueError('Underrun threshold must equal the stream cushion')
        if motion_profile is not None and not self.refined_stream:
            raise ValueError('Motion profiles require refined six-axis passthrough')
        if self.refined_stream:
            self.client.configure_refined_stream()
            if motion_profile is not None or buffer_points is not None:
                ack=self.client.configure_motion_limits(self.motion_profile.velocity_deg_s,
                    self.motion_profile.acceleration_deg_s2,self.motion_profile.jerk_deg_s3,self.buffer_points,self.underrun_points)
                if not math.isclose(ack['max_step_deg'],self.motion_profile.step_deg,abs_tol=1e-12):
                    raise RuntimeError('Native/host fixed tick step limits differ')
        # Reference-vs-callback distance includes buffered points and feedback
        # transport age. Preserve the conservative time-equivalent envelope;
        # final HOLDING position tolerance remains an independent 0.1 degree.
        self.braking_tracking_limit_rad=math.radians(3.*self.motion_profile.velocity_deg_s/10.)
        self._next_batch_s=None
        self._batch_credit=0
        self._batch_credit_received_s=0.
        self._batch_credit_diagnosis=None
        self.batch_point_limit=batch_point_limit
        self.fixed_tick_batch=bool(fixed_tick_batch)
        self.command_mode = command_mode
        if command_mode=="passthrough":self.active_push_query_period_s=self.hot_query_period_s
        if follow_hz not in (20., 25., 50., 200.) or (follow_hz != 20. and (not self.six_axis or command_mode != "follow")):
            raise ValueError("Follow rate requires six-axis follow and 20, 25, 50 or 200 Hz")
        if self.six_axis:
            self.period_s = (self.six_axis_period_s if command_mode == "passthrough"
                             else 1./follow_hz)
        self.waypoint_feedback_enabled = waypoint_feedback
        self.parallel_state_query = state_observer is not None
        if self.parallel_state_query and not (self.six_axis and waypoint_feedback):
            raise ValueError("Independent state query requires six-axis control with waypoint feedback")
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
        self._push_read_lock=threading.RLock()
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
        self._first_active_hold = False
        self._hold_q = None
        self._brake_pending = False
        self._hot_fast = 0
        self._slow_sends = 0
        self._hot_released_s = self._holding_since_s = None
        self._hot_quiet_q = self._hot_quiet_s = None
        self._cold_abort_anchor = None
        self._cold_enter_not_before = 0.
        self._priming_anchor = None
        self._priming_started_s = None
        self._priming_steps = self._priming_fast = 0
        self._priming_reanchor_ack = None
        self._priming_reanchor_required = priming_reanchor_required
        self._trajectory = JointTrajectory(self.anchor,self.joint_ranges_rad,profile=self.motion_profile.name) if command_mode=='passthrough' else None
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
            target_policy=("bounded_actual_dt" if command_mode == "passthrough" else "direct_ik") if self.six_axis else "j6_rate_limited",
            passthrough_alpha=self.passthrough_alpha if command_mode == "passthrough" else None,
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
        self._status.update(command_mode=command_mode,fixed_tick_batch=self.fixed_tick_batch,
            wall_clock_batch=self.wall_clock_batch,
            refined_stream=self.refined_stream,underrun_recovering=False,
            batch_tick_s=.005 if self.fixed_tick_batch else None,
            batch_max_points=self.batch_point_limit if self.fixed_tick_batch else None,
            batch_max_pending_points=8 if self.fixed_tick_batch and not self.wall_clock_batch else None,
            motion_profile=self.motion_profile.metadata(),braking_tracking_limit_deg=math.degrees(self.braking_tracking_limit_rad),
            holding_tolerance_deg=math.degrees(self.holding_tolerance_rad),
            batch_underrun_gap_s=self.underrun_points*.005,
            batch_nominal_cushion_points=(self.buffer_points if self.refined_stream else 8) if self.wall_clock_batch else None,
            state_monitoring_during_stream='A_calls_and_C_joints' if self.refined_stream else 'B_state_and_joints',
            batch_queue_bound_verified=False,
            sdk_profile_controls_passthrough=False if command_mode == "passthrough" else None)
        self._status.update(priming_ready=False,priming_steps=0,priming_consecutive_fast=0,
            priming_max_s=self.priming_max_s,priming_age_limit_s=self.sdk_budget_s if self.refined_stream else self.priming_limit_s,
            active_since_monotonic_s=None,priming_reanchor_required=priming_reanchor_required,
            hot_ready=False,hot_fast=0,consecutive_slow_sends=0,
            hot_feedback_limits_s=dict(state=self.hot_state_max_age_s,joints=self.hot_joint_max_age_s),hold_q_rad=None,holding_idle_s=self.holding_idle_s,
            cold_enter_not_before_s=0.)
        self.state_query = (StateQueryWorker(state_observer,
            on_failure=lambda reason:self.revoke(reason, fatal=True),
            is_active=lambda:self.status()["state"] in ("active","braking","holding"), diagnose=self._diagnose, clock=clock,
            active_period_s=self.active_push_query_period_s, idle_period_s=self.idle_query_period_s,
            budget_s=self.sdk_budget_s) if self.parallel_state_query else None)
        if self.parallel_state_query:
            connections = getattr(state_observer, "sdk_connections", 2)
            self._status.update(state_query_policy="independent_sdk_connection",
                state_query_mode="parallel", state_query_mode_label="并行查询（双 SDK 连接）",
                state_query_schedule="start_to_start_single_inflight_no_catchup",
                serial_sdk_calls=False, motion_sdk_calls_serial=True, sdk_connections=connections)
            if getattr(state_observer, "process_isolated", False):
                self._status.update(state_query_mode="process",
                    state_query_mode_label=f"独立进程查询（{connections} 条 SDK 连接）",
                    state_query_policy="process_isolated_sdk_connection",
                    state_observer_process=state_observer.status())

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
        with self._push_read_lock:
            return self._measured_feedback()

    def _measured_feedback(self):
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
        return self._push_cache.require_recent(self._reception_limit())

    def _hot_feedback_phase(self):
        return self.command_mode=='passthrough' and self._status['state'] in ('active','braking','holding')

    def _state_age_required(self):
        return not (self.refined_stream and self._state_poll_paused and
            self._status['state'] in ('priming','active','braking','holding'))

    def _reception_limit(self, channel='joints'):
        if self._status['state']=='priming':return self.sdk_budget_s if self.refined_stream else self.priming_limit_s
        if self._hot_feedback_phase():
            return self.hot_state_max_age_s if channel=='state' else self.hot_joint_max_age_s
        return self.feedback_max_age_s

    def _check_send_streak(self, result):
        if self.command_mode!='passthrough':return True
        self._slow_sends=self._slow_sends+1 if result['duration_s']>self.slow_send_s else 0
        self._status['consecutive_slow_sends']=self._slow_sends
        if self._slow_sends>=self.slow_send_count:
            self.revoke('consecutive_slow_sends',fatal=True)
            return False
        return True

    def acknowledge_priming_reanchor(self, session, stamp):
        """Control owner confirms that current hand pose was anchored; no old IK replay."""
        with self._lock:
            if (self._status['state']!='priming' or not self._status['priming_ready'] or
                    self._stop_needed or self._wanted is None or session!=self._active_session or
                    not math.isfinite(stamp) or not self._priming_started_s<=stamp<=self.clock()):
                return False
            self._priming_reanchor_ack=stamp
            # Replace any goal generated during warmup with the static anchor.
            self._wanted=(self._priming_anchor,stamp,session,self._active_source)
            return True

    def _record_planner_input(self, q, mode, source=None):
        if self.recorder:
            self.recorder.planner_target(q, control_source=source or self._active_source or 'teleop',
                                        mode=mode, control_session=self._active_session,
                                        stamp=self.clock())

    def offer(self, candidate, *, control_source='teleop'):
        # Candidate is the author's output; the backend only bounds submission.
        try:
            q = joints(candidate.q_rad)
            stamp = float(candidate.generated_monotonic_s)
            valid = (control_source in ('teleop', 'home_return', 'placement') and
                     (control_source != 'home_return' or self.six_axis) and
                     (control_source != 'placement' or (self.six_axis and self.command_mode == 'follow')) and
                     math.isfinite(stamp) and 0 <= self.clock() - stamp <= self.candidate_max_age_s)
        except (ValueError, TypeError, OverflowError):
            valid = False
        if not valid:
            self.revoke("invalid_or_expired_candidate", fatal=True)
            return False
        with self._lock:
            if self._quit.is_set() or self._fatal or self._unknown or self._stop_needed:
                return False
            if self._status['state'] in ('braking','holding'):return False
            self._wanted = (q, stamp, candidate.session, control_source)
        self._wake.set()
        return True

    def resume_clutch(self, session, stamp, *, reanchor=None):
        """Owner has reanchored latest hand pose; atomically admit a hot session."""
        with self._lock:
            now=self.clock()
            if (self._status['state'] not in (('braking','holding') if self.refined_stream else ('holding',)) or
                    (not self.refined_stream and not self._status['hot_ready']) or
                    self._stop_needed or self._fatal or self._unknown or
                    not math.isfinite(stamp) or not 0<=now-stamp<=.1 or
                    self._input_s is None or not 0<=now-self._input_s<=.1):return False
            snap=self._status['last_snapshot']
            if not self._hot_admissible(now,snap):return False
            anchor=tuple(self._trajectory.q) if self.refined_stream else self._hold_q
            if reanchor is not None and not reanchor(anchor):return False
            if not self.refined_stream:self._trajectory.reset(anchor)
            self._brake_pending=False
            self._first_active_hold=not self.refined_stream
            self._active_session=session
            self._active_source='teleop'
            self._wanted=(anchor,now,session,'teleop')
            self._priming_anchor=anchor
            self._status.update(state='active',hot_ready=False,active_since_monotonic_s=now,
                active_anchor_rad=list(anchor),reanchor_monotonic_s=stamp)
            self._diagnose('hot_clutch_resumed',session=session,q_anchor_rad=list(anchor),
                v_anchor_rad_s=list(self._trajectory.v),a_anchor_rad_s2=list(self._trajectory.a),
                reanchor_monotonic_s=stamp,feedback_snapshot=snap)
            self._wake.set()
            return True

    def _hot_admissible(self, now, snap):
        if self.refined_stream:
            return (snap is not None and record_ok(snap['joints']) and record_ok(snap['state']) and
                snap['state']['data'] in (0,1) and
                0<=now-snap['joints']['host_received_monotonic_s']<=self.hot_joint_max_age_s)
        return (self._underrun is None and
            self._last_send_s is not None and 0<=now-self._last_send_s<=.05 and
            self._hot_quiet_s is not None and snap is not None and
            snap['joints']['host_received_monotonic_s']-self._hot_quiet_s>=.1 and
            max(abs(a-b) for a,b in zip(snap['joints']['data'],self._hot_quiet_q))<=math.radians(.02) and
            self._hot_fast>=5 and snap['state']['data'] in (0,1) and
            (not self._state_age_required() or 0<=now-snap['state']['host_received_monotonic_s']<=self.resume_state_max_age_s) and
            0<=now-snap['joints']['host_received_monotonic_s']<=self.resume_joint_max_age_s and
            max(abs(a-b) for a,b in zip(snap['joints']['data'],self._hold_q))<=self.holding_tolerance_rad)

    def _release_hot_clutch(self):
        with self._lock:
            phase=self._status['state']
            if self.command_mode!='passthrough' or phase not in ('active','braking','holding'):
                return False
            if self._stop_needed or self._fatal or self._unknown:return False
            self._wanted=None
            if phase=='active':
                self._brake_pending=True
                self._hot_released_s=self.clock()
                self._hot_fast=0
                self._hot_quiet_q=self._hot_quiet_s=None
                self._status.update(state='braking',hot_ready=False)
                self._diagnose('hot_clutch_released',q_cmd_rad=self._trajectory.q,
                    v_cmd_rad_s=self._trajectory.v,a_cmd_rad_s2=self._trajectory.a)
            self._wake.set()
            return True

    def revoke(self, reason, *, fatal=False):
        if reason=='grip_released' and not fatal and self._release_hot_clutch():return
        # Native revoke is atomic, has no SDK call and takes no I/O mutex.
        self.client.revoke()
        with self._lock:
            if self.recorder:
                self.recorder.invalidate_planner(reason, stamp=self.clock())
            # Preserve the initiating host event before secondary feedback or
            # transport failures overwrite the current reason.
            now = self.clock()
            if self._status['state'] in ('arming','priming') and reason in ('priming_timeout','grip_released'):
                snap=self._status['last_snapshot']
                self._cold_abort_anchor=(self._priming_anchor if self._status['state']=='priming' else
                    tuple(snap['joints']['data']) if snap else None)
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
            self._status["hot_ready"] = False
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
                immediate_hot_clutch=self.refined_stream,
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
        priming=stop_deadline is None and self._status['state']=='priming'
        if self.parallel_state_query and snap.get("observation_pending") and not (
                stop_deadline is None and all(0<=v and (k=='state' and not self._state_age_required() or v<=self._reception_limit(k)) for k,v in ages.items())):
            raise ObservationPending("Parallel observation pending: "
                f"joint_age_s={ages['joints']:.6f}, state_age_s={ages['state']:.6f}", snap)
        # The legacy snapshot validator accepts only motion-permitted states.
        # During stop confirmation, Paused/Resumed are valid observed
        # states to keep polling, but can never confirm stopped or authorize send.
        # ACK-only SlowStop can return before these transitions finish on either
        # serial or parallel feedback; both must await RobotStopped below.
        if (stop_deadline is not None and
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
        if (stop_deadline is None and not self.parallel_state_query and
                self.waypoint_feedback_enabled and
                (callable(getattr(type(self.client), "waypoint_feedback_since", None)) or
                 callable(getattr(self.client, "waypoint_feedback", None))) and
                now-float(snap["joints"]["host_received_monotonic_s"]) > self.feedback_max_age_s and
                0 <= now-float(snap["state"]["host_received_monotonic_s"]) <= self.feedback_max_age_s):
            # A snapshot may copy joint push before the state request, then
            # return after a host scheduling pause. The SDK's independent push
            # cache can already contain newer real observations. Read that
            # memory once; never relabel the old joint sample with host 'now',
            # retry SDK I/O, or extend either channel's freshness limit.
            self._check_measured(joints(snap["joints"]["data"]))
            fresh = self.measured_feedback()
            old_joints = dict(snap["joints"])
            snap = dict(snap, snapshot_joints_before_refresh=old_joints,
                joints=dict(old_joints, data=list(fresh.q_rad), source=fresh.source,
                    host_started_monotonic_s=fresh.received_monotonic_s,
                    host_received_monotonic_s=fresh.received_monotonic_s, duration_s=0.))
            now = self.clock()
            with self._lock:
                self._status["last_snapshot"] = snap
                self._status["last_observation_attempt"] = dict(snapshot=snap,
                    checked_monotonic_s=now, reception_age_s={key:
                        now-float(snap[key]["host_received_monotonic_s"]) for key in ("joints", "state")})
            self._diagnose("serial_snapshot_joint_refresh", old_joints=old_joints,
                new_joints=snap["joints"], state=snap["state"])
        if any(not 0 <= now-float(snap[key]["host_received_monotonic_s"]) <= (self._reception_limit(key) if stop_deadline is None else self.feedback_max_age_s)
               for key in ("joints", "state") if not (key=='state' and stop_deadline is None and not self._state_age_required())):
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
                self.revoke("feedback_reception_expired",fatal=self._status['state']=='priming' or self._hot_feedback_phase())
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
            # Idle cache misses must yield so B/C can publish fresh feedback.
            # Re-revoking an already empty intent sets _wake on every cycle,
            # turning the wait below into a busy loop and starving receivers.
            if first_wait or self._wanted is not None:
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
            priming = self._status['state']=='priming'
        observation = self.state_query.status() if self.state_query else None
        # A B result can be published while status() is read. Validate the
        # copied records against a later clock sample, never an earlier one.
        now = self.clock()
        if self.state_query:
            query = observation["sdk_inflight"]
            if query and now-query["started_s"] > query["budget_s"]:
                self.state_query.fail("query_over_budget")
            raw = observation["last_result"]
            if active and self._state_age_required():
                age = None if raw is None else now-raw["host_received_monotonic_s"]
                if age is None or not math.isfinite(age) or age < 0:
                    self.state_query.fail("state_feedback_invalid")
                elif age > self._reception_limit('state'):
                    with self._lock:
                        self._status["feedback_ready"] = False
                    self.revoke("state_reception_expired",fatal=self._hot_feedback_phase())
            if active and self.waypoint_feedback_enabled:
                try:
                    # Nonblocking callback cache read also runs while A's RPC is in flight.
                    sample=self.measured_feedback()
                    if priming and self._priming_anchor is not None and max(
                            abs(a-b) for a,b in zip(sample.q_rad,self._priming_anchor))>math.radians(.1):
                        self.revoke('priming_pose_deviation',fatal=True)
                    elif self._status['state']=='holding' and self._hold_q is not None and max(
                            abs(a-b) for a,b in zip(sample.q_rad,self._hold_q))>self.holding_tolerance_rad:
                        self.revoke('hot_holding_pose_deviation',fatal=True)
                except (RuntimeError,ValueError,TypeError,KeyError) as exc:
                    self.revoke('joint_reception_expired: '+str(exc),fatal=priming or self._hot_feedback_phase())
        if active and priming and self._priming_started_s is not None and now-self._priming_started_s>self.priming_max_s:
            self.revoke('priming_timeout')
        if inflight and now-inflight["started_s"] > inflight["budget_s"]:
            reason = {"slow_stop": "stop_completion_over_budget_operator_stop_required",
                      "stop_snapshot": "stop_confirmation_timeout_operator_stop_required"}.get(
                          inflight["operation"], "sdk_call_over_budget_operator_stop_required")
            self.revoke(reason, fatal=True)
            with self._lock:
                self._status["operator_stop_required"] = True
        elif active:
            # A watchdog may be descheduled after copying an intent. Recheck
            # the current producer stamps under the same lock as revocation.
            with self._lock:
                check_now=self.clock()
                if self._may_move and not self._stop_needed and not self._unknown:
                    if self._input_s is None or check_now-self._input_s>self.input_max_age_s:
                        self.revoke('input_timeout')
                    elif self._wanted and check_now-self._wanted[1]>self.candidate_max_age_s:
                        self.revoke('candidate_timeout')

    def _watchdog(self):
        while not self._watch_quit.wait(.01):
            self.watchdog_tick()

    def _stop_and_confirm(self):
        with self._lock:
            self._status.update(state="stopping", stop_confirmed=False, stop_confirmed_monotonic_s=None)
        self._diagnose("slow_stop_call_started")
        try:
            result = self._call("slow_stop", self.client.stop)
            if self.refined_stream and self.state_query:
                self._state_poll_paused=False
                self.state_query.resume()
                self._diagnose('state_polling_resumed_after_exit')
            with self._lock:
                self._status["last_stop"] = result
                leave=result.get('leave_passthrough_result')
                if self.command_mode=='passthrough':
                    ended=(leave or result).get('host_received_monotonic_s',self.clock())
                    self._cold_enter_not_before=ended+self.cold_cooldown_s
                    self._status['cold_enter_not_before_s']=self._cold_enter_not_before
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
                if self._cold_abort_anchor is not None and max(abs(a-b) for a,b in zip(q,self._cold_abort_anchor))>math.radians(.1):
                    self.revoke('priming_abort_pose_deviation',fatal=True)
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
                                self._cold_abort_anchor = None
                                self._engagement_barrier = None
                                # Recovery belongs to the exited native stream. A new
                                # cold entry gets a new point ledger and endpoint.
                                self._underrun=None
                                self._next_batch_s=None
                                self._hot_fast=0
                                self._status.update(underrun_recovering=False,hot_ready=False,hot_fast=0)
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

    def _priming_cycle(self, q, state, wanted):
        now=self.clock()
        if now-self._priming_started_s>self.priming_max_s:
            self.revoke('priming_timeout');return
        if max(abs(a-b) for a,b in zip(q,self._priming_anchor))>math.radians(.1):
            self.revoke('priming_pose_deviation',fatal=True);return
        snapshot=self._status['last_snapshot']
        ready=(self._priming_steps>=self.priming_min_steps and
            self._priming_fast>=self.priming_fast_count and state in (0,1) and
            (not self._state_age_required() or 0<=now-snapshot['state']['host_received_monotonic_s']<=.1) and
            0<=now-snapshot['joints']['host_received_monotonic_s']<=.06)
        with self._lock:
            self._status.update(priming_ready=ready,priming_steps=self._priming_steps,
                priming_consecutive_fast=self._priming_fast,priming_elapsed_s=now-self._priming_started_s)
            ack=self._priming_reanchor_ack
            if not ready:self._priming_reanchor_ack=None
        if ready and (not self._priming_reanchor_required or (ack is not None and now-ack<=.1)):
            with self._lock:
                if self._stop_needed or self._fatal or self._wanted is None:return
                if not self.client.finish_priming(self._token):return
                transitioned=self.clock()
                # Automatic home uses the same bounded planner and sender as
                # teleop, with the home speed/acceleration caps. Restore the
                # selected teleop profile on the next cold engagement.
                limits=self.motion_profile.radians
                if wanted[3]=='home_return':
                    from .home import HOME_SPEED_RAD_S, HOME_ACCEL_RAD_S2
                    limits=(min(limits[0],HOME_SPEED_RAD_S),
                            min(limits[1],HOME_ACCEL_RAD_S2),limits[2])
                (self._trajectory.velocity_rad_s,self._trajectory.acceleration_rad_s2,
                 self._trajectory.jerk_rad_s3)=limits
                self._trajectory.reset(self._priming_anchor)
                self._record_planner_input(self._priming_anchor, 'active', wanted[3])
                self._first_active_hold=True
                self._wanted=(self._priming_anchor,transitioned,wanted[2],wanted[3])
                self._status.update(state='active',priming_ready=False,
                    active_since_monotonic_s=transitioned,
                    active_anchor_rad=list(self._priming_anchor),reanchor_monotonic_s=ack)
                self._diagnose('priming_completed',steps=self._priming_steps,
                    consecutive_fast=self._priming_fast,elapsed_s=transitioned-self._priming_started_s,
                    q_anchor_rad=list(self._priming_anchor),reanchor_monotonic_s=ack,
                    feedback_snapshot=snapshot)
            return
        if self._priming_steps and now-self._last_send_s<self.period_s:return
        step=self._priming_steps
        priming_call_limit=self.sdk_budget_s if self.refined_stream else self.priming_limit_s
        result=self._call('priming_hold',lambda:self.client.send_priming(self._token),budget_s=priming_call_limit)
        attempt=dict(session=wanted[2],phase='priming',priming_step=step,
            requested_q_rad=list(self._priming_anchor),sdk_result=result,
            first_measured_hold=step==0,outcome='sdk_accepted' if record_ok(result) else 'sdk_rejected')
        with self._lock:
            self._status['last_attempt']=attempt
            self._status['submitted']+=1
        self._diagnose('send_returned',attempt=attempt)
        if self._stop_needed:return
        if not record_ok(result) or result.get('revoked_during_call') or result['duration_s']>priming_call_limit:
            self.revoke('priming_send_failed_or_late',fatal=True);return
        if self.refined_stream and not self._check_send_streak(result):return
        self._priming_steps+=1
        self._priming_fast=self._priming_fast+1 if result['duration_s']<=self.priming_fast_s else 0
        self._last_send_s=result['host_started_monotonic_s']
        self._last_sent_q=self._priming_anchor

    def _fixed_tick_send(self, goal, phase):
        # Wall pacing uses a native monotonic point ledger, not a diagnosis RPC.
        # The old watermark experiment remains independently selectable.
        with self._lock:
            if self._stop_needed or self._fatal or self._status['state']!=phase:return
        if self.wall_clock_batch:
            stationary=(phase=='holding' or
                all(abs(x)<=1e-12 for x in self._trajectory.v+self._trajectory.a))
            plan=(self.client.clock_batch_plan(self._token,self.batch_point_limit,stationary)
                if self.refined_stream else self.client.clock_batch_plan(self._token,self.batch_point_limit))
            if self.refined_stream and plan.get('recovery_required'):
                self._recover_underrun(plan)
                return
            self._next_batch_s=plan['next_due_monotonic_s']
            if plan['discarded_wall_ticks']:
                self._diagnose('batch_wall_clock_rebased',plan=plan,phase=phase)
            slots=plan['point_count'];diagnosis=None
        elif self._batch_credit<=0 or self.clock()-self._batch_credit_received_s>.035:
            diagnosis=self._call('batch_diagnosis',self.client.query_diagnosis)
            self._diagnose('batch_diagnosis_returned',result=diagnosis,phase=phase)
            if not record_ok(diagnosis):
                self.revoke('batch_diagnosis_failed',fatal=True);return
            raw=diagnosis['data']['buffer_data_raw']
            if type(raw) is not int or raw<0 or raw%6 or raw>48:
                self.revoke('batch_watermark_invalid_or_excessive',fatal=True);return
            self._batch_credit=8-raw//6
            self._batch_credit_received_s=diagnosis['host_received_monotonic_s']
            self._batch_credit_diagnosis=diagnosis
        if not self.wall_clock_batch:
            diagnosis=self._batch_credit_diagnosis
            slots=min(self.batch_point_limit,self._batch_credit)
            plan=None
        with self._lock:
            if self._stop_needed or self._fatal or self._status['state']!=phase:return
            if not slots:return
            first=phase=='active' and self._first_active_hold
            if phase=='active':
                wanted=self._wanted
                if wanted is None:return
                goal=self.project_target(wanted[0])
                if wanted[1]<=self._status['active_since_monotonic_s']:goal=self._priming_anchor
            self._record_planner_input(goal, phase)
            points,detail=self._trajectory.sample_fixed_ticks(goal,slots,first_anchor=first)
            if phase=='holding':points=tuple(self._hold_q for _ in points)
            before=self._last_sent_q
            if any(max(abs(a-b) for a,b in zip(q,p))>self._trajectory.velocity_rad_s*.005+1e-10
                   for p,q in zip((before,)+points[:-1],points)):
                self.revoke('batch_step_limit',fatal=True);return
        if not self.wall_clock_batch:self._batch_credit-=len(points)
        try:
            result=self._call('fixed_tick_batch',lambda:(self.client.send_batch(points,self._token,True)
                if self.wall_clock_batch else self.client.send_batch(points,self._token)))
        except RuntimeError as exc:
            if not self.refined_stream or str(exc)!='Clock recovery required before sending':raise
            # The ledger can expire while Python samples the curve. Native
            # rejects before SDK submission; restart at its last sent endpoint.
            self._diagnose('batch_blocked_for_underrun',phase=phase,wall_clock_plan=plan)
            self._recover_underrun(dict(plan,recovery_required=True,detected_before_submit=True))
            return
        attempt=dict(phase=phase,session=self._active_session,requested_q_rad=list(points[-1]),
            generated_q_rad=list(goal),trajectory=detail,sdk_result=result,
            feedback_snapshot=self._status['last_snapshot'],queue_diagnosis=diagnosis,wall_clock_plan=plan)
        self._diagnose('send_returned',attempt=attempt)
        with self._lock:
            self._status['last_attempt']=attempt
            self._status['submitted']+=1
            # An intentional stop can revoke the native permit while this RPC
            # is in flight. Preserve that stop rather than inventing a fault.
            if self._stop_needed:return
            if not record_ok(result) or result.get('revoked_during_call') or result['duration_s']>self.sdk_budget_s:
                self.revoke('batch_send_failed_or_late',fatal=True);return
            if not self._check_send_streak(result):return
            self._first_active_hold=False
            self._hot_fast=self._hot_fast+1 if result['duration_s']<=self.priming_fast_s else 0
            self._last_send_s=result['host_started_monotonic_s']
            self._last_sent_q=points[-1]
            if self.wall_clock_batch:
                self._next_batch_s=result['clock_epoch_monotonic_s']+(result['clock_sent_points']+
                    result.get('clock_minimum_points',4)-result.get('clock_cushion_points',8))*.005
            self._send_samples.append(dict(started_s=self._last_send_s,duration_s=result['duration_s'],
                session=self._active_session,accepted=True))

    def _recover_underrun(self,plan):
        """Immediately restart the reference at the submitted endpoint, v=a=0.

        The native ledger detected estimated net starvation after prefill. It
        still enforces valid C feedback <=500ms and the A-call hard limits.
        No quiet-window wait or declaration of measured physical rest occurs.
        """
        with self._lock:
            if self._stop_needed or self._fatal:return
            before=dict(q=self._trajectory.q,v=self._trajectory.v,a=self._trajectory.a)
            ack=self.client.restart_clock_from_rest(self._token)
            endpoint=tuple(ack['q_restart_rad'])
            self._trajectory.restart_from_rest(endpoint)
            self._first_active_hold=True
            if self._status['state'] in ('braking','holding'):
                if hasattr(self._trajectory,'program_enabled'):self._trajectory.program_enabled=False
                self._hold_q,_=self._trajectory.brake()
                self._brake_pending=False
                self._status['hold_q_rad']=list(self._hold_q)
                self._hot_quiet_q=self._hot_quiet_s=None
            self._underrun=None
            self._status.update(underrun_recovering=False,hot_ready=False)
            self._next_batch_s=self.clock()
            self._diagnose('underrun_zero_velocity_restart',phase=self._status['state'],
                recovery=ack,wall_clock_plan=plan,reference_before=before,
                q_cmd_rad=self._trajectory.q,v_cmd_rad_s=self._trajectory.v,a_cmd_rad_s2=self._trajectory.a,
                measured_rest_asserted=False,quiet_wait_s=0.)

    def _hot_cycle(self, q, state):
        now=self.clock()
        with self._lock:
            if self._stop_needed or self._fatal:return
            phase=self._status['state']
            if state not in (0,1):
                self.revoke('hot_clutch_invalid_robot_state',fatal=True);return
            if self._brake_pending:
                self._hold_q,duration=self._trajectory.brake()
                self._brake_pending=False
                self._record_planner_input(self._hold_q, 'braking')
                self._status['hold_q_rad']=list(self._hold_q)
                self._diagnose('hot_brake_planned',q_hold_rad=self._hold_q,duration_s=duration)
            if phase=='braking':
                if now-self._hot_released_s>self.braking_max_s:
                    self.revoke('hot_braking_settle_timeout',fatal=True);return
                if max(abs(a-b) for a,b in zip(q,self._last_sent_q))>self.braking_tracking_limit_rad:
                    self.revoke('hot_braking_tracking_deviation',fatal=True);return
                if self._trajectory.stopped:
                    stamp=self._status['last_snapshot']['joints']['host_received_monotonic_s']
                    near=max(abs(a-b) for a,b in zip(q,self._hold_q))<=self.holding_tolerance_rad
                    quiet=self._hot_quiet_q is not None and max(abs(a-b) for a,b in zip(q,self._hot_quiet_q))<=math.radians(.02)
                    if not near or not quiet:
                        self._hot_quiet_q,self._hot_quiet_s=q,stamp
                    elif stamp-self._hot_quiet_s>=.1:
                        self._holding_since_s=now
                        self._status.update(state='holding',hot_ready=False)
                        self._diagnose('hot_holding_entered',q_hold_rad=self._hold_q,
                            elapsed_s=now-self._hot_released_s,feedback_snapshot=self._status['last_snapshot'])
                        phase='holding'
            if phase=='holding':
                if max(abs(a-b) for a,b in zip(q,self._hold_q))>self.holding_tolerance_rad:
                    self.revoke('hot_holding_pose_deviation',fatal=True);return
                stamp=self._status['last_snapshot']['joints']['host_received_monotonic_s']
                if self._hot_quiet_q is None or max(abs(a-b) for a,b in zip(q,self._hot_quiet_q))>math.radians(.02):
                    self._hot_quiet_q,self._hot_quiet_s=q,stamp
                ready=self._hot_admissible(now,self._status['last_snapshot'])
                self._status.update(hot_ready=ready,hot_fast=self._hot_fast)
                if now-self._holding_since_s>=self.holding_idle_s:
                    self.revoke('hot_holding_idle_timeout');return
            dt=now-self._last_send_s
            if dt<self.period_s and not self.wall_clock_batch:return
            if dt>self.send_max_gap_s:
                self.revoke('send_period_over_budget');return
            if not self.fixed_tick_batch:
                self._record_planner_input(self._hold_q, phase)
            target,detail=(None,None) if self.fixed_tick_batch else self._trajectory.advance(self._hold_q,dt)
            if phase=='holding':target=self._hold_q
        if self.fixed_tick_batch:
            self._fixed_tick_send(self._hold_q,phase);return
        result=self._call('hot_'+phase,lambda:self.client.send_joints(target,self._token))
        attempt=dict(phase=phase,session=self._active_session,requested_q_rad=list(target),
            trajectory=detail,sdk_result=result,feedback_snapshot=self._status['last_snapshot'])
        self._diagnose('send_returned',attempt=attempt)
        if self.recorder:self.recorder.record('sdk_command',phase='returned',attempt=attempt)
        with self._lock:
            self._status['last_attempt']=attempt
            self._status['submitted']+=int(result.get('sdk_invoked') is not False)
            if not record_ok(result) or result.get('sdk_invoked') is False or result['duration_s']>self.sdk_budget_s:
                self.revoke('hot_send_failed_or_late',fatal=True);return
            if not self._check_send_streak(result):return
            self._hot_fast=self._hot_fast+1 if result['duration_s']<=self.priming_fast_s else 0
            self._last_send_s=result['host_started_monotonic_s']
            self._last_sent_q=target
            self._send_samples.append(dict(started_s=self._last_send_s,duration_s=result['duration_s'],
                session=self._active_session,accepted=True))

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
        if self._status['state']=='priming':needs_observation=True
        if needs_observation:
            try:
                q, state = self._observe()
            except ObservationPending as pending:
                self._handle_pending_observation(pending)
                return
        else:
            q, state = self.cache.require_recent(self._reception_limit()).q_rad, self._robot_state
        with self._lock:
            wanted, fatal = self._wanted, self._fatal
            input_s, active_session = self._input_s, self._active_session
        if not fatal and self._status['state'] in ('braking','holding'):
            if input_s is None or not 0<=self.clock()-input_s<=self.input_max_age_s:
                self.revoke('input_timeout');return
            self._hot_cycle(q,state)
            return
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
        if self._status['state']=='priming':
            self._priming_cycle(q,state,wanted)
            return
        first_hold = False
        if active_session is None:
            if now<self._cold_enter_not_before:return
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
                if self.refined_stream:
                    self._state_poll_paused=True
                    self.state_query.pause()
                    self._diagnose('state_polling_paused_before_entry')
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
                self._status.update(state="priming" if self.command_mode=='passthrough' else "active", fault=None)
                if self.command_mode=='passthrough':
                    self._priming_anchor=tuple(entry.get('anchor_rad',q))
                    self._priming_started_s=entry['host_received_monotonic_s']
                    self._batch_credit=0
                    self._slow_sends=0
                    self._priming_steps=self._priming_fast=0
                    self._priming_reanchor_ack=None
                    self._status.update(priming_ready=False,active_since_monotonic_s=None,
                        priming_anchor_rad=list(self._priming_anchor),
                        priming_started_monotonic_s=self._priming_started_s)
            if self.command_mode != "passthrough":
                return
            self._priming_cycle(q,state,wanted)
            return
        if not first_hold and now-self._last_send_s < self.period_s and not self.wall_clock_batch:
            return
        if now-self._last_send_s > self.send_max_gap_s:
            # A missed cadence is recoverable only after confirmed stop and a
            # new release/press. SDK errors/timeouts remain fatal separately.
            self.revoke("send_period_over_budget")
            return
        if self._status['state'] in ('braking','holding') or self._wanted is None:return
        if self.fixed_tick_batch:
            self._fixed_tick_send(wanted[0],'active');return
        projected = self.project_target(wanted[0])
        desired = (projected[5]-self.anchor[5])/math.radians(1.)
        if self.six_axis:
            # The trajectory is advanced only for one real submitted frame.
            target = projected
            trajectory_detail=None
            if self.command_mode == "passthrough":
                if self._first_active_hold:
                    # Exactly one immutable zero-speed anchor frame, even if
                    # a new IK target arrives before the first ACTIVE send.
                    self._trajectory.reset(self._priming_anchor)
                    target=self._priming_anchor
                    trajectory_detail=dict(first_active_anchor=True,q_cmd_rad=target,
                        v_cmd_rad_s=self._trajectory.v,a_cmd_rad_s2=self._trajectory.a,
                        dt_capped=False,target_replan_deferred=False)
                else:
                    if wanted[1]<=self._status['active_since_monotonic_s']:
                        projected=self._priming_anchor
                    self._record_planner_input(projected, 'active', wanted[3])
                    target,trajectory_detail=self._trajectory.advance(projected,now-self._last_send_s)
            else:
                self._record_planner_input(projected, 'active', wanted[3])
            offset = (target[5]-self.anchor[5])/math.radians(1.)
            if self.command_mode=='follow' and wanted[3] in ('home_return', 'placement'):
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
            phase='active',trajectory=trajectory_detail if self.six_axis else None,
            control_source=wanted[3], first_measured_hold=first_hold,
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
                self.revoke("feedback_reception_expired",fatal=self._hot_feedback_phase())
                return
            self.revoke("native_send_refused: "+result.get("local_refusal_reason", "unknown"), fatal=True)
            return
        if not record_ok(result) or result["duration_s"] > self.sdk_budget_s:
            self.revoke("follow_failed_or_late", fatal=True)
            return
        if not self._check_send_streak(result):return
        self._first_active_hold=False
        self._hot_fast=self._hot_fast+1 if result['duration_s']<=self.priming_fast_s else 0
        self._last_send_s = result["host_started_monotonic_s"]
        self._last_sent_offset = offset
        self._last_sent_q = target
        if self.command_mode=='follow' and wanted[3] in ('home_return', 'placement'):
            self._home_velocity = home_velocity
        # Never re-authorize after a call returned following revoke.

    def _wait_timeout_s(self):
        """Wake by the next send deadline without adding a fixed 5 ms delay."""
        with self._lock:
            if not self._may_move or self._stop_needed or self._unknown:
                return .005
            if self.wall_clock_batch and self._status['state'] in ('active','braking','holding') and self._next_batch_s is not None:
                return max(0.,min(.02,self._next_batch_s-self.clock()))
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
                        cancelled_before_sdk = self._stop_needed and str(exc) in (
                            "Send permission revoked", "Clock batch permission revoked or wrong mode",
                            "Batch send permission revoked or wrong mode")
                    if (self._status['state']=='priming' and str(exc)=='Priming deadline expired'):
                        self.revoke('priming_timeout')
                        continue
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
