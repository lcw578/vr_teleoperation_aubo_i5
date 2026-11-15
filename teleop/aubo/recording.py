"""State/action raw capture. Producers never serialize or perform filesystem I/O."""
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
import uuid


LEGACY_SCHEMA = 'aubo_state_action_v1'
SCHEMA = 'aubo_multimodal_v2'
JOINTS = [f'joint_{n}' for n in range(1, 7)]
FPS = (20, 25, 30, 50)
CAMERAS = ('scene_left', 'scene_right', 'wrist')
TASK_STAGES = ('unknown', 'select', 'approach', 'align', 'clamp_cut', 'withdraw',
               'transport', 'release', 'verify', 'retry', 'reset')


def pause_type(reason):
    if reason in ('grip_released', 'awaiting_grip', 'grip_reengaged'):
        return 'clutch_reanchor'
    if reason in ('tool_wait', 'clamp_wait', 'release_wait'):
        return 'task_wait'
    if reason in ('home_return', 'placement', 'placement_requested'):
        return 'automatic_motion'
    if reason == 'operator_pause':
        return 'operator_pause'
    return 'input_or_system'


def tool_values(gripper):
    """Measured position and admitted absolute goal, normalized open=0/closed=1."""
    if not gripper or gripper.get('fault') or gripper.get('hardware_error') not in (None, 0):
        return None
    try:
        settings = gripper['settings']
        opened, closed = settings['open_position'], settings['close_position']
        position, stamp = gripper['position'], gripper['feedback_received_s']
        if (not all(isinstance(v, (int, float)) and math.isfinite(v)
                    for v in (opened, closed, position, stamp)) or closed <= opened):
            return None
        goal = gripper.get('commanded_closed')
        if type(goal) is not bool:
            return None
        measured = (position-opened)/(closed-opened)
        if not -.1 <= measured <= 1.1:
            return None
        return dict(position=max(0., min(1., measured)), target=float(goal),
                    received_monotonic_s=stamp)
    except (KeyError, TypeError, ValueError):
        return None


def _allowed_roots():
    """Bulk-output roots: /data always, plus AUBO_ALLOWED_ROOTS (colon-separated)."""
    roots = [Path('/data')]
    for part in os.environ.get('AUBO_ALLOWED_ROOTS', '').split(':'):
        if part:
            roots.append(Path(part).expanduser().resolve())
    return roots


def recording_path(value):
    base = Path(os.environ.get('AUBO_RUNTIME_DIR', '/data/VR_teleoperation_runtime'),
                'recordings', 'aubo')
    path = (base / (time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
            if value == 'auto' else Path(value).expanduser()).resolve()
    if not any(root in path.parents for root in _allowed_roots()):
        raise ValueError('Recording must resolve under /data or AUBO_ALLOWED_ROOTS')
    if path.exists():
        raise ValueError('Recording directory must be new; existing data is never overwritten')
    return path


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    with temporary.open('w') as output:
        json.dump(value, output, ensure_ascii=False, allow_nan=False, indent=2)
        output.write('\n')
        output.flush()
        os.fsync(output.fileno())
    temporary.replace(path)


class RawRecorder:
    def __init__(self, path, *, task, fps=30, metadata=None, cameras=None, clock=time.monotonic, capacity=4096):
        if not task.strip() or fps not in FPS:
            raise ValueError('A task description and recording FPS of 20, 25, 30 or 50 are required')
        self.path, self.task, self.fps, self.clock = Path(path), task.strip(), fps, clock
        self._lock = threading.RLock()
        self._queue = queue.Queue(maxsize=capacity)
        self._quit = threading.Event()
        self._thread = threading.Thread(target=self._run, name='aubo-recording-writer', daemon=True)
        self._active = None
        self._active_started_monotonic_s = None
        self._paused = False
        self._pause_reason = None
        self._pause_kind = 'none'
        self._stage = 'unknown'
        self._planner = None
        self._planner_revision = 0
        self.cameras = cameras or {}
        self.session_id = self.path.name
        self._paused_started_monotonic_s = None
        self._paused_total_s = 0.
        self._next_episode = self._sequence = 0
        self._saving = set()
        self._saved = self._invalid = self._written = self._dropped = 0
        self.error = None
        self._closed = False
        self._last_result = None
        self._next_sample_s = None
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path/'episodes').mkdir()
        active_cameras = [key for key in CAMERAS if key in self.cameras]
        atomic_json(self.path/'manifest.json', dict(schema=SCHEMA, mode='multimodal_planner_target',
            session_id=self.session_id,
            task=self.task, sample_fps=fps, joint_names=JOINTS, joint_unit='rad', cameras=self.cameras,
            active_cameras=active_cameras,
            state_names=JOINTS+['gripper'], action_names=JOINTS+['gripper'],
            action_semantics='admitted absolute six-joint planner input (rad) and absolute tool goal (open=0, closed=1); not SDK endpoints',
            tool_semantics='measured position normalized by recorded open/close calibration; target is admitted commanded_closed',
            pause_policy='continuous_raw_capture; typed pauses annotate rather than suspend',
            clock='host_monotonic_seconds; XR source milliseconds kept separately; controller sample time unknown',
            started_monotonic_s=self.clock(), started_wall_time_ns=time.time_ns(),
            metadata=metadata or {}))
        self._thread.start()

    def status(self):
        with self._lock:
            elapsed = None
            if self._active_started_monotonic_s is not None:
                now = self.clock()
                elapsed = max(0., now-self._active_started_monotonic_s-self._paused_total_s-
                    (max(0., now-self._paused_started_monotonic_s)
                     if self._paused_started_monotonic_s is not None else 0.))
            return dict(enabled=True, state='error' if self.error else
                        'paused' if self._active is not None and self._paused else
                        'recording' if self._active is not None else 'saving' if self._saving else 'ready',
                        active_episode=self._active, saved_episodes=self._saved, invalid_episodes=self._invalid,
                        active_episode_elapsed_s=elapsed, saving_episodes=sorted(self._saving),
                        active_episode_started_monotonic_s=self._active_started_monotonic_s,
                        recording_paused=self._paused, pause_reason=self._pause_reason,
                        pending_saves=len(self._saving), last_result=self._last_result,
                        error=self.error, dropped_records=self._dropped, written_records=self._written,
                        path=str(self.path), fps=self.fps, cameras=bool(self.cameras),
                        task_stage=self._stage, raw_capture_continuous=True)

    def _put(self, kind, stamp, data):
        if self.error or self._closed:
            return False
        self._sequence += 1
        row = dict(kind=kind, sequence=self._sequence, host_monotonic_s=stamp,
                   episode_id=self._active, **data)
        try:
            self._queue.put_nowait(row)
            return True
        except queue.Full:
            self._dropped += 1
            self.error = 'recording_queue_overflow; session incomplete'
            return False

    def record(self, kind, *, stamp=None, **data):
        with self._lock:
            return self._put(kind, self.clock() if stamp is None else stamp, data)

    def planner_target(self, q_rad, *, control_source, mode, control_session=None, stamp=None):
        """Called at the planner boundary, never reconstructed from SDK returns."""
        q = list(q_rad)
        if len(q) != 6 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in q):
            raise ValueError('Six finite admitted planner joints required')
        with self._lock:
            previous = self._planner
            if previous and (previous['q_rad'], previous['control_source'], previous['mode'],
                    previous['control_session']) == (q, control_source, mode, control_session):
                return True
            self._planner_revision += 1
            at = self.clock() if stamp is None else stamp
            target = dict(q_rad=q, control_source=control_source, mode=mode,
                          control_session=control_session, admitted_monotonic_s=at,
                          revision=self._planner_revision)
            if not self._put('planner_target', at, target):
                return False
            self._planner = target
            return True

    def invalidate_planner(self, reason, *, stamp=None):
        with self._lock:
            self._planner = None
            return self._put('planner_invalid', self.clock() if stamp is None else stamp,
                             dict(reason=str(reason)))

    def set_task_stage(self, stage, *, stamp=None):
        if stage not in TASK_STAGES:
            raise ValueError('Unknown task stage')
        with self._lock:
            if stage == self._stage:
                return True
            if not self._put('task_stage', self.clock() if stamp is None else stamp, dict(stage=stage)):
                return False
            self._stage = stage
            return True

    def begin(self, stamp=None):
        with self._lock:
            if self._active is not None or self._saving or self.error or self._closed:
                return False
            self._active = self._next_episode
            self._next_episode += 1
            started = self.clock() if stamp is None else stamp
            self._active_started_monotonic_s = started
            self._paused = False
            self._pause_reason = None
            self._pause_kind = 'none'
            self._paused_started_monotonic_s = None
            self._paused_total_s = 0.
            self._stage = 'unknown'
            self._next_sample_s = None
            return self._put('episode_start', started, dict(task=self.task))

    def pause_episode(self, reason, *, stamp=None, pause_kind=None):
        """Annotate a pause without interrupting raw observations or events."""
        with self._lock:
            if self._active is None or (self._paused and self._pause_reason == str(reason)):
                return False
            at = self.clock() if stamp is None else stamp
            if not self._put('pause', at, dict(tag='pause', reason=str(reason),
                    pause_type=pause_kind or pause_type(reason), task_stage=self._stage)):
                return False
            self._paused = True
            self._pause_reason = str(reason)
            self._pause_kind = pause_kind or pause_type(reason)
            if self._paused_started_monotonic_s is None:
                self._paused_started_monotonic_s = at
            return True

    def resume_episode(self, reason='grip_reengaged', *, stamp=None):
        """Resume sampling in the same episode after safe re-engagement."""
        with self._lock:
            if self._active is None or not self._paused:
                return False
            at = self.clock() if stamp is None else stamp
            if not self._put('resume', at, dict(tag='resume', reason=str(reason))):
                return False
            self._paused = False
            self._pause_reason = None
            self._pause_kind = 'none'
            if self._paused_started_monotonic_s is not None:
                self._paused_total_s += max(0., at-self._paused_started_monotonic_s)
            self._paused_started_monotonic_s = None
            return True

    def end(self, *, disposition='saved', stamp=None):
        with self._lock:
            if self._active is None:
                return False
            episode = self._active
            self._saving.add(episode)
            ok = self._put('episode_end', self.clock() if stamp is None else stamp,
                           dict(disposition=disposition))
            self._active = None
            self._active_started_monotonic_s = None
            self._paused = False
            self._pause_reason = None
            self._paused_started_monotonic_s = None
            self._paused_total_s = 0.
            return ok

    def sample(self, *, feedback, hardware, paused=False, candidate=None, stamp=None, control_source='teleop', gripper=None):
        now = self.clock() if stamp is None else stamp
        with self._lock:
            if (self._active is None or
                    (self._next_sample_s is not None and now < self._next_sample_s)):
                return False
            period = 1./self.fps
            deadline = now if self._next_sample_s is None else self._next_sample_s
            self._next_sample_s = deadline+(math.floor(max(0.,now-deadline)/period)+1)*period
            valid = feedback is not None and feedback.valid
            return self._put('sample', now, dict(
                measured_q_rad=list(feedback.q_rad) if valid else None,
                feedback_received_monotonic_s=feedback.received_monotonic_s if feedback else None,
                feedback_source=feedback.source if feedback else None,
                feedback_valid=valid, controller_sample_time=None,
                candidate_q_rad=list(candidate.q_rad) if candidate else None,
                hardware_state=hardware['state'], hardware_fault=hardware.get('fault'),
                fatal=hardware.get('fatal', False), stop_result_unknown=hardware.get('stop_result_unknown', False),
                operator_paused=paused, control_source=control_source, last_attempt=hardware.get('last_attempt'),
                recording_paused=self._paused, pause_reason=self._pause_reason,
                pause_type=self._pause_kind if self._paused else 'none',
                task_stage=self._stage,
                planner_target=dict(self._planner) if self._planner else None,
                gripper=gripper, tool=tool_values(gripper)))

    def _run(self):
        episodes = {}
        try:
            with (self.path/'streams.jsonl').open('x') as output:
                while not self._quit.is_set() or not self._queue.empty():
                    try:
                        row = self._queue.get(timeout=.05)
                    except queue.Empty:
                        # Keep records visible during an active episode without
                        # forcing storage synchronization on the idle path.
                        output.flush()
                        continue
                    output.write(json.dumps(row, ensure_ascii=False, allow_nan=False, separators=(',', ':'))+'\n')
                    self._written += 1
                    episode = row['episode_id']
                    if row['kind'] == 'episode_start':
                        episodes[episode] = dict(episode_id=episode, task=self.task,
                            started_monotonic_s=row['host_monotonic_s'], records=0, sample_count=0,
                            pause_count=0, resume_count=0, abnormal_stop=False)
                    summary = episodes.get(episode)
                    if summary:
                        summary['records'] += 1
                        summary['sample_count'] += row['kind'] == 'sample'
                        summary['pause_count'] += row['kind'] == 'pause'
                        summary['resume_count'] += row['kind'] == 'resume'
                        summary['abnormal_stop'] |= row['kind'] == 'hardware_event' and row.get('reason') not in (
                            'grip_released', 'operator_pause', 'episode_started',
                            'input_timeout',
                            'episode_finished', 'episode_invalid', 'session_finished', 'operator_software_reset',
                            'placement_requested', 'placement_segment_completed', 'placement_complete', 'placement_cancelled',
                            'home_requested', 'home_ready', 'home_completed', 'home_cancelled',
                            'home_button_released', 'home_button_released_or_expired')
                    if row['kind'] == 'episode_end':
                        summary.update(ended_monotonic_s=row['host_monotonic_s'],
                            disposition=row['disposition'], complete=True)
                        # Durable acknowledgement is required before reporting
                        # an episode as saved.
                        output.flush()
                        os.fsync(output.fileno())
                        atomic_json(self.path/'episodes'/f'episode_{episode:06d}.json', summary)
                        with self._lock:
                            self._saving.discard(episode)
                            self._saved += row['disposition'] == 'saved'
                            self._invalid += row['disposition'] != 'saved'
                            self._last_result = dict(episode_id=episode, disposition=row['disposition'], flushed=True)
                    self._queue.task_done()
                output.flush()
                # Session close is the other durability boundary.
                os.fsync(output.fileno())
        except Exception as exc:
            with self._lock:
                self.error = type(exc).__name__+': '+str(exc)
        finally:
            try:
                atomic_json(self.path/'session_status.json', dict(schema=SCHEMA, closed=True,
                    complete=self.error is None, error=self.error, dropped_records=self._dropped,
                    written_records=self._written))
            except OSError:
                pass

    def close(self):
        if self._closed:
            return
        if self._active is not None:
            self.end(disposition='interrupted')
        with self._lock:
            self._closed = True
        self._quit.set()
        self._thread.join(timeout=5.)
        if self._thread.is_alive():
            raise RuntimeError('Recording writer still active; data not confirmed saved')
        if self.error:
            raise RuntimeError('Recording incomplete: '+self.error)
