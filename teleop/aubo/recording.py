"""State/action raw capture. Producers never serialize or perform filesystem I/O."""
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
import uuid


SCHEMA = 'aubo_state_action_v1'
JOINTS = [f'joint_{n}' for n in range(1, 7)]


def recording_path(value):
    base = Path('/data/VR_teleoperation_runtime/recordings/aubo')
    path = (base / (time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
            if value == 'auto' else Path(value).expanduser()).resolve()
    if Path('/data') not in path.parents:
        raise ValueError('Recording must resolve under /data')
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
    def __init__(self, path, *, task, fps=25, metadata=None, clock=time.monotonic, capacity=4096):
        if not task.strip() or fps not in (20, 25, 50):
            raise ValueError('A task description and recording FPS of 20, 25 or 50 are required')
        self.path, self.task, self.fps, self.clock = Path(path), task.strip(), fps, clock
        self._lock = threading.RLock()
        self._queue = queue.Queue(maxsize=capacity)
        self._quit = threading.Event()
        self._thread = threading.Thread(target=self._run, name='aubo-recording-writer', daemon=True)
        self._active = None
        self._active_started_monotonic_s = None
        self._paused = False
        self._pause_reason = None
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
        atomic_json(self.path/'manifest.json', dict(schema=SCHEMA, mode='state_action_only',
            task=self.task, sample_fps=fps, joint_names=JOINTS, joint_unit='rad', cameras={},
            action_semantics='absolute joint target accepted by SDK; not measured or proven executed',
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
                        path=str(self.path), fps=self.fps, cameras=False)

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
            if self._paused and kind in ('xr', 'sdk_command'):
                return False
            return self._put(kind, self.clock() if stamp is None else stamp, data)

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
            self._paused_started_monotonic_s = None
            self._paused_total_s = 0.
            self._next_sample_s = None
            return self._put('episode_start', started, dict(task=self.task))

    def pause_episode(self, reason, *, stamp=None):
        """Tag and suspend samples without ending the active episode."""
        with self._lock:
            if self._active is None or self._paused:
                return False
            at = self.clock() if stamp is None else stamp
            if not self._put('pause', at, dict(tag='pause', reason=str(reason))):
                return False
            self._paused = True
            self._pause_reason = str(reason)
            self._paused_started_monotonic_s = at
            self._next_sample_s = None
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
            if self._paused_started_monotonic_s is not None:
                self._paused_total_s += max(0., at-self._paused_started_monotonic_s)
            self._paused_started_monotonic_s = None
            self._next_sample_s = None
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

    def sample(self, *, feedback, hardware, paused=False, candidate=None, stamp=None, control_source='teleop'):
        now = self.clock() if stamp is None else stamp
        with self._lock:
            if (self._active is None or self._paused or
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
                operator_paused=paused, control_source=control_source, last_attempt=hardware.get('last_attempt')))

    def _run(self):
        episodes = {}
        try:
            with (self.path/'streams.jsonl').open('x') as output:
                while not self._quit.is_set() or not self._queue.empty():
                    try:
                        row = self._queue.get(timeout=.05)
                    except queue.Empty:
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
                            'home_requested', 'home_ready', 'home_completed', 'home_cancelled',
                            'home_button_released', 'home_button_released_or_expired')
                    if row['kind'] == 'episode_end':
                        summary.update(ended_monotonic_s=row['host_monotonic_s'],
                            disposition=row['disposition'], complete=True)
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
