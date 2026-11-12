"""Opt-in unloaded two-leg placement. No trajectory replay or device I/O."""
import hashlib
import json
import math
from pathlib import Path

from .feedback import joints
from .home import HOME_TOLERANCE_RAD
from .offline_session import Candidate


EXPECTED_STOPS = ('placement_requested', 'placement_segment_completed',
                  'placement_complete', 'placement_cancelled')


def load_placement(path, home, ranges):
    """Bind the measured unloaded endpoint to this exact home reference."""
    path = Path(path).expanduser().resolve()
    value = json.loads(path.read_text())
    if (value.get('schema') != 'aubo_placement_reference_v1' or
            value.get('role') != 'basket_above' or
            value.get('joint_source') != 'sdk_waypoint_push' or
            value.get('joint_names') != [f'joint_{n}' for n in range(1, 7)] or
            value.get('tool') != 'bare_flange' or value.get('gripper_installed') is not False):
        raise ValueError('Placement requires a measured bare-flange basket reference')
    q = joints(value['q_rad'])
    if len(ranges) != 6 or any(not lo <= angle <= hi for angle, (lo, hi) in zip(q, ranges)):
        raise ValueError('Basket outside current controller joint ranges')
    linked = value['home_reference']
    if (not home or joints(linked['q_rad']) != tuple(home['q_rad']) or
            linked['sha256'] != hashlib.sha256(Path(home['path']).read_bytes()).hexdigest()):
        raise ValueError('Basket reference was taught with a different home')
    return dict(path=str(path), q_rad=q, home_q_rad=tuple(home['q_rad']),
                route='current_to_left_y_home_then_basket', tool='bare_flange',
                route_validated=False, commissioning=True)


class Placement:
    """Runs on the existing runtime/SDK owner, never on a second connection."""
    def __init__(self, runtime, reference):
        self.runtime = runtime
        self.reference = reference
        self.status = dict(enabled=reference is not None,
            state='idle' if reference else 'disabled', segment=None,
            reference_path=reference['path'] if reference else None,
            route_validated=False, tool='bare_flange', error=None,
            requested_monotonic_s=None, completed_monotonic_s=None,
            max_error_deg=None, control_button='left_x', requires_right_grip=False)
        self.session = -1000000
        self.cancel_reason = None
        self.last_input_s = None
        self.offered = False
        self.started_s = None
        self.quiet_start = self.quiet_stamp = None
        self.next_observation_s = 0.

    def busy(self):
        return self.status['state'] in ('waiting_for_stop', 'ready', 'moving', 'stopping')

    def event(self):
        if self.runtime.recorder:
            self.runtime.recorder.record('placement', **self.status)

    def cancel(self, reason, *, failed=False):
        if not self.busy():
            return
        self.status.update(state='failed' if failed else 'cancelled', error=str(reason))
        self.offered = False
        self.cancel_reason = None
        self.runtime.submit_fault('placement_cancelled')
        self.runtime.shadow.reject_input('placement_cancelled')
        self.runtime.last_operator_message = '自动搬运已停止：'+str(reason)+'；不会自动续跑'
        self.event()

    def request(self, stamp):
        r = self.runtime
        if self.busy():
            self.cancel('left_x_pressed_again')
            return
        hw = r.worker.status()
        if (not self.reference or not r.worker.six_axis or r.worker.command_mode != 'follow' or
                r._home_busy() or r.operator_paused or r.software_reset_pending or
                r._episode_transition is not None or r.session_finished or r.session_finish_pending or
                hw['fatal'] or hw['stop_result_unknown'] or r._last_grip_state is not False or
                self.last_input_s is None or not 0 <= r.clock()-self.last_input_s <= .2):
            r.last_operator_message = '自动搬运未启动：请松开 Grip、保持手柄在线，并等待其他操作结束'
            return
        self.cancel_reason = None
        self.status.update(state='waiting_for_stop', segment='transfer', error=None,
            requested_monotonic_s=stamp, completed_monotonic_s=None, max_error_deg=None)
        self.started_s = r.clock()
        self.offered = False
        r.submit_fault('placement_requested')
        r.shadow.reject_input('placement_requested')
        r._pause_recording_for_clutch('placement', stamp)
        r.last_operator_message = '自动搬运：等待停稳后回左 Y 中转位，再到篮筐点；再按左 X 取消'
        self.event()

    def target(self):
        return self.reference['home_q_rad' if self.status['segment'] == 'transfer' else 'q_rad']

    def error(self, sample):
        error = max(abs(q-g) for q, g in zip(sample.q_rad, self.target()))
        self.status['max_error_deg'] = math.degrees(error)
        return error

    def _next_leg_or_complete(self):
        r = self.runtime
        if self.status['segment'] == 'transfer':
            self.status.update(state='waiting_for_stop', segment='basket')
            self.offered = False
            self.started_s = r.clock()
            r.last_operator_message = '已到左 Y 中转位并停稳；接下来直接前往篮筐点'
        else:
            self.status.update(state='complete', completed_monotonic_s=r.clock())
            self.offered = False
            r.shadow.reject_input('placement_complete')
            r.last_operator_message = '已到篮筐点并停稳；松开再按 Grip 可以重新遥操'
        self.event()

    def advance(self):
        if not self.busy():
            return
        r = self.runtime
        hw = r.worker.status()
        if self.cancel_reason:
            self.cancel(self.cancel_reason)
            return
        if (hw['fatal'] or hw['stop_result_unknown'] or r.session_finished or r.session_finish_pending):
            self.cancel('hardware_fault_or_session_finished', failed=True)
            return
        if (self.last_input_s is None or
                not 0 <= r.clock()-self.last_input_s <= r.worker.input_max_age_s):
            self.cancel('input_timeout')
            return
        if r.clock()-self.started_s > 180.:
            self.cancel('placement_timeout', failed=True)
            return
        self.observe(hw)
        phase = self.status['state']
        if phase in ('waiting_for_stop', 'stopping'):
            sample = r._stationary_feedback()
            if sample is None or (phase == 'stopping' and not hw['stop_confirmed']):
                return
            error = self.error(sample)
            if phase == 'stopping' and error > HOME_TOLERANCE_RAD:
                self.cancel('outside_tolerance_after_stop', failed=True)
            elif error <= HOME_TOLERANCE_RAD:
                self._next_leg_or_complete()
            else:
                self.session -= 1
                self.offered = False
                self.quiet_start = self.quiet_stamp = None
                self.status['state'] = 'ready'
                self.event()
        elif phase == 'ready':
            if self.offered and hw['state'] == 'active':
                self.status['state'] = 'moving'
                self.event()
            elif self.offered and hw['state'] in ('stop_requested', 'stopping'):
                self.cancel(hw.get('fault') or 'placement_start_interrupted')
        elif phase == 'moving':
            if hw['state'] != 'active':
                self.cancel(hw.get('fault') or 'placement_interrupted')
                return
            try:
                sample = r.worker.measured_feedback()
            except (RuntimeError, ValueError, KeyError, TypeError, OverflowError):
                self.cancel('placement_feedback_unavailable', failed=True)
                return
            if self.error(sample) <= HOME_TOLERANCE_RAD:
                if self.quiet_start is None:
                    self.quiet_start = sample.received_monotonic_s
                if (self.quiet_stamp != sample.received_monotonic_s and
                        sample.received_monotonic_s-self.quiet_start >= .15):
                    self.status['state'] = 'stopping'
                    r.submit_fault('placement_segment_completed')
                    r.shadow.reject_input('placement_segment_completed')
                    self.event()
                self.quiet_stamp = sample.received_monotonic_s
            else:
                self.quiet_start = self.quiet_stamp = None

    def observe(self, hardware):
        r = self.runtime
        if not r.recorder or r.clock() < self.next_observation_s:
            return
        self.next_observation_s = r.clock()+1./r.recorder.fps
        sample = r.shadow.session.cache.latest()
        r.recorder.record('placement_observation', control_source='placement',
            segment=self.status['segment'], phase=self.status['state'],
            measured_q_rad=list(sample.q_rad) if sample and sample.valid else None,
            feedback_received_monotonic_s=sample.received_monotonic_s if sample else None,
            hardware_state=hardware['state'])

    def motion_step(self):
        r = self.runtime
        r.watchdog_tick()
        with r._operator_lock:
            if (not self.busy() or self.cancel_reason or r._operator_barrier or
                    self.status['state'] not in ('ready', 'moving')):
                return None
            candidate = Candidate(tuple(self.target()), r.clock(), self.session)
            if r.worker.offer(candidate, control_source='placement'):
                self.offered = True
                return candidate
        return None
