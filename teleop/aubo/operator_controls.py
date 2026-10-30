"""Edge events are independent of the latest-only motion slot; no SDK calls."""
from collections import deque
import math
import threading


class OperatorButtons:
    STICK_DEADZONE = .35
    STICK_PRESS_THRESHOLD = .7

    def __init__(self, *, capacity=64):
        self._lock = threading.Lock()
        self._pending = deque()
        self.capacity = capacity
        self._epoch = None
        self._seq = -1
        self._source = None
        self._offset = None
        self._received = None
        self._last = {}
        self._stick_armed = False
        self._home_held = False
        self._home_received = None
        self.error = None

    def home_hold_active(self, stamp, *, max_age_s=.2):
        """A home permit requires a fresh, continuously held left Y."""
        with self._lock:
            return (self._home_held and self._home_received is not None and
                    0 <= stamp-self._home_received <= max_age_s)

    def request(self, command, stamp):
        if command not in ('pause', 'toggle_episode', 'restart_episode', 'restart_loop',
                           'exit_loop', 'exit_record', 'invalidate',
                           'software_reset', 'return_home', 'mark', 'finish_session'):
            raise ValueError('Unknown operator command')
        with self._lock:
            return self._enqueue(command, stamp)

    def _enqueue(self, command, stamp):
        if len(self._pending) >= self.capacity:
            self.error = 'operator_event_queue_overflow'
            return False
        self._pending.append(dict(command=command, host_monotonic_s=stamp))
        return True

    def feed(self, frame, stamp, *, max_age_s=.2):
        """Only fresh, ordered frames; a reappearing held button is not a press."""
        with self._lock:
            source = float(frame['t_client']) / 1000.
            if not math.isfinite(source):
                return []
            emitted = []
            epoch, seq = frame['epoch'], frame['seq']
            if epoch != self._epoch:
                if self._home_held:
                    if self._enqueue('home_hold_end', stamp):
                        emitted.append('home_hold_end')
                self._home_held = False
                self._epoch, self._seq, self._source, self._offset = epoch, -1, None, None
                self._last.clear()
                self._stick_armed = False
            if seq <= self._seq or (self._source is not None and source < self._source):
                return []
            self._offset = min(stamp-source, self._offset if self._offset is not None else stamp-source)
            if stamp-source-self._offset > max_age_s:
                return []
            if ((self._source is not None and source-self._source > max_age_s) or
                    (self._received is not None and stamp-self._received > max_age_s)):
                if self._home_held:
                    if self._enqueue('home_hold_end', stamp):
                        emitted.append('home_hold_end')
                self._home_held = False
                self._last.clear()
                self._stick_armed = False
            self._seq, self._source = seq, source
            self._received = stamp
            left = frame.get('controllers', {}).get('left')
            home_buttons = left.get('buttons') if isinstance(left, dict) else None
            home_pressed = (home_buttons[5]['p'] if isinstance(home_buttons, list) and
                            len(home_buttons) > 5 and isinstance(home_buttons[5], dict) and
                            type(home_buttons[5].get('p')) is bool else None)
            prior = self._last.get(('left', 5))
            self._last[('left', 5)] = home_pressed
            if self._home_held and home_pressed is not True:
                self._home_held = False
                if self._enqueue('home_hold_end', stamp):
                    emitted.append('home_hold_end')
            elif home_pressed is True and prior is False:
                self._home_held = True
                if self._enqueue('home_hold_start', stamp):
                    emitted.append('home_hold_start')
            if self._home_held:
                self._home_received = stamp
            # Right stick click remains a non-motion event marker. Grip is
            # handled by the clutch/recording pause lifecycle, not as a toggle.
            for hand, index, command in (('right', 3, 'mark'),):
                key = (hand, index)
                ctrl = frame.get('controllers', {}).get(hand)
                buttons = ctrl.get('buttons') if isinstance(ctrl, dict) else None
                valid = (isinstance(buttons, list) and len(buttons) > index and
                         isinstance(buttons[index], dict) and type(buttons[index].get('p')) is bool)
                if not valid:
                    self._last[key] = None
                    continue
                pressed = buttons[index]['p']
                prior = self._last.get(key)
                self._last[key] = pressed
                if pressed and prior is False:
                    if self._enqueue(command, stamp):
                        emitted.append(command)
            # Left stick directions are edge-triggered. A neutral return is
            # required before another command can fire, including after a
            # reconnect or an input gap.
            left = frame.get('controllers', {}).get('left')
            axes = left.get('axes') if isinstance(left, dict) else None
            if (not isinstance(axes, list) or len(axes) < 2):
                self._stick_armed = False
            else:
                try:
                    # xr-standard may expose touchpad axes before the stick.
                    offset = 2 if len(axes) >= 4 else 0
                    x, y = float(axes[offset]), float(axes[offset+1])
                except (TypeError, ValueError, OverflowError):
                    x = y = float('nan')
                if not math.isfinite(x) or not math.isfinite(y):
                    self._stick_armed = False
                elif max(abs(x), abs(y)) <= self.STICK_DEADZONE:
                    self._stick_armed = True
                elif self._stick_armed:
                    command = None
                    if max(abs(x), abs(y)) >= self.STICK_PRESS_THRESHOLD:
                        if abs(x) > abs(y):
                            command = 'restart_loop' if x < 0 else 'exit_loop'
                        elif abs(y) > abs(x) and y < 0:
                            command = 'exit_record'
                        # A down or diagonal gesture is consumed but has no
                        # action; returning to neutral rearms all directions.
                        self._stick_armed = False
                    if command is not None:
                        if self._enqueue(command, stamp):
                            emitted.append(command)
            return emitted

    def drain(self):
        with self._lock:
            result = list(self._pending)
            self._pending.clear()
            return result


def xr_record(frame):
    """Keep source time and valid fields; optional left pose never gates motion."""
    controllers = {}
    for hand in ('right', 'left'):
        ctrl = frame.get('controllers', {}).get(hand)
        if not isinstance(ctrl, dict):
            continue
        clean = {}
        for field, size in (('position', 3), ('orientation', 4)):
            value = ctrl.get(field)
            if isinstance(value, list) and len(value) == size:
                try:
                    value = [float(v) for v in value]
                    if all(math.isfinite(v) for v in value):
                        clean[field] = value
                except (TypeError, ValueError, OverflowError):
                    pass
        buttons = []
        for b in ctrl.get('buttons', []) if isinstance(ctrl.get('buttons'), list) else []:
            try:
                v = float(b['v'])
                if type(b['p']) is bool and math.isfinite(v) and 0 <= v <= 1:
                    buttons.append(dict(p=b['p'], v=v))
                else:
                    break
            except (TypeError, ValueError, KeyError, OverflowError):
                break
        clean['buttons'] = buttons
        controllers[hand] = clean
    return dict(epoch=frame['epoch'], seq=frame['seq'], reference_id=frame.get('reference_id'),
                t_client_ms=float(frame['t_client']),
                controllers=controllers)
