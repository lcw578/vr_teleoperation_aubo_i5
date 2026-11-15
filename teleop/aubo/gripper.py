"""Vehicle RS-485 gripper: rising-edge Trigger toggle, one serial owner."""
import ast
from collections import deque
import copy
import importlib.util
import math
from pathlib import Path
import threading
import time

VEHICLE_DRIVER_ROOT = '/home/xc/code/test/ros2/robot_ws/src/gripper_driver_v2'
VEHICLE_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_575A003863-if00'


def load_vehicle_driver(root, port):
    """Read current vehicle node defaults without importing/starting ROS."""
    root = Path(root).expanduser().resolve(strict=True)
    package = root / 'gripper_driver_v2'
    defaults = {}
    for node in ast.walk(ast.parse((package / 'gripper_control_node.py').read_text())):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter' and len(node.args) >= 2
                and isinstance(node.args[0], ast.Constant)):
            try:
                defaults[node.args[0].value] = ast.literal_eval(node.args[1])
            except (ValueError, TypeError):
                pass
    keys = ('motor_id', 'bitrate', 'open_position', 'close_position', 'position_min', 'position_max')
    if any(type(defaults.get(k)) is not int for k in keys):
        raise ValueError('Missing integer vehicle gripper parameters')
    if (defaults['motor_id'] != 1 or defaults['bitrate'] <= 0
            or not defaults['position_min'] < defaults['open_position'] < defaults['close_position'] < defaults['position_max']):
        raise ValueError('Invalid vehicle gripper parameters')
    spec = importlib.util.spec_from_file_location('_vehicle_gripper_485', package / 'gripper_485_driver.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    driver = module.Gripper485Driver(str(port), defaults['bitrate'])
    driver.POSITION_OPEN = defaults['open_position']
    driver.POSITION_CLOSE = defaults['close_position']
    return driver, dict(defaults, driver_source=str(package / 'gripper_485_driver.py'), port=str(port))


class TriggerToggle:
    """Fresh ordered right-index presses only; release/hold never toggles."""
    def __init__(self, max_age_s=.2):
        self.max_age_s = max_age_s
        self.generation = 0
        self.reset()

    def reset(self):
        self.generation += 1
        self.epoch = self.source = self.received = self.offset = None
        self.seq = -1
        self.last_pressed = None

    def disarm(self):
        # Keep ordering and clock alignment across a pause. Otherwise the
        # first buffered frame after a timeout can establish a new offset
        # and make an old press sequence appear fresh.
        self.generation += 1
        self.received = None
        self.last_pressed = None

    def feed(self, frame, stamp):
        source = float(frame['t_client']) / 1000.
        if not math.isfinite(source) or not math.isfinite(stamp):
            self.reset()
            return False
        epoch, seq = frame['epoch'], frame['seq']
        if epoch != self.epoch:
            self.reset()
            self.epoch = epoch
        if seq <= self.seq or (self.source is not None and source < self.source):
            return False
        self.offset = min(stamp-source, self.offset if self.offset is not None else stamp-source)
        if not 0 <= stamp-source-self.offset <= self.max_age_s:
            self.last_pressed = None
            self.generation += 1
            return False
        if ((self.source is not None and source-self.source > self.max_age_s)
                or (self.received is not None and stamp-self.received > self.max_age_s)):
            self.last_pressed = None
            self.generation += 1
        button = frame['controllers']['right']['buttons'][0]
        if type(button.get('p')) is not bool:
            self.reset()
            return False
        pressed = button['p']
        edge = self.last_pressed is False and pressed
        self.seq, self.source, self.received, self.last_pressed = seq, source, stamp, pressed
        return edge


class GripperWorker:
    """Thread owns all driver I/O; receiver only publishes a bounded latest target.

    No enable/target writes until a new Trigger press. Disconnect/exit closes
    the host port and preserves motor torque/target, including a held object.
    Input loss cancels unsubmitted commands; an issued target is not reversed.
    """
    def __init__(self, driver, settings, *, recorder=None, clock=time.monotonic):
        self.driver, self.settings, self.recorder, self.clock = driver, settings, recorder, clock
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread = None
        self._toggle = TriggerToggle()
        self._pending = None
        self._active = None
        self._revision = 0
        self._desired_closed = None
        self._state = dict(enabled=True, state='starting', fault=None, position=None,
            velocity_rpm=None, current_raw=None, torque_enable=None, hardware_error=None,
            desired_closed=None, commanded_closed=None, input_ready=False,
            input_last_received_s=None, feedback_received_s=None, target=None,
            control='right_trigger_press_toggle', exit_policy='preserve_torque_and_target',
            settings=copy.deepcopy(settings))
        self.events = deque(maxlen=128)

    def _event(self, event, **values):
        row = dict(event=event, host_monotonic_s=self.clock(), **values)
        with self._lock:
            self.events.append(row)
        if self.recorder:
            payload = dict(row)
            stamp = payload.pop('host_monotonic_s')
            self.recorder.record('gripper_event', stamp=stamp, **payload)

    def status(self):
        with self._lock:
            return copy.deepcopy(self._state)

    def invalidate_input(self):
        with self._lock:
            self._toggle.disarm()
            self._pending = None
            self._revision += 1
            self._desired_closed = self._state['commanded_closed']
            self._state.update(input_ready=False, desired_closed=self._desired_closed)

    def feed(self, frame, stamp, *, allowed=True):
        with self._lock:
            if (not allowed or frame.get('input_active') is not True
                    or self._stop.is_set() or self._state['state'] in ('starting', 'fault', 'closed')):
                self.invalidate_input()
                return False
            generation = self._toggle.generation
            edge = self._toggle.feed(frame, stamp)
            if generation != self._toggle.generation:
                self._pending = None
                self._revision += 1
                self._desired_closed = self._state['commanded_closed']
                self._state['desired_closed'] = self._desired_closed
            self._state.update(input_last_received_s=self._toggle.received,
                               input_ready=self._toggle.last_pressed is not None)
            if not edge:
                return False
            if self._desired_closed is None:
                return False
            self._desired_closed = not self._desired_closed
            self._revision += 1
            self._pending = (self._desired_closed, self._revision)
            self._state['desired_closed'] = self._desired_closed
            self._event('trigger_toggle', closed=self._desired_closed, seq=frame['seq'], epoch=frame['epoch'])
            return True

    def _fresh(self):
        stamp = self._toggle.received
        return not self._stop.is_set() and stamp is not None and 0 <= self.clock()-stamp <= .2

    @staticmethod
    def _read(reader, name):
        ok, value, message = reader()
        if not ok or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise RuntimeError(f'{name}: {message}')
        return value

    def _sample(self):
        started = self.clock()
        state = dict(position=self._read(self.driver.read_position, 'position'),
            velocity_rpm=self._read(self.driver.read_velocity, 'velocity'),
            current_raw=self._read(self.driver.read_current, 'current'),
            torque_enable=self._read(self.driver.read_torque_enable, 'torque'),
            hardware_error=self._read(self.driver.read_hardware_error, 'hardware_error'),
            mode=self._read(self.driver.read_operation_mode, 'mode'))
        if (self.clock()-started > .25 or state['mode'] != 3 or state['hardware_error'] != 0
                or state['torque_enable'] not in (0, 1)
                or not self.settings['position_min'] < state['position'] < self.settings['position_max']):
            raise RuntimeError(f'Invalid or stale gripper feedback: {state}')
        with self._lock:
            self._state.update(state, feedback_received_s=self.clock())
        return state

    def _prepare_position_mode(self):
        mode = self._read(self.driver.read_operation_mode, 'mode')
        torque = self._read(self.driver.read_torque_enable, 'torque')
        velocity = self._read(self.driver.read_velocity, 'velocity')
        hardware_error = self._read(self.driver.read_hardware_error, 'hardware_error')
        if hardware_error != 0:
            raise RuntimeError(f'Gripper hardware error before startup: {hardware_error}')
        if mode == 3:
            return
        if mode != 1 or torque != 0 or abs(velocity) > .05:
            raise RuntimeError(
                f'Cannot select position mode safely: mode={mode}, '
                f'torque_enable={torque}, velocity_rpm={velocity}')
        self.driver.write_register(0x0021, 3, 1)
        confirmed = self._read(self.driver.read_operation_mode, 'mode')
        if confirmed != 3:
            raise RuntimeError(f'Position mode readback failed: mode={confirmed}')

    def _send(self, closed, revision, sample):
        target = self.driver.POSITION_CLOSE if closed else self.driver.POSITION_OPEN
        def permitted():
            with self._lock:
                return self._fresh() and revision == self._revision
        if not permitted():
            return
        if sample['torque_enable'] == 0:
            if abs(sample['velocity_rpm']) > .05:
                raise RuntimeError('Cannot enable a moving disabled gripper')
            hold = sample['position']
            self.driver.write_register(532, hold, 4, signed=True)
            if self.driver.read_register(532, 4, signed=True) != hold:
                raise RuntimeError('Pre-enable goal synchronization failed')
            if not permitted():
                return
            # Mode 3 was verified above. Use the vehicle driver's validated
            # one-byte exchange, with no mode writes or automatic retries.
            self.driver.write_register(512, 1, 1)
            sample = self._sample()
            if sample['torque_enable'] != 1 or abs(sample['position']-hold) > 5:
                raise RuntimeError('Enable hold not confirmed')
        # This is the dispatch boundary. An already submitted target is never
        # reversed by disconnect; receiver threads never wait for serial I/O.
        if not permitted():
            return
        ok, _, message = (self.driver.close_gripper() if closed else self.driver.open_gripper())
        if not ok:
            raise RuntimeError('Gripper target unconfirmed: '+message)
        if self.driver.read_register(532, 4, signed=True) != target:
            raise RuntimeError('Gripper target readback mismatch')
        self._active = dict(target=target, closed=closed, started=self.clock(), recent=[])
        with self._lock:
            self._state.update(state='moving', target=target, commanded_closed=closed)
            if self._pending is None:
                self._desired_closed = closed
                self._state['desired_closed'] = closed
        self._event('command_sent', closed=closed, target=target)

    def _cycle(self):
        sample = self._sample()
        with self._lock:
            if not self._fresh():
                self.invalidate_input()
            pending, self._pending = self._pending, None
        if pending:
            self._send(*pending, sample)
            return
        if self._active:
            active = self._active
            if sample['torque_enable'] != 1:
                raise RuntimeError('Gripper torque lost during motion')
            near = abs(sample['position']-active['target']) <= 5 and abs(sample['velocity_rpm']) <= .05
            active['recent'] = (active['recent']+[sample['position']])[-3:] if near else []
            if len(active['recent']) == 3 and max(active['recent'])-min(active['recent']) <= 2:
                self._event('target_reached', closed=active['closed'], target=active['target'], position=sample['position'])
                self._active = None
                with self._lock:
                    self._state['state'] = 'ready'
            elif self.clock()-active['started'] > 5.:
                raise TimeoutError('Gripper did not reach a stationary target within 5 seconds')

    def _run(self):
        connected = False
        try:
            import fcntl
            import subprocess
            import termios
            occupancy = subprocess.run(['fuser', '-s', self.driver.port], capture_output=True)
            if occupancy.returncode != 1:
                raise RuntimeError('Gripper serial port occupied or ownership unknown')
            if not self.driver.connect():
                raise RuntimeError('Cannot open gripper serial port')
            connected = True
            fcntl.ioctl(self.driver.ser.fileno(), termios.TIOCEXCL)
            self._prepare_position_mode()
            sample = self._sample()
            closed = abs(sample['position']-self.driver.POSITION_CLOSE) < abs(sample['position']-self.driver.POSITION_OPEN)
            with self._lock:
                self._desired_closed = closed
                self._state.update(state='ready', desired_closed=closed, commanded_closed=closed)
            self._event('connected', initial_closed_estimate=closed, position=sample['position'])
            while not self._stop.wait(.05):
                self._cycle()
        except Exception as exc:
            with self._lock:
                self._state.update(state='fault', fault=str(exc))
                self.invalidate_input()
            self._event('fault', error=str(exc))
        finally:
            if connected:
                self.driver.close()  # Never disconnect(): preserves torque and held load.
            with self._lock:
                if self._state['state'] != 'fault':
                    self._state['state'] = 'closed'

    def start(self):
        if self._thread is not None:
            raise RuntimeError('Gripper worker already started')
        self._thread = threading.Thread(target=self._run, name='gripper-rs485', daemon=True)
        self._thread.start()
        return self

    def close(self):
        self._stop.set()
        self.invalidate_input()
        if self._thread:
            self._thread.join(timeout=3.)
            if self._thread.is_alive():
                raise RuntimeError('Gripper serial worker did not close')
