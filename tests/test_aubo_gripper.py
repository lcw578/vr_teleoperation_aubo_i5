"""Toggle/worker/runtime regression with fake serial driver only."""
from pathlib import Path
import sys
import json
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_aubo_follow import Clock
from test_aubo_recording import frame as recording_frame
from test_aubo_six_axis import SixClient
from teleop.aubo.gripper import GripperWorker, TriggerToggle
from teleop.aubo.follow import J6FollowWorker
from teleop.aubo.quest_follow import J6QuestRuntime
from teleop.aubo.quest_preview import make_shadow
from teleop.aubo.recording import RawRecorder

SETTINGS = dict(position_min=0, position_max=3800)

def frame(*args, **kwargs):
    return dict(recording_frame(*args, **kwargs), input_active=True)

class Driver:
    POSITION_OPEN = 834
    POSITION_CLOSE = 3500
    def __init__(self):
        self.position = 834
        self.goal = 0
        self.torque = 0
        self.mode = 3
        self.writes = []
        self.hardware_error = 0
    def read_position(self): return True, self.position, 'ok'
    def read_velocity(self): return True, 0., 'ok'
    def read_current(self): return True, 0, 'ok'
    def read_torque_enable(self): return True, self.torque, 'ok'
    def read_hardware_error(self): return True, self.hardware_error, 'ok'
    def read_operation_mode(self): return True, self.mode, 'ok'
    def read_register(self, *args, **kwargs): return self.goal
    def write_register(self, addr, value, width, **kwargs):
        self.writes.append((addr, value, width))
        if addr == 532: self.goal = value
        elif addr == 512: self.torque = value
        elif addr == 0x0021: self.mode = value
    def close_gripper(self):
        self.writes.append(('close',))
        self.position = self.goal = self.POSITION_CLOSE
        return True, self.position, 'ok'
    def open_gripper(self):
        self.writes.append(('open',))
        self.position = self.goal = self.POSITION_OPEN
        return True, self.position, 'ok'
    def close(self): pass

class GripperTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.driver = Driver()
        self.gripper = GripperWorker(self.driver, SETTINGS, clock=self.clock)
        self.gripper._state.update(state='ready', commanded_closed=False)
        self.gripper._desired_closed = False
        self.seq = 0
    def feed(self, pressed=False, *, dt=.02, epoch='test', allowed=True):
        self.clock.advance(dt)
        f = frame(self.seq, self.clock(), right=(0,) if pressed else (), epoch=epoch)
        self.seq += 1
        return self.gripper.feed(f, self.clock(), allowed=allowed)
    def test_press_toggles_release_and_hold_do_not_and_no_grip_required(self):
        self.feed(True); self.gripper._cycle()
        self.assertEqual(self.driver.writes, []) # held at startup
        self.feed(False)
        self.assertTrue(self.feed(True))
        self.feed(False) # short press survives release before serial tick
        self.gripper._cycle()
        self.assertEqual(self.driver.writes, [(532,834,4),(512,1,1),('close',)])
        self.assertTrue(self.feed(True))
        for _ in range(3): self.feed(True)
        self.gripper._cycle()
        self.assertEqual(self.driver.writes[-1], ('open',))
        self.assertEqual(sum(w == ('open',) for w in self.driver.writes), 1)

    def test_startup_selects_position_mode_only_when_torque_is_off_and_stationary(self):
        self.driver.mode = 1
        self.gripper._prepare_position_mode()
        self.assertEqual(self.driver.mode, 3)
        self.assertEqual(self.driver.torque, 0)
        self.assertEqual(self.driver.writes, [(0x0021, 3, 1)])

    def test_startup_refuses_mode_change_when_torque_is_enabled(self):
        self.driver.mode = 1
        self.driver.torque = 1
        with self.assertRaisesRegex(RuntimeError, 'Cannot select position mode safely'):
            self.gripper._prepare_position_mode()
        self.assertEqual(self.driver.writes, [])
    def test_reconnect_and_timeout_held_press_need_release(self):
        self.feed(False)
        self.feed(True, epoch='new')
        self.assertIsNone(self.gripper._pending)
        self.feed(False, epoch='new')
        self.feed(True, dt=.3, epoch='new')
        self.assertIsNone(self.gripper._pending)
        self.feed(False, epoch='new'); self.assertTrue(self.feed(True, epoch='new'))
    def test_stale_duplicate_reordered_frames_cannot_toggle(self):
        detector = TriggerToggle()
        detector.feed(frame(0,10.),10.)
        detector.feed(frame(1,10.01),10.01)
        self.assertFalse(detector.feed(frame(1,10.02,right=(0,)),10.02))
        self.assertFalse(detector.feed(frame(2,9.9,right=(0,)),10.03))
        self.assertFalse(detector.feed(frame(3,10.02,right=(0,)),10.5))
    def test_expired_pending_command_is_discarded_without_enable(self):
        self.feed(False); self.feed(True)
        self.clock.advance(.3); self.gripper._cycle()
        self.assertEqual(self.driver.writes, [])

    def test_timeout_does_not_rebase_buffered_button_sequence_as_fresh(self):
        self.feed(False)
        source = self.clock()
        self.clock.advance(2.)
        for seq, pressed in enumerate([False, True, False, True], start=self.seq):
            # Repeated watchdog invalidation used to erase the source offset.
            self.gripper._cycle()
            old = frame(seq, source + (seq+1)*.01, right=(0,) if pressed else ())
            self.assertFalse(self.gripper.feed(old, self.clock()))
            self.clock.advance(.01)
        self.gripper._cycle()
        self.assertEqual(self.driver.writes, [])
        self.seq = seq+1
        self.feed(True) # first current held frame is not a new press
        self.gripper._cycle()
        self.assertEqual(self.driver.writes, [])
        self.feed(False)
        self.assertTrue(self.feed(True))
        self.gripper._cycle()
        self.assertEqual(self.driver.writes[-1], ('close',))

    def test_suspend_cancels_unsubmitted_press_and_requires_new_active_release(self):
        self.feed(False); self.feed(True)
        suspended = dict(frame(self.seq, self.clock()), input_active=False)
        self.seq += 1
        self.assertFalse(self.gripper.feed(suspended, self.clock()))
        self.gripper._cycle()
        self.assertEqual(self.driver.writes, [])
        self.feed(True) # held on resume must not activate
        self.gripper._cycle()
        self.assertEqual(self.driver.writes, [])
        self.feed(False)
        self.assertTrue(self.feed(True))

    def test_old_browser_without_focus_gate_cannot_command_gripper(self):
        for seq, pressed in enumerate([False, True, False, True]):
            self.clock.advance(.02)
            old = recording_frame(seq, self.clock(), right=(0,) if pressed else ())
            self.assertFalse(self.gripper.feed(old, self.clock()))
            self.gripper._cycle()
        self.assertEqual(self.driver.writes, [])
    def test_reconnect_discards_previous_connection_pending_press(self):
        self.feed(False); self.feed(True)
        self.feed(True, epoch='reconnect')
        self.gripper._cycle()
        self.assertEqual(self.driver.writes, [])
        self.feed(False, epoch='reconnect'); self.feed(True, epoch='reconnect')
        self.gripper._cycle()
        self.assertEqual(self.driver.writes[-1], ('close',))
    def test_fault_or_pause_does_not_restore_held_press(self):
        self.feed(False); self.feed(True, allowed=False)
        self.feed(True); self.gripper._cycle()
        self.assertEqual(self.driver.writes, [])
        self.feed(False); self.assertTrue(self.feed(True))
    def test_serial_io_does_not_block_receiver(self):
        entered, release = threading.Event(), threading.Event()
        original = self.driver.close_gripper
        def delayed():
            entered.set(); release.wait(2.); return original()
        self.driver.close_gripper = delayed
        self.feed(False); self.feed(True)
        thread = threading.Thread(target=self.gripper._cycle)
        thread.start()
        self.assertTrue(entered.wait(1.))
        try:
            self.feed(False)
            self.assertTrue(self.feed(True))
        finally:
            release.set(); thread.join(2.)
        self.assertFalse(thread.is_alive())
        self.assertFalse(self.gripper.status()['desired_closed'])
    def test_fault_stops_commands_and_close_preserves_torque(self):
        self.driver.torque = 1
        self.gripper.close()
        self.feed(False); self.feed(True)
        self.assertEqual(self.driver.writes, [])
        self.assertEqual(self.driver.torque, 1)
    def test_stationary_arrival_and_torque_loss(self):
        self.feed(False); self.feed(True); self.gripper._cycle()
        for _ in range(3):
            self.feed(True); self.gripper._cycle()
        self.assertEqual(self.gripper.status()['state'], 'ready')
        self.feed(False); self.feed(True); self.gripper._cycle()
        self.driver.torque = 0
        with self.assertRaisesRegex(RuntimeError,'torque lost'):self.gripper._cycle()
    def test_runtime_preserves_short_trigger_event_with_right_grip_released(self):
        client = SixClient(self.clock)
        worker = J6FollowWorker(client,[0.]*6,j6_range_rad=(-3.,3.),
                               joint_ranges_rad=[[-3.,3.]]*6,clock=self.clock)
        shadow = make_shadow([0.]*6,[[-3.,3.]]*6)
        runtime = J6QuestRuntime(shadow,worker,gripper=self.gripper,control_dt=.005,clock=self.clock)
        for seq,pressed in enumerate([False,True,False]):
            self.clock.advance(.02)
            self.assertTrue(runtime.submit_xr_frame(frame(seq,self.clock(),right=(0,) if pressed else ())))
        self.gripper._cycle()
        self.assertEqual(self.driver.writes[-1], ('close',))
        self.assertEqual(client.sent, [])
        runtime.submit_xr_frame({'invalid':True})
        self.assertFalse(self.gripper.status()['input_ready'])
        self.assertTrue(runtime.status()['gripper_installed'])

    def test_runtime_discards_invisible_frames_before_gripper_or_operator_buttons(self):
        client = SixClient(self.clock)
        worker = J6FollowWorker(client,[0.]*6,j6_range_rad=(-3.,3.),
                               joint_ranges_rad=[[-3.,3.]]*6,clock=self.clock)
        shadow = make_shadow([0.]*6,[[-3.,3.]]*6)
        runtime = J6QuestRuntime(shadow,worker,gripper=self.gripper,control_dt=.005,clock=self.clock)
        runtime.submit_xr_frame(frame(0,self.clock()))
        for seq in range(1,5):
            self.clock.advance(.02)
            invisible = dict(frame(seq,self.clock(),right=(0,1)),input_active=False)
            self.assertFalse(runtime.submit_xr_frame(invisible))
            self.gripper._cycle()
        self.assertEqual(self.driver.writes, [])
        self.assertEqual(client.sent, [])
        self.assertFalse(self.gripper.status()['input_ready'])
        self.clock.advance(.02)
        self.assertTrue(runtime.submit_xr_frame(frame(5,self.clock(),right=(0,))))
        self.gripper._cycle()
        self.assertEqual(self.driver.writes, [])
        for seq, pressed in [(6,False),(7,True)]:
            self.clock.advance(.02)
            runtime.submit_xr_frame(frame(seq,self.clock(),right=(0,) if pressed else ()))
        self.gripper._cycle()
        self.assertEqual(self.driver.writes[-1], ('close',))

    def record_real_events(self, action):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'capture'
            recorder = RawRecorder(path, task='synthetic gripper recording regression', clock=self.clock)
            self.gripper.recorder = recorder
            try:
                action(recorder)
            finally:
                recorder.close()
            self.assertIsNone(recorder.error)
            return [json.loads(line) for line in (path/'streams.jsonl').read_text().splitlines()]

    def run_fake_serial_owner(self):
        self.driver.port = '/dev/synthetic-gripper-no-device-access'
        self.driver.connect = Mock(return_value=True)
        self.driver.ser = Mock()
        self.driver.close = Mock()
        self.gripper._stop.set() # exercise startup/cleanup without a polling loop
        with patch('subprocess.run', return_value=Mock(returncode=1)), patch('fcntl.ioctl'):
            self.gripper._run()
        self.driver.close.assert_called_once()

    def test_real_recorder_accepts_connection_event_before_episode(self):
        rows = self.record_real_events(lambda recorder: self.run_fake_serial_owner())
        self.assertIsNone(self.gripper.status()['fault'])
        self.assertEqual([r['event'] for r in rows], ['connected'])
        self.assertIsNone(rows[0]['episode_id'])
        self.assertEqual(rows[0]['host_monotonic_s'], self.gripper.events[0]['host_monotonic_s'])
        self.assertEqual(self.driver.writes, [])

    def test_real_recorder_accepts_toggle_command_and_arrival(self):
        def action(recorder):
            recorder.begin()
            self.feed(False); self.feed(True); self.gripper._cycle()
            for _ in range(3):
                self.feed(True); self.gripper._cycle()
            recorder.end(disposition='saved')
        rows = self.record_real_events(action)
        events = [r for r in rows if r['kind']=='gripper_event']
        self.assertEqual([r['event'] for r in events], ['trigger_toggle','command_sent','target_reached'])
        self.assertTrue(all(r['episode_id']==0 for r in events))
        self.assertEqual([r['host_monotonic_s'] for r in events],
                         [r['host_monotonic_s'] for r in self.gripper.events])

    def test_real_recorder_preserves_serial_fault_without_secondary_exception(self):
        self.driver.hardware_error = 1
        rows = self.record_real_events(lambda recorder: self.run_fake_serial_owner())
        self.assertEqual(self.gripper.status()['state'], 'fault')
        self.assertIn('Gripper hardware error', self.gripper.status()['fault'])
        self.assertEqual([r['event'] for r in rows], ['fault'])
        self.assertEqual(rows[0]['error'], self.gripper.status()['fault'])
        self.assertEqual(self.driver.writes, [])

if __name__ == '__main__': unittest.main()
