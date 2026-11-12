"""Replay a slow desktop startup without SDK connections or robot motion."""
from contextlib import ExitStack, contextmanager
from pathlib import Path
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_aubo_follow import Clock
from test_aubo_six_axis import SixClient
from teleop.aubo.follow import J6FollowWorker
from teleop.aubo.quest_preview import serve_quest
from teleop.aubo.quest_follow import main


class ControlStartupTests(unittest.TestCase):
    def exercise(self, *, ui_error=False, feedback_error=False, gripper_status_tick=False):
        clock = Clock()
        client = SixClient(clock)
        worker = J6FollowWorker(client, [0.]*6, j6_range_rad=(-3., 3.),
            joint_ranges_rad=[[-3., 3.]]*6, clock=clock, diagnostics=True)
        events = []
        viewer = SimpleNamespace(opt=SimpleNamespace(geomgroup=[0]*4),
            cam=SimpleNamespace(lookat=[0.]*3, distance=0.), is_running=lambda: False)

        @contextmanager
        def launch(*args):
            events.append('preview_initializing')
            # Field case: a 212ms host pause aged the state query by 205ms.
            clock.advance(.212)
            if ui_error:
                raise RuntimeError('injected UI failure')
            events.append('preview_ready')
            yield viewer
            events.append('preview_closed')

        receiver = Mock()
        receiver.start.side_effect = lambda: events.append('receiver_started')
        receiver.close.side_effect = lambda: events.append('receiver_closed')
        server = SimpleNamespace(started=True, should_exit=False, run=Mock())
        thread = Mock()
        thread.start.side_effect = lambda: events.append('relay_started')
        thread.is_alive.return_value = False
        relay = SimpleNamespace(CAMERA_SPECS={}, CAMERA_READERS={},
            app=SimpleNamespace(state=SimpleNamespace(), add_api_route=Mock()))
        fake_mujoco = ModuleType('mujoco')
        fake_mujoco.viewer = SimpleNamespace(launch_passive=launch)
        fake_mujoco.MjData = lambda model: object()
        modules = {'mujoco': fake_mujoco,
            'uvicorn': SimpleNamespace(Config=Mock(), Server=lambda config: server),
            'fastapi.responses': SimpleNamespace(HTMLResponse=Mock(), JSONResponse=Mock()),
            'vr_teleop_kit.relay': SimpleNamespace(server=relay)}
        shadow = SimpleNamespace(geometry=SimpleNamespace(model=None, data=None),
            session=SimpleNamespace(policy=SimpleNamespace(velocity_rad_s=[1.]*6)),
            core=SimpleNamespace(input_stats={'frames': 0}))
        runtime = SimpleNamespace(mode='test', motion_available=True, control_dt=.005)
        args = SimpleNamespace(rotation='world', usb=True, relay_port=8443, headless=False)
        if gripper_status_tick:
            viewer.is_running = Mock(side_effect=[True, False])
            shadow.session.cache = SimpleNamespace(latest=lambda: None)
            shadow.session.backend = SimpleNamespace(stop_requests=0)
            shadow.core.config = SimpleNamespace(r_calib=None)
            runtime.expected_reference_id = None
            runtime.step = Mock(return_value=None)
            runtime.submit_fault = Mock()
            runtime.status = lambda: dict(gripper_installed=True,
                gripper=dict(enabled=True, state='ready'),
                xr_input=dict(valid_frame_age_s=None))
            args.preview_speed_deg_s = None
            args.duration = .001

        def start_feedback():
            events.append('first_feedback')
            self.assertEqual(events[:4], ['relay_started', 'receiver_started',
                'preview_initializing', 'preview_ready'])
            self.assertIsNone(worker.cache.latest())
            self.assertEqual(client.sent, [])
            if feedback_error:
                raise RuntimeError('injected initial feedback failure')
            worker._cycle()

        expected_error = ('injected UI failure' if ui_error else
            'injected initial feedback failure' if feedback_error else None)
        with patch.dict(sys.modules, modules), \
                patch('teleop.aubo.quest_preview.QuestReceiver', return_value=receiver), \
                patch('teleop.aubo.quest_preview.threading.Thread', return_value=thread), \
                patch('teleop.aubo.j6_trial.time.monotonic', clock), ExitStack() as stack:
            if expected_error:
                with self.assertRaisesRegex(RuntimeError, expected_error):
                    serve_quest(args, shadow, runtime, stack, {},
                        refresh_feedback=Mock(), feedback_status=Mock(), on_ready=start_feedback)
            else:
                serve_quest(args, shadow, runtime, stack, {},
                    refresh_feedback=Mock(), feedback_status=Mock(), on_ready=start_feedback)
        self.assertEqual(client.sent, [])
        self.assertFalse(client.permitted)
        self.assertEqual(client.stops, 0)
        self.assertTrue(server.should_exit)
        self.assertIn('receiver_closed', events)
        return clock, worker, events

    def test_enabled_gripper_status_survives_first_preview_tick(self):
        self.exercise(gripper_status_tick=True)

    def test_first_feedback_is_fresh_after_212ms_ui_startup_without_arming(self):
        clock, worker, events = self.exercise()
        self.assertGreater(clock(), 100.2)
        self.assertFalse(worker.status()['fatal'])
        self.assertTrue(worker.status()['feedback_ready'])
        self.assertEqual(worker.status()['state'], 'idle')
        self.assertEqual(worker.cache.latest().received_monotonic_s, clock())
        self.assertEqual(events.count('first_feedback'), 1)

    def test_ui_failure_never_starts_feedback(self):
        _, worker, events = self.exercise(ui_error=True)
        self.assertNotIn('first_feedback', events)
        self.assertIsNone(worker.cache.latest())

    def test_initial_feedback_failure_aborts_startup_and_closes_ui(self):
        _, worker, events = self.exercise(feedback_error=True)
        self.assertIsNone(worker.cache.latest())
        self.assertIn('preview_closed', events)

    def exercise_gripper_main(self, *, ui_error=False, gripper_error=False):
        # Exercise the production main/on_ready wiring with no SDK or serial
        # connection. A gripper started before serve_quest would fail here.
        events = []
        client = SixClient(time.monotonic)
        client.login = Mock(return_value=dict(return_code=0))
        client.logout = Mock(return_value=dict(return_code=0))
        client.start_waypoint_feedback = Mock(return_value=dict(return_code=0, duration_s=0.))
        client.waypoint_feedback = Mock(return_value=dict(samples=[1]))
        client.prepare = Mock(return_value=dict(return_code=0, prepared=True,
            anchor_rad=[0.]*6, joint_ranges_rad=[[-3., 3.]]*6))
        gripper = Mock()
        gripper.start.side_effect = lambda: events.append('gripper_started')
        gripper.status.return_value = dict(state='fault' if gripper_error else 'ready',
            fault='injected serial failure' if gripper_error else None)

        def serve(*args, on_ready, **kwargs):
            events.append('preview_initializing')
            gripper.start.assert_not_called()
            if ui_error:
                raise RuntimeError('injected UI failure')
            events.append('preview_ready')
            on_ready()
            events.append('control_ready')

        def start_worker(worker):
            events.append('arm_feedback_started')
            self.assertEqual(events[-2], 'gripper_started')
            worker._cycle()

        readback = dict(max_velocity_rad_s=None, max_acceleration_rad_s2=None,
            source='not_read_six_axis_unmodified')
        args = ['--execute-six-axis', '--command-mode', 'follow', '--waypoint-feedback',
            '--gripper', '--usb', '--host', '127.0.0.1', '--port', '8899', '--user', 'test']
        with tempfile.TemporaryDirectory() as root, \
                patch('teleop.aubo.quest_follow.ROOT', Path(root)), \
                patch('teleop.aubo.quest_follow.load_follow_native',
                   return_value=SimpleNamespace(J6Follower=lambda **kwargs: client)), \
                patch('teleop.aubo.quest_follow.getpass.getpass', return_value='test'), \
                patch('teleop.aubo.quest_follow.check_relay_port'), \
                patch('teleop.aubo.quest_follow.validate_prepare_profile', return_value=readback), \
                patch('teleop.aubo.quest_follow.load_home', return_value=None), \
                patch('teleop.aubo.quest_follow.load_vehicle_driver', return_value=(Mock(), {})), \
                patch('teleop.aubo.quest_follow.GripperWorker', return_value=gripper), \
                patch('teleop.aubo.quest_follow.serve_quest', side_effect=serve), \
                patch.object(J6FollowWorker, 'start', start_worker), \
                patch.object(J6FollowWorker, 'close'):
            error = ('injected UI failure' if ui_error else
                'injected serial failure' if gripper_error else None)
            if error:
                with self.assertRaisesRegex(RuntimeError, error):
                    main(args)
            else:
                self.assertEqual(main(args), 0)
        gripper.close.assert_called()
        self.assertEqual(client.sent, [])
        self.assertFalse(client.permitted)
        return events

    def test_main_starts_gripper_after_preview_before_arm_feedback(self):
        self.assertEqual(self.exercise_gripper_main(), ['preview_initializing', 'preview_ready',
            'gripper_started', 'arm_feedback_started', 'control_ready'])

    def test_main_ui_failure_never_opens_gripper(self):
        self.assertEqual(self.exercise_gripper_main(ui_error=True), ['preview_initializing'])

    def test_main_gripper_failure_aborts_before_arm_feedback(self):
        self.assertEqual(self.exercise_gripper_main(gripper_error=True),
            ['preview_initializing', 'preview_ready', 'gripper_started'])


if __name__ == '__main__':
    unittest.main()
