"""One-second hardware input allowance with a fake SDK and simulated clock."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_aubo_follow import Clock
from test_aubo_six_axis import SixClient
from teleop.aubo.feedback import FeedbackCache
from teleop.aubo.follow import J6FollowWorker
from teleop.aubo.offline_session import Candidate
from teleop.aubo.quest_follow import J6QuestRuntime
from teleop.aubo.quest_preview import make_shadow


class InputTimeoutTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.client = SixClient(self.clock)
        self.q = (.18, .26, 2.37, 2.04, 1.51, .8)
        self.client.q = list(self.q)
        self.bounds = ((-3., 3.),) * 6
        self.worker = J6FollowWorker(self.client, self.q, j6_range_rad=self.bounds[5],
            joint_ranges_rad=self.bounds, clock=self.clock, follow_hz=25, diagnostics=True)
        self.shadow = make_shadow(self.q, self.bounds, input_timeout_s=self.worker.input_max_age_s)
        self.shadow.clock = self.shadow.session.clock = self.clock
        self.shadow.session.cache = FeedbackCache(self.clock)
        self.runtime = J6QuestRuntime(self.shadow, self.worker, control_dt=.005, clock=self.clock)
        self.patch = patch('vr_teleop_kit.core.quest_teleop.time.monotonic', self.clock)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.addCleanup(self.runtime.close)
        self.seq = 0
        self.step(grip=False)
        self.clock.advance(.005)
        self.step(grip=True)
        self.last_input = self.clock()
        self.assertEqual(self.worker.status()['state'], 'active')

    def frame(self, grip, *, source_delay_s=0.):
        self.seq += 1
        return dict(type='xr_frame', epoch='isolated-input-timeout-test', seq=self.seq,
            t_client=(self.clock()-source_delay_s)*1000,
            controllers=dict(right=dict(position=[0., 0., 0.], orientation=[0., 0., 0., 1.],
                buttons=[dict(p=False, v=0.), dict(p=grip, v=float(grip))])))

    def step(self, *, grip=None):
        self.shadow.feedback(self.client.q, received_monotonic_s=self.clock(), source='isolated_fake_sdk')
        if grip is not None:
            self.assertTrue(self.runtime.submit_xr_frame(self.frame(grip)))
        candidate = self.runtime.step()
        self.assertFalse(self.worker._stop_needed, self.worker.status()['first_stop_trigger'])
        self.worker._cycle()
        self.worker.watchdog_tick()
        return candidate

    def gap(self, duration):
        deadline = self.clock()+duration
        while self.clock() < deadline-1e-9:
            self.clock.advance(min(.005, deadline-self.clock()))
            self.step()

    def test_531ms_gap_and_fresh_held_grip_recover_without_stop(self):
        self.gap(.531)
        self.assertTrue(self.shadow.session.engaged)
        self.assertEqual(self.worker.status()['state'], 'active')
        self.clock.advance(.005)
        self.assertIsNotNone(self.step(grip=True))
        self.assertIsNone(self.worker.status()['first_stop_trigger'])
        self.assertFalse(self.shadow.session.require_release)

    def test_990ms_is_tolerated_but_over_one_second_stops(self):
        self.gap(.990)
        self.assertTrue(self.shadow.session.engaged)
        self.assertTrue(self.client.permitted)
        self.assertIsNone(self.worker.status()['first_stop_trigger'])
        # Check the request before running the fake SDK stop/confirmation.
        self.clock.advance(.015)
        self.shadow.feedback(self.client.q, received_monotonic_s=self.clock())
        self.assertIsNone(self.runtime.step())
        trigger = self.worker.status()['first_stop_trigger']
        self.assertGreater(trigger['input_age_s'], 1.)
        self.assertFalse(self.client.permitted)
        self.assertTrue(self.worker._stop_needed)
        self.assertTrue(self.shadow.session.require_release)

    def test_release_revokes_on_receiver_before_any_control_or_sdk_tick(self):
        self.gap(.15)
        before = len(self.client.sent)
        stamp = self.clock()
        # Even a delayed release must revoke immediately, before the source
        # pose gate runs; a following press cannot overwrite the stop event.
        self.runtime.submit_xr_frame(self.frame(False, source_delay_s=.35))
        self.runtime.submit_xr_frame(self.frame(True))
        trigger = self.worker.status()['first_stop_trigger']
        self.assertEqual(trigger['reason'], 'grip_released')
        self.assertEqual(trigger['host_monotonic_s'], stamp)
        self.assertEqual(self.clock(), stamp)
        self.assertFalse(self.client.permitted)
        self.assertIsNone(self.worker._wanted)
        self.assertEqual(len(self.client.sent), before)
        self.assertEqual(self.client.stops, 0)  # SDK owner has not run yet.

    def test_old_queued_pose_is_rejected_despite_one_second_gap_allowance(self):
        self.gap(.531)
        old_input = self.shadow.input_time
        self.clock.advance(.005)
        self.runtime.submit_xr_frame(self.frame(True, source_delay_s=.5))
        self.step()
        self.assertEqual(self.shadow.input_time, old_input)
        self.assertEqual(self.shadow.last_input_event['reason'], 'source_delay_exceeded')
        self.assertTrue(self.shadow.session.engaged)
        self.clock.advance(.005)
        self.assertIsNotNone(self.step(grip=True))
        self.assertIsNone(self.worker.status()['first_stop_trigger'])

    def test_independent_worker_watchdog_uses_one_second_with_fresh_candidate(self):
        # Keep candidates fresh to isolate the worker's input watchdog.
        for age in (.53, .99, 1.005):
            self.clock.now = self.last_input+age
            self.worker.offer(Candidate(self.q, self.clock(), self.shadow.session.session))
            self.worker.watchdog_tick()
            if age < 1.:
                self.assertFalse(self.worker._stop_needed)
            else:
                self.assertEqual(self.worker.status()['first_stop_trigger']['reason'], 'input_timeout')
                self.assertFalse(self.client.permitted)

    def test_hardware_layers_align_and_preview_keeps_previous_default(self):
        self.assertEqual(self.worker.status()['input_timeout_s'], 1.)
        self.assertEqual(self.shadow.session.policy.input_max_age_s, 1.)
        self.assertEqual(self.shadow.core.config.xr_frame_stale_timeout_s, 1.)
        preview = make_shadow(self.q, self.bounds)
        self.addCleanup(preview.close)
        self.assertEqual(preview.core.config.xr_frame_stale_timeout_s, .2)
        self.assertEqual(preview.session.policy.input_max_age_s, .25)


if __name__ == '__main__':
    unittest.main(verbosity=2)
