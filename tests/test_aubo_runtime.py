"""Phase-one runtime adapter tests; no SDK or hardware access."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.aubo.offline_session import SessionPolicy
from teleop.aubo.runtime import OfflineRuntime
from teleop.aubo.shadow import AuboShadowCoordinator
from teleop.aubo.input_timing import InputTiming


class Clock:
    now = 10.0
    def __call__(self):
        return self.now


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        policy = SessionPolicy((-2.94,) * 6, (2.94,) * 6, (.5,) * 6, .1, .1, .05)
        self.shadow = AuboShadowCoordinator(policy, control_dt=.005, clock=self.clock)
        self.runtime = OfflineRuntime(self.shadow, control_dt=.005, clock=self.clock)
        self.q = self.shadow.geometry.template[:6].copy()
        self.seq = 0
        self._patch = patch("vr_teleop_kit.core.quest_teleop.time.monotonic", self.clock)
        self._patch.start()
        self.addCleanup(self._patch.stop)
        self.runtime.submit_feedback(self.q, received_monotonic_s=self.clock())

    def tearDown(self):
        self.runtime.close()

    def frame(self, grip=False, *, epoch="runtime", position=(0., 0., 0.)):
        buttons = [dict(p=False, v=0.) for _ in range(6)]
        buttons[1] = dict(p=grip, v=float(grip))
        frame = {
            "type": "xr_frame", "epoch": epoch, "seq": self.seq,
            "t_client": self.clock() * 1000,
            "controllers": {"right": {
                "position": list(position), "orientation": [0., 0., 0., 1.],
                "buttons": buttons,
            }},
        }
        self.seq += 1
        return frame

    def tick(self, grip=False, **kwargs):
        self.clock.now += .005
        self.runtime.submit_feedback(self.q, received_monotonic_s=self.clock())
        self.assertTrue(self.runtime.submit_xr_frame(self.frame(grip, **kwargs),
                                                      received_monotonic_s=self.clock()))
        return self.runtime.step()

    def test_receiver_does_not_run_core_until_control_step(self):
        self.assertTrue(self.runtime.submit_xr_frame(self.frame(False), received_monotonic_s=self.clock()))
        self.assertIsNone(self.shadow.input_time)
        self.runtime.step()
        self.assertIsNotNone(self.shadow.input_time)

    def test_candidate_only_mode_never_dispatches_even_to_fake_backend(self):
        self.runtime.dispatch_enabled = False
        self.tick(False); self.tick(True)
        for _ in range(5): self.tick(True)
        self.assertTrue(self.shadow.session.engaged)
        self.assertEqual(self.shadow.session.backend.submitted, [])
        self.assertEqual(self.shadow.session.attempts, [])

    def test_latest_motion_frame_and_fault_are_not_silently_lost(self):
        self.assertTrue(self.runtime.submit_xr_frame(self.frame(False), received_monotonic_s=self.clock()))
        invalid = self.frame(True)
        invalid["controllers"] = {}
        self.assertFalse(self.runtime.submit_xr_frame(invalid, received_monotonic_s=self.clock()))
        self.runtime.step()
        self.assertEqual(self.shadow.session.fault, "invalid_xr_frame")
        self.assertIsNone(self.shadow.session.pending)

    def test_bad_frame_cannot_be_hidden_by_later_valid_frame(self):
        self.tick(False); self.tick(True)
        self.assertFalse(self.runtime.submit_xr_frame(None))
        self.runtime.submit_xr_frame(self.frame(True))
        self.runtime.step()
        self.assertFalse(self.shadow.session.engaged)
        self.assertTrue(self.shadow.session.require_release)

    def test_release_between_control_ticks_is_processed_before_new_press(self):
        self.tick(False); self.tick(True)
        self.clock.now += .005
        self.runtime.submit_feedback(self.q)
        self.runtime.submit_xr_frame(self.frame(False))
        self.runtime.submit_xr_frame(self.frame(True))
        self.runtime.step()
        self.assertFalse(self.shadow.session.engaged)
        self.clock.now += .005
        self.runtime.step()
        self.assertTrue(self.shadow.session.engaged)

    def test_reference_change_cannot_be_hidden_in_latest_slot(self):
        self.tick(False); self.tick(True)
        self.runtime.submit_xr_frame(self.frame(True, epoch="new"))
        self.runtime.submit_xr_frame(self.frame(True, epoch="new"))
        self.runtime.step()
        self.assertEqual(self.shadow.session.fault, "reference_changed")
        self.assertFalse(self.shadow.session.engaged)

    def test_network_disconnect_does_not_rearm_while_grip_stays_pressed(self):
        self.tick(False); self.tick(True)
        self.runtime.submit_fault("websocket_disconnected")
        self.runtime.step()
        self.tick(True)
        self.assertFalse(self.shadow.session.engaged)
        self.assertTrue(self.shadow.session.require_release)
        self.tick(False); self.tick(True)
        self.assertTrue(self.shadow.session.engaged)

    def test_first_candidate_waits_one_control_period(self):
        self.tick(False)
        self.tick(True)
        self.assertEqual(self.shadow.session.backend.submitted, [])
        self.clock.now += .005
        self.runtime.submit_feedback(self.q, received_monotonic_s=self.clock())
        self.runtime.submit_xr_frame(self.frame(True), received_monotonic_s=self.clock())
        self.runtime.step()
        self.assertLessEqual(len(self.shadow.session.backend.submitted), 1)

    def test_watchdog_requires_release_after_input_gap(self):
        self.tick(False)
        self.tick(True)
        self.clock.now += .11
        self.assertFalse(self.runtime.watchdog_tick())
        self.assertTrue(self.shadow.session.require_release)
        self.assertEqual(self.shadow.session.fault, "input_timeout")
        self.assertIsNone(self.shadow.session.pending)

    def test_out_of_order_transport_frame_is_rejected(self):
        stamp = self.clock()
        self.assertTrue(self.runtime.submit_xr_frame(self.frame(False), received_monotonic_s=stamp))
        self.assertFalse(self.runtime.submit_xr_frame(self.frame(False), received_monotonic_s=stamp - .001))
        self.runtime.step()
        self.assertEqual(self.shadow.session.fault, "out_of_order_input_reception")

    def test_input_diagnostics_distinguish_missing_hand_from_grip(self):
        frame = self.frame(True)
        frame["controllers"] = {}
        self.assertFalse(self.runtime.submit_xr_frame(frame))
        self.runtime.step()
        info = self.runtime.status()["xr_input"]
        self.assertEqual(info["last_error"], "right_controller_missing")
        self.assertFalse(info["right_present"])
        self.assertIsNone(info["valid_frame_age_s"])
        self.tick(False)
        self.tick(True)
        info = self.runtime.status()["xr_input"]
        self.assertTrue(info["right_present"])
        self.assertTrue(info["grip"])
        self.assertEqual(info["rejected"], 1)
        self.assertEqual(info["accepted"], 2)
        self.assertTrue(self.shadow.session.engaged)

    def test_timing_separates_source_pause_from_extra_receive_delay_and_is_bounded(self):
        timing = InputTiming(capacity=3)
        frame = self.frame(True)
        timing.received(frame, 10.)
        frame = dict(frame, seq=frame['seq']+1, t_client=10010.)
        timing.received(frame, 10.25)
        last = timing.snapshot()['last_received']
        self.assertAlmostEqual(last['receive_gap_s'], .25)
        self.assertAlmostEqual(last['source_gap_s'], .01)
        self.assertAlmostEqual(last['relative_delay_change_s'], .24)
        frame = dict(frame, seq=frame['seq']+1, t_client=10310.)
        timing.received(frame, 10.55)
        last = timing.snapshot()['last_received']
        self.assertAlmostEqual(last['source_gap_s'], .3)
        self.assertAlmostEqual(last['relative_delay_change_s'], 0.)
        frozen = timing.snapshot(recent=True)
        timing.received(dict(frame, epoch='new'), 10.56)
        last = timing.snapshot()['last_received']
        self.assertIsNone(last['source_gap_s'])
        self.assertIsNone(last['seq_gap'])
        timing.received(None, float('nan'))
        self.assertEqual(len(timing.snapshot(recent=True)['recent_received']), 3)
        self.assertTrue(frozen['last_received']['grip'])
        self.assertFalse(timing.snapshot()['last_received']['right_present'])

    def test_core_rejection_records_source_delay_without_accepting_old_input(self):
        self.tick(False); self.tick(True)
        original_time = self.shadow.input_time
        frame = self.frame(True)
        frame['t_client'] -= 300.
        self.runtime.submit_xr_frame(frame)
        self.runtime.step()
        event = self.runtime.status()['input_timing']['last_processed']
        self.assertFalse(event['accepted'])
        self.assertEqual(event['reason'], 'source_delay_exceeded')
        self.assertGreater(event['relative_source_delay_s'], .2)
        self.assertEqual(self.shadow.input_time, original_time)


if __name__ == "__main__":
    unittest.main(verbosity=2)
