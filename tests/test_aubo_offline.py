"""Offline preparation tests. No SDK login, controller address or real movement."""
import contextlib
import io
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.aubo.feedback import FeedbackCache
from teleop.aubo.offline_session import FakePositionBackend, OfflinePositionSession, SessionPolicy
from teleop.aubo.probe import json_safe, main, parse_args, read_report, sample_feedback
from teleop.aubo.readonly import (DEFAULT_QUERY_NAMES, MotionUnavailable,
                                 QUERY_NAMES, ReadOnlyFollower)


class Clock:
    now = 10.
    def __call__(self): return self.now
    def advance(self, value): self.now += value


class FeedbackTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.cache = FeedbackCache(self.clock)

    def test_query_failure_invalidates_previous_measurement(self):
        self.cache.update([.1]*6, source="fake", return_code=0)
        self.cache.update([0]*6, source="fake", return_code=10007)
        with self.assertRaisesRegex(RuntimeError, "failed"):
            self.cache.require_recent(.1)
        self.assertIsNone(self.cache.latest().q_rad)

    def test_unchanged_positions_with_new_reception_are_valid(self):
        self.cache.update([.1]*6, source="fake")
        self.clock.advance(.09)
        self.cache.update([.1]*6, source="fake")
        self.clock.advance(.09)
        self.assertEqual(self.cache.require_recent(.1).q_rad, (.1,)*6)
        self.assertIsNone(self.cache.latest().controller_sample_time)

    def test_expiry_and_nonfinite_feedback(self):
        self.cache.update([0]*6, source="fake")
        self.clock.advance(.2)
        with self.assertRaisesRegex(RuntimeError, "expired"):
            self.cache.require_recent(.1)
        for bad in ([float("nan")]*6, [0]*5, None):
            self.cache.update(bad, source="fake")
            with self.assertRaisesRegex(RuntimeError, "invalid"):
                self.cache.require_recent(.1)

    def test_reject_future_and_out_of_order_reception(self):
        self.cache.update([0]*6, source="fake")
        with self.assertRaises(ValueError):
            self.cache.update([0]*6, source="fake", received_monotonic_s=11)
        self.assertFalse(self.cache.latest().valid)
        self.cache.update([.2]*6, source="fake")
        with self.assertRaises(ValueError):
            self.cache.update([0]*6, source="fake", received_monotonic_s=9)
        self.assertEqual(self.cache.require_recent(.1).q_rad, (.2,)*6)

    def test_unparseable_feedback_timestamp_invalidates_old_sample(self):
        for bad in (None, "bad", float("inf"), float("nan")):
            self.cache.update([0]*6, source="fake")
            with self.assertRaises(ValueError):
                self.cache.update([0]*6, source="fake", received_monotonic_s=bad)
            with self.assertRaises(RuntimeError):
                self.cache.require_recent(.1)


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.backend = FakePositionBackend()
        # Deliberately artificial test values, NOT AUBO commissioning settings.
        self.policy = SessionPolicy((-1,)*6, (1,)*6, (1,)*6, .1, .1, .05)
        self.session = OfflinePositionSession(self.backend, self.policy, clock=self.clock)
        self.feedback()

    def feedback(self, value=0.):
        self.session.cache.update([value]*6, source="fake_only")

    def input(self, grip):
        self.session.input(grip=grip, received_monotonic_s=self.clock(), tracking_valid=True)

    def engage(self):
        self.input(False); self.input(True)
        self.assertTrue(self.session.engaged)

    def offer(self, value, **kw):
        return self.session.offer([value]*6, generated_monotonic_s=kw.get("time", self.clock()),
                                  session=kw.get("session", self.session.session))

    def test_no_initial_grip_motion_and_measured_anchor(self):
        self.input(True)
        self.assertFalse(self.session.engaged)
        self.feedback(.3); self.engage()
        self.assertEqual(self.session.last_reference, (.3,)*6)
        self.assertEqual(self.backend.submitted, [])

    def test_latest_candidate_only_and_reference_is_not_measurement(self):
        self.engage(); self.clock.advance(.01)
        self.assertTrue(self.offer(.001)); self.clock.advance(.001)
        self.assertTrue(self.offer(.002))
        self.assertTrue(self.session.dispatch())
        self.assertEqual(self.backend.submitted, [(.002,)*6])
        self.assertEqual(self.session.cache.latest().q_rad, (0.,)*6)

    def test_grip_release_reanchors_measured_not_previous_target(self):
        self.engage(); self.clock.advance(.01); self.offer(.002); self.session.dispatch()
        self.input(False); self.feedback(.001); self.input(True)
        self.assertEqual(self.session.last_reference, (.001,)*6)
        self.assertIsNone(self.session.pending)

    def test_delayed_older_candidate_cannot_replace_already_submitted_target(self):
        self.engage(); self.clock.advance(.01)
        older_stamp = self.clock()
        self.clock.advance(.01)
        self.assertTrue(self.offer(.002))
        self.assertTrue(self.session.dispatch())
        self.clock.advance(.01)
        self.assertFalse(self.offer(.001, time=older_stamp))
        self.assertFalse(self.session.dispatch())
        self.assertEqual(self.backend.submitted, [(.002,)*6])

    def test_timeout_requires_release_and_never_replays_old_session(self):
        self.engage(); old = self.session.session
        self.offer(.001); self.clock.advance(.2)
        self.assertFalse(self.session.check())
        self.feedback(); self.input(True)
        self.assertFalse(self.session.engaged)
        self.input(False); self.input(True)
        self.assertTrue(self.session.engaged)
        self.assertFalse(self.offer(.001, session=old))
        self.assertFalse(self.session.dispatch())
        self.assertEqual(self.backend.submitted, [])

    def test_feedback_failure_stops_before_dispatch(self):
        self.engage(); self.offer(.001)
        self.session.cache.invalidate("query_failed")
        self.assertFalse(self.session.dispatch())
        self.assertTrue(self.session.require_release)
        self.assertEqual(self.backend.submitted, [])
        self.assertEqual(self.backend.stop_requests, 1)

    def test_expired_pending_does_not_get_refreshed_by_new_input(self):
        self.engage(); self.offer(.001); self.clock.advance(.06)
        self.feedback(); self.input(True)
        self.assertFalse(self.session.dispatch())
        self.assertEqual(self.session.fault, "candidate_expired")

    def test_large_jump_is_rejected_without_clipping_to_a_new_pose(self):
        self.engage(); self.clock.advance(.01); self.offer(.2)
        self.assertFalse(self.session.dispatch())
        self.assertIn("velocity_budget", self.session.fault)
        self.assertEqual(self.backend.submitted, [])

    def test_long_pause_does_not_expand_step_budget(self):
        self.engage(); self.clock.advance(.06); self.feedback(); self.input(True)
        self.offer(.001)
        self.assertFalse(self.session.dispatch())
        self.assertEqual(self.session.fault, "submission_period_invalid")

    def test_unknown_submission_response_is_not_retried(self):
        self.engage(); self.clock.advance(.01); self.offer(.001)
        self.backend.fail_next = TimeoutError("fake delayed/lost response")
        self.assertFalse(self.session.dispatch())
        self.assertEqual(len(self.backend.submitted), 1)
        self.assertEqual(self.session.attempts[0]["outcome"], "response_unknown")
        self.assertFalse(self.session.dispatch())
        self.feedback(); self.input(True)
        self.assertFalse(self.session.engaged)
        self.assertEqual(len(self.backend.submitted), 1)

    def test_invalid_candidates_and_outside_measured_range(self):
        self.engage()
        self.assertFalse(self.session.offer([float("nan")]*6,
                         generated_monotonic_s=self.clock(), session=self.session.session))
        self.feedback(2.); self.input(False); self.input(True)
        self.assertFalse(self.session.engaged)
        self.assertIn("outside", self.session.fault)

    def test_disconnect_clears_target_feedback_and_engagement(self):
        self.engage(); self.offer(.001); self.session.disconnect()
        self.assertIsNone(self.session.pending)
        self.assertFalse(self.session.cache.latest().valid)
        self.feedback(); self.input(True)
        self.assertFalse(self.session.engaged)

    def test_harness_refuses_non_fake_backend(self):
        with self.assertRaises(TypeError):
            OfflinePositionSession(object(), self.policy)

    def test_malformed_timestamps_revoke_pending_before_any_later_dispatch(self):
        for operation in ("input", "offer"):
            for bad in (None, "bad", float("inf"), float("nan")):
                self.feedback(); self.engage(); self.clock.advance(.001)
                self.assertTrue(self.offer(.0001))
                if operation == "input":
                    self.session.input(grip=True, received_monotonic_s=bad, tracking_valid=True)
                else:
                    self.session.offer([0]*6, generated_monotonic_s=bad, session=self.session.session)
                self.assertFalse(self.session.engaged)
                self.assertTrue(self.session.require_release)
                self.assertIsNone(self.session.pending)
                self.assertFalse(self.session.dispatch())
        self.assertEqual(self.backend.submitted, [])

    def test_duplicate_candidate_timestamp_does_not_replace_newer_candidate(self):
        self.engage(); self.clock.advance(.01)
        self.assertTrue(self.offer(.001))
        self.assertFalse(self.offer(.002))
        self.assertTrue(self.session.dispatch())
        self.assertEqual(self.backend.submitted, [(.001,)*6])

    def test_stop_failure_locks_session_even_after_release(self):
        self.engage(); self.offer(.001)
        self.backend.fail_stop_next = RuntimeError("fake stop failure")
        self.input(False)
        self.assertIsNone(self.session.pending)
        self.assertEqual(self.session.fault, "fake_stop_failed")
        self.feedback(); self.input(False); self.input(True)
        self.assertFalse(self.session.engaged)
        self.assertFalse(self.session.dispatch())


class QueryClient:
    """Query-only fake; has no motion implementation."""
    def __init__(self, clock):
        self.clock, self.calls, self.rc = clock, [], 0
    def login(self, *args): return {"return_code": 0}
    def logout(self): return {"return_code": 0}
    def query(self, name):
        self.calls.append(name)
        return {"operation": name, "return_code": self.rc, "host_received_monotonic_s": self.clock(),
                "data": {"joint_positions_rad": [0]*6} if self.rc == 0 else None}
    def fk(self, q): return {"operation": "fk", "return_code": 0, "data": {}}


class ReadOnlyTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock(); self.client = QueryClient(self.clock)
        self.follower = ReadOnlyFollower(self.client, FeedbackCache(self.clock))
        self.follower.connect(host="fake.test", port=1, user="fake", password="fake")

    def test_motion_and_unallowlisted_queries_never_reach_client(self):
        with self.assertRaises(MotionUnavailable): self.follower.send_action([0]*6)
        with self.assertRaises(MotionUnavailable): self.follower.stop()
        with self.assertRaises(ValueError): self.follower.query("startup")
        self.assertEqual(self.client.calls, [])

    def test_invalid_query_cannot_return_old_feedback(self):
        self.follower.query("joint_angles")
        self.assertTrue(self.follower.get_observation(max_age_s=.1)["feedback_valid"])
        self.client.rc = 10007; self.follower.query("joint_angles")
        with self.assertRaises(RuntimeError): self.follower.get_observation(max_age_s=.1)

    def test_communication_timeout_aborts_report_without_retry(self):
        self.client.rc = 10007
        report = read_report(self.follower, QUERY_NAMES)
        self.assertEqual(len(report), 1)
        self.assertEqual(len(self.client.calls), 1)

    def test_partial_report_is_retained_after_query_exception(self):
        original = self.client.query
        def query(name):
            if name == "robot_state": raise RuntimeError("fake failure")
            return original(name)
        self.client.query = query
        report = read_report(self.follower, ("joint_angles", "robot_state"))
        self.assertEqual(report[0]["return_code"], 0)
        self.assertEqual(report[1]["exception"]["type"], "RuntimeError")

    def test_raw_sdk_result_is_retained_when_host_validation_fails(self):
        original = self.client.query
        def query(name):
            result = original(name)
            result.update(host_received_monotonic_s=self.clock()+1, duration_s=.001)
            return result
        self.client.query = query
        report = read_report(self.follower, ("joint_angles", "robot_state"))
        self.assertEqual(len(report), 1)
        self.assertEqual(report[0]["return_code"], 0)
        self.assertEqual(report[0]["duration_s"], .001)
        self.assertEqual(report[0]["data"]["joint_positions_rad"], [0]*6)
        self.assertFalse(report[0]["feedback_validated"])
        self.assertIn("validation_error", report[0])
        with self.assertRaises(RuntimeError): self.follower.get_observation(max_age_s=.1)

    def test_nonfinite_measurement_is_not_marked_validated(self):
        self.client.query = lambda name: {"operation": name, "return_code": 0,
            "host_received_monotonic_s":self.clock(), "data":{"joint_positions_rad":[float("nan")]*6}}
        report = read_report(self.follower, ("joint_angles",))
        self.assertFalse(report[0]["feedback_validated"])
        self.assertEqual(len(report), 1)

    def test_disconnect_invalidates_and_blocks_observation(self):
        self.follower.query("joint_angles"); self.follower.disconnect()
        with self.assertRaises(RuntimeError): self.follower.get_observation(max_age_s=.1)

    def test_default_probe_never_loads_sdk(self):
        with patch("teleop.aubo.probe.load_native", side_effect=AssertionError("must not load")):
            with contextlib.redirect_stdout(io.StringIO()) as stream:
                self.assertEqual(main([]), 0)
        report = json.loads(stream.getvalue())
        self.assertFalse(report["sdk_loaded"])
        self.assertFalse(report["controller_connected"])
        self.assertNotIn("joint_position_limits", report["queries"])
        self.assertIn("10007", report["default_excluded_queries"]["joint_position_limits"])

    def test_default_connection_omits_known_timeout_and_labels_local_results(self):
        # main() owns a real host-clock cache; use the same clock domain here.
        self.client.clock = time.monotonic
        native = SimpleNamespace(ReadOnlyClient=lambda: self.client)
        with patch("teleop.aubo.probe.load_native", return_value=native), \
                patch("teleop.aubo.probe.getpass.getpass", return_value="fake"), \
                contextlib.redirect_stdout(io.StringIO()) as stream:
            status = main(["--connect", "--host", "fake.test", "--port", "1", "--user", "fake"])
        report = json.loads(stream.getvalue())
        self.assertEqual(status, 0)
        self.assertEqual(self.client.calls, list(DEFAULT_QUERY_NAMES))
        self.assertNotIn("joint_position_limits", self.client.calls)
        rows = {row["operation"]: row for row in report["queries"]}
        self.assertEqual(rows["tool_kinematics"]["data_source"], "sdk_client_move_profile")
        self.assertEqual(rows["fk"]["data_source"], "sdk_local_fk")

    def test_explicit_timeout_query_is_not_silently_skipped_or_retried(self):
        self.client.rc = 10007
        native = SimpleNamespace(ReadOnlyClient=lambda: self.client)
        with patch("teleop.aubo.probe.load_native", return_value=native), \
                patch("teleop.aubo.probe.getpass.getpass", return_value="fake"), \
                contextlib.redirect_stdout(io.StringIO()) as stream:
            status = main(["--connect", "--host", "fake.test", "--port", "1", "--user", "fake",
                           "--query", "joint_position_limits", "--query", "diagnosis"])
        report = json.loads(stream.getvalue())
        self.assertEqual(status, 1)
        self.assertEqual(self.client.calls, ["joint_position_limits"])
        self.assertEqual(report["query_selection"], "explicit")
        self.assertEqual(report["default_excluded_queries"], {})
        self.assertEqual(report["queries"][0]["return_code"], 10007)
        self.assertEqual(report["logout"]["return_code"], 0)

    def test_loader_failure_is_reported_before_asking_for_password(self):
        with patch("teleop.aubo.probe.load_native", side_effect=RuntimeError("fake unsafe preload")), \
                patch("teleop.aubo.probe.getpass.getpass") as password, \
                contextlib.redirect_stdout(io.StringIO()) as stream:
            status = main(["--connect", "--host", "fake.test", "--port", "1", "--user", "fake"])
        report = json.loads(stream.getvalue())
        self.assertEqual(status, 1)
        password.assert_not_called()
        self.assertEqual(report["queries"], [])
        self.assertIsNone(report["logout"])
        self.assertEqual(report["exception"]["message"], "fake unsafe preload")

    def test_explicit_connection_parameters_are_required(self):
        for args in (["--connect"], ["--host", "fake.test"], ["--connect", "--host", "fake.test", "--port", "1"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(args)

    def test_sampling_requires_explicit_pair_and_connection(self):
        for args in (["--feedback-samples", "2"], ["--feedback-samples","2","--feedback-period",".1"],
                     ["--connect","--host","fake.test","--port","1","--user","fake",
                      "--feedback-samples","2","--feedback-period","nan"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): parse_args(args)

    def test_bounded_sampling_preserves_unchanged_q_and_measures_host_intervals(self):
        result = sample_feedback(self.follower, count=3, period_s=.03,
                                 clock=self.clock, wait=self.clock.advance)
        self.assertEqual(len(result["records"]), 3)
        self.assertAlmostEqual(result["host_reception_interval_s"]["mean"], .03)
        self.assertIsNone(result["controller_sample_time"])
        self.assertTrue(self.follower.get_observation(max_age_s=.1)["feedback_valid"])

    def test_bounded_sampling_stops_on_failed_query_without_retry(self):
        self.client.rc = 10007
        result = sample_feedback(self.follower, count=3, period_s=.03,
                                 clock=self.clock, wait=self.clock.advance)
        self.assertEqual(len(result["records"]), 1)
        self.assertEqual(self.client.calls, ["joint_angles"])

    def test_nonfinite_data_is_marked_in_strict_json(self):
        invalid=[];value=json_safe({"q": [float("nan"), 1.]}, invalid=invalid)
        self.assertEqual(invalid, [".q[0]"])
        self.assertIsNone(value["q"][0])
        json.dumps(value, allow_nan=False)


if __name__ == "__main__":
    unittest.main(verbosity=2)
