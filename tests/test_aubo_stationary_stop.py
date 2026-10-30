"""Stationary stop diagnostic gates; fake SDK only, no connection."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.aubo.stationary_stop_probe import main, run_diagnostic


def record(data=None, rc=0):
    return {"return_code": rc, "data": data}


class FakeClient:
    def __init__(self, *, state=0, joint_data=None, stop_error=False, stop_rc=0):
        self.state_value = state
        self.joint_data = [0.] * 6 if joint_data is None else joint_data
        self.stop_error = stop_error
        self.stop_rc = stop_rc
        self.calls = []

    def login(self, *args):
        self.calls.append("login")
        return record()

    def state(self):
        self.calls.append("state")
        return record(self.state_value)

    def joints(self):
        self.calls.append("joints")
        return record(self.joint_data)

    def stop_at_rest(self, method):
        self.calls.append(method)
        if self.stop_error:
            raise RuntimeError("lost response")
        return dict(record(rc=self.stop_rc), stop_invoked=True)

    def logout(self):
        self.calls.append("logout")
        return record()


class StopTests(unittest.TestCase):
    def run_probe(self, client):
        report, snapshots = {}, []
        status = run_diagnostic(client, report, host="fake", port=1, user="fake", password="fake",
            method="slow_control", checkpoint=lambda: snapshots.append(dict(report)))
        return status, report, snapshots

    def test_reported_running_refuses_stop_and_disconnects(self):
        client = FakeClient(state=1)
        status, report, _ = self.run_probe(client)
        self.assertEqual(status, 1)
        self.assertEqual(report["refusal_reason"], "controller_not_stopped")
        self.assertNotIn("slow_control", client.calls)
        self.assertEqual(client.calls[-1], "logout")

    def test_invalid_joint_feedback_refuses_stop(self):
        client = FakeClient(joint_data=[float("nan")] * 6)
        status, report, _ = self.run_probe(client)
        self.assertEqual(status, 1)
        self.assertIn("exception", report)
        self.assertNotIn("slow_control", client.calls)
        self.assertEqual(client.calls[-1], "logout")

    def test_exactly_one_request_records_intent_and_observations(self):
        client = FakeClient()
        status, report, snapshots = self.run_probe(client)
        self.assertEqual(status, 0)
        self.assertEqual(client.calls.count("slow_control"), 1)
        self.assertTrue(any(s.get("stop_call_phase") == "in_flight_result_unknown" for s in snapshots))
        self.assertEqual(report["stop_call_phase"], "returned")
        self.assertEqual(report["joint_delta_rad"], [0.] * 6)

    def test_exception_keeps_result_unknown_without_retry(self):
        client = FakeClient(stop_error=True)
        status, report, _ = self.run_probe(client)
        self.assertEqual(status, 1)
        self.assertEqual(report["stop_call_phase"], "in_flight_result_unknown")
        self.assertEqual(client.calls.count("slow_control"), 1)
        self.assertEqual(client.calls[-1], "logout")

    def test_failed_return_is_preserved(self):
        status, report, _ = self.run_probe(FakeClient(stop_rc=10001))
        self.assertEqual(status, 1)
        self.assertEqual(report["stop_result"]["return_code"], 10001)
        self.assertIn("after_joints", report)

    def test_default_plan_never_loads_sdk(self):
        with patch("teleop.aubo.stationary_stop_probe.load_stop_native") as loader, patch("builtins.print"):
            self.assertEqual(main([]), 0)
            loader.assert_not_called()

    def test_connect_requires_explicit_fields_and_operator_presence(self):
        with patch("teleop.aubo.stationary_stop_probe.load_stop_native") as loader:
            with self.assertRaises(SystemExit):
                main(["--connect", "--host", "fake"])
            loader.assert_not_called()


if __name__ == "__main__":
    unittest.main(verbosity=2)
