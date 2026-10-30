"""ServoJoint compatibility probe lifecycle with isolated synthetic SDK results."""
from contextlib import redirect_stderr, redirect_stdout
import fcntl
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.aubo import servoj_probe_cli as cli


def inspection():
    return dict(complete=True, controller_version="synthetic-controller",
        controller_version_code=1, interface_version=1, robot_mode=8, safety_mode=1,
        servo_mode_select=0, is_steady=True, joint_positions_rad=[0.] * 6,
        joint_speeds_rad_s=[0.] * 6, joint_min_positions_rad=[-3.] * 6,
        joint_max_positions_rad=[3.] * 6, joint_max_speeds_rad_s=[1.] * 6,
        joint_max_accelerations_rad_s2=[1.] * 6)


class FakeProbe:
    def __init__(self):
        self.calls = []
        self.data = inspection()
        self.connection = dict(return_code=0, phase="login", exception=False)
        self.cleanup = dict(logout_return_code=0, disconnect_return_code=0)

    def connect(self, host, port, user, password):
        self.calls.append("connect")
        return self.connection

    def inspect(self, robot):
        self.calls.append("inspect")
        return self.data

    def logout(self):
        self.calls.append("logout")
        return self.cleanup


class ProbeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir="/data/VR_teleoperation_runtime")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.probe = FakeProbe()
        self.native = types.SimpleNamespace(ServoJReadOnlyProbe=lambda timeout: self.probe)

    def run_cli(self, extra=(), *, connected=True):
        args = ["--runtime", str(self.root)]
        if connected:
            args += ["--connect", "--host", "synthetic.invalid", "--port", "30004", "--user", "test"]
        stdout, stderr = io.StringIO(), io.StringIO()
        with patch.object(cli, "ROOT", self.root), \
             patch.object(cli, "load_probe", return_value=(self.native, {})), \
             patch.object(cli.getpass, "getpass", return_value="synthetic-not-a-secret") as password, \
             redirect_stdout(stdout), redirect_stderr(stderr):
            code = cli.main(args + list(extra))
        return code, json.loads(stdout.getvalue()) if stdout.getvalue() else None, password

    def test_offline_does_not_load_sdk_or_prompt(self):
        stdout = io.StringIO()
        with patch.object(cli, "load_probe", side_effect=AssertionError("SDK loaded")) as load, \
             patch.object(cli.getpass, "getpass") as password, redirect_stdout(stdout):
            code = cli.main([])
        report = json.loads(stdout.getvalue())
        self.assertEqual(code, 0)
        load.assert_not_called()
        password.assert_not_called()
        self.assertEqual(report["status"], "offline_ready")
        self.assertFalse(report["motion_calls_made"])

    def test_valid_inspection_is_not_motion_support_proof(self):
        code, report, _ = self.run_cli()
        self.assertEqual(code, 0)
        self.assertTrue(report["read_only_compatibility_passed"])
        self.assertIsNone(report["servo_joint_supported_by_controller"])
        self.assertFalse(report["ready_for_motion_test"])
        self.assertEqual(self.probe.calls, ["connect", "inspect", "logout"])
        saved = Path(report["report_path"])
        self.assertEqual(json.loads(saved.read_text()), report)
        self.assertEqual(saved.stat().st_mode & 0o777, 0o600)

    def test_connection_refused_preserves_phase_and_cleans_up(self):
        self.probe.connection = dict(return_code=-8, phase="connect", exception=False)
        code, report, _ = self.run_cli()
        self.assertEqual(code, 1)
        self.assertEqual(report["failure_phase"], "connect")
        self.assertEqual(self.probe.calls, ["connect", "logout"])

    def test_version_mismatch_is_reported_without_motion(self):
        self.probe.connection = dict(return_code=-15, phase="connect", exception=False)
        code, report, _ = self.run_cli()
        self.assertEqual(code, 1)
        self.assertEqual(report["connection"]["return_code"], -15)
        self.assertFalse(report["sdk_connected"])

    def test_invalid_feedback_stops_before_sampling_and_is_serializable(self):
        self.probe.data["joint_positions_rad"][0] = float("nan")
        code, report, _ = self.run_cli(["--samples", "5"])
        self.assertEqual(code, 1)
        self.assertIn("invalid_joint_positions_rad", report["validation_errors"])
        self.assertIsNone(report["inspection"]["joint_positions_rad"][0])
        self.assertNotIn("sampling", report)

    def test_failed_query_keeps_partial_result_and_cleanup(self):
        self.probe.data = dict(complete=False, controller_version="synthetic-controller",
                               queries=[dict(field="servo_mode_select", success=False)])
        code, report, _ = self.run_cli()
        self.assertEqual(code, 1)
        self.assertEqual(report["inspection"]["queries"][-1]["field"], "servo_mode_select")
        self.assertEqual(self.probe.calls[-1], "logout")

    def test_exception_does_not_print_sdk_credentials_and_still_cleans_up(self):
        def fail(*args):
            raise RuntimeError("synthetic-not-a-secret")
        self.probe.inspect = fail
        code, report, _ = self.run_cli()
        self.assertEqual(code, 1)
        self.assertNotIn("synthetic-not-a-secret", json.dumps(report))
        self.assertEqual(self.probe.calls[-1], "logout")

    def test_cleanup_failure_cannot_report_success(self):
        self.probe.cleanup["disconnect_return_code"] = -1
        code, report, _ = self.run_cli()
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "cleanup_failed")
        self.assertTrue(report["cleanup_failed"])

    def test_cleanup_does_not_mask_primary_failure(self):
        self.probe.connection["return_code"] = -8
        self.probe.cleanup["logout_return_code"] = -1
        code, report, _ = self.run_cli()
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "connection_or_login_failed")
        self.assertTrue(report["cleanup_failed"])

    def test_busy_session_blocks_before_password(self):
        with (self.root / ".teleop_session.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            code, report, password = self.run_cli()
        self.assertEqual(code, 1)
        password.assert_not_called()
        self.assertIn("session is active", report["error"])
        self.assertEqual(self.probe.calls, [])

    def test_existing_report_is_never_overwritten(self):
        output = self.root / "existing.json"
        output.write_text("original")
        code, report, password = self.run_cli(["--output", str(output)])
        self.assertEqual(code, 1)
        self.assertIsNone(report)
        self.assertEqual(output.read_text(), "original")
        password.assert_not_called()

    def test_output_outside_data_rejected_before_password(self):
        code, report, password = self.run_cli(["--output", "/tmp/not-created-servoj.json"])
        self.assertEqual(code, 1)
        password.assert_not_called()

    def test_validation_rejects_bad_sizes_nonfinite_and_incoherent_limits(self):
        for field, value in (("joint_positions_rad", [0.] * 5),
                ("joint_speeds_rad_s", [float("inf")] * 6),
                ("joint_max_speeds_rad_s", [0.] * 6),
                ("joint_min_positions_rad", [5.] * 6),
                ("servo_mode_select", -1)):
            with self.subTest(field=field):
                data = inspection()
                data[field] = value
                self.assertTrue(cli.validate_inspection(data))

    def test_bad_cli_rejected_before_connection(self):
        cases = (["--connect"], ["--host", "synthetic.invalid"], ["--samples", "1"],
            ["--request-timeout-ms", "0"], ["--sample-hz", "nan"],
            ["--sample-hz", "inf"], ["--samples", "-1"])
        for case in cases:
            with self.subTest(case=case), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                cli.parse_args(case)

    def test_hash_mismatch_and_preloaded_sdk_block_import(self):
        binary, library = self.root / "probe.so", self.root / "libaubo_sdk.so"
        binary.write_bytes(b"synthetic-module")
        library.write_bytes(b"synthetic-library")
        manifest = dict(sdk_package_version=cli.SDK_VERSION, sdk_library=str(library),
            module_path=str(binary), sdk_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
            module_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
        path = self.root / "servoj-probe-build.json"
        path.write_text(json.dumps(manifest))
        library.write_bytes(b"changed")
        with self.assertRaisesRegex(cli.ProbeError, "hash changed"):
            cli.load_probe(self.root)
        library.write_bytes(b"synthetic-library")
        with patch.object(cli, "mapped_sdk_libraries", return_value={library}), \
             self.assertRaisesRegex(cli.ProbeError, "already loaded"):
            cli.load_probe(self.root)

    def test_slow_sample_skips_slots_instead_of_bursting(self):
        class Clock:
            now = 10.
            def monotonic(self): return self.now
            def sleep(self, duration): self.now += duration
        clock = Clock()
        starts = []
        def sample():
            started = clock.now
            starts.append(started)
            clock.now += .13
            return dict(complete=True, joint_positions_rad=[0.] * 6, queries=[dict(
                duration_s=.13, host_started_monotonic_s=started)])
        probe = types.SimpleNamespace(sample_joint_positions=sample)
        args = types.SimpleNamespace(samples=3, sample_hz=25.)
        with patch.object(cli.time, "monotonic", clock.monotonic), patch.object(cli.time, "sleep", clock.sleep):
            result = cli.sample_queries(probe, args)
        self.assertTrue(result["complete"])
        self.assertAlmostEqual(starts[1] - starts[0], .16)
        self.assertAlmostEqual(starts[2] - starts[1], .16)
        self.assertAlmostEqual(result["actual_request_hz"], 6.25)
        self.assertAlmostEqual(result["rpc_duration"]["median_ms"], 130.)

    def test_sampling_failure_stops_immediately(self):
        calls = []
        def sample():
            calls.append(1)
            return dict(complete=False, queries=[])
        probe = types.SimpleNamespace(sample_joint_positions=sample)
        result = cli.sample_queries(probe, types.SimpleNamespace(samples=10, sample_hz=25.))
        self.assertFalse(result["complete"])
        self.assertEqual(len(calls), 1)


if __name__ == "__main__":
    unittest.main()
