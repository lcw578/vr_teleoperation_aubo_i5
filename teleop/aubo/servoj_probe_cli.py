"""Separate ARCS compatibility and read-only RPC timing probe; no motion API."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import getpass
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import statistics
import sys
import threading
import time
import uuid
import warnings

ROOT = Path(__file__).resolve().parents[2]
SDK_VERSION = "0.24.1-rc.3+318754d"
_LOAD_LOCK = threading.RLock()
PROBE_METHODS = {"connect", "inspect", "sample_joint_positions", "logout"}


class ProbeError(RuntimeError):
    """Fixed, non-sensitive message safe to include in a report."""


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", action="store_true", help="explicit read-only login, never servo entry")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int, help="ARCS RPC port, normally 30004; old Follow uses another protocol")
    parser.add_argument("--user")
    parser.add_argument("--robot", default="", help="required if controller exposes multiple robots")
    parser.add_argument("--request-timeout-ms", type=int, default=1000)
    parser.add_argument("--samples", type=int, default=0, help="optional extra joint-position queries, 0..200")
    parser.add_argument("--sample-hz", type=float, default=10.0, help="requested read-only query rate, 1..50 Hz")
    parser.add_argument("--runtime", default=os.environ.get("AUBO_RUNTIME_DIR", "/data/VR_teleoperation_runtime"))
    parser.add_argument("--output", type=Path, help="new JSON report under /data; default auto-name for live probe")
    args = parser.parse_args(argv)
    if args.connect and (not args.host or args.port is None or not args.user):
        parser.error("--connect requires explicit --host, --port and --user")
    if args.port is not None and not 1 <= args.port <= 65535:
        parser.error("--port must be in 1..65535")
    if not args.connect and (any((args.host, args.port, args.user, args.robot)) or args.samples):
        parser.error("connection details and sampling require --connect")
    if not 100 <= args.request_timeout_ms <= 5000:
        parser.error("--request-timeout-ms must be in 100..5000")
    if not 0 <= args.samples <= 200:
        parser.error("--samples must be in 0..200")
    if not math.isfinite(args.sample_hz) or not 1 <= args.sample_hz <= 50:
        parser.error("--sample-hz must be finite and in 1..50")
    if args.samples / args.sample_hz > 30:
        parser.error("requested sampling period exceeds 30 seconds")
    return args


def mapped_sdk_libraries():
    mapped = set()
    for line in Path("/proc/self/maps").read_text().splitlines():
        fields = line.split(None, 5)
        if len(fields) == 6 and Path(fields[5]).name.startswith("libaubo_sdk"):
            mapped.add(Path(fields[5]).resolve())
    return mapped


def load_probe(runtime):
    """Load in a fresh process, checking hashes before and mapped SDK after import."""
    with _LOAD_LOCK:
        root = Path(runtime).resolve()
        manifest = json.loads((root / "servoj-probe-build.json").read_text())
        if manifest.get("sdk_package_version") != SDK_VERSION:
            raise ProbeError("ARCS build manifest version mismatch; rebuild probe")
        sdk_lib = Path(manifest["sdk_library"]).resolve()
        module_path = Path(manifest["module_path"]).resolve()
        for path, key in ((sdk_lib, "sdk_sha256"), (module_path, "module_sha256")):
            if hashlib.sha256(path.read_bytes()).hexdigest() != manifest[key]:
                raise ProbeError("ARCS library or probe hash changed; rebuild probe")
        if not module_path.is_relative_to(root) or not sdk_lib.is_relative_to(Path("/data")):
            raise ProbeError("Probe must be in runtime and SDK must be under /data")
        if mapped_sdk_libraries() or "_aubo_servoj_probe" in sys.modules:
            raise ProbeError("An AUBO SDK is already loaded; run this probe in a fresh process")
        if not hasattr(os, "RTLD_DEEPBIND"):
            raise ProbeError("This probe requires Linux RTLD_DEEPBIND isolation")
        import ctypes
        handle = ctypes.CDLL(str(sdk_lib), mode=os.RTLD_NOW | os.RTLD_LOCAL | os.RTLD_DEEPBIND)
        spec = importlib.util.spec_from_file_location("_aubo_servoj_probe", module_path)
        if spec is None or spec.loader is None:
            raise ProbeError("Could not load built ServoJoint read-only probe")
        native = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(native)
        if mapped_sdk_libraries() != {sdk_lib}:
            raise ProbeError("Mapped SDK differs from the recorded ARCS library; restart process")
        methods = {name for name in dir(native.ServoJReadOnlyProbe) if not name.startswith("_")}
        if methods != PROBE_METHODS or native.sdk_package_version != SDK_VERSION:
            raise ProbeError("Native read-only API or SDK version mismatch")
        native._isolated_sdk_handle = handle
        sys.modules["_aubo_servoj_probe"] = native
        return native, manifest


@contextmanager
def session_lock():
    with (ROOT / ".teleop_session.lock").open("a") as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ProbeError("Another teleoperation session is active; close it before probing") from None
        try:
            yield
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


def finite_six(value):
    return (isinstance(value, (list, tuple)) and len(value) == 6 and
            all(type(x) in (int, float) and math.isfinite(x) for x in value))


def validate_inspection(data):
    errors = []
    if data.get("complete") is not True:
        return ["inspection_incomplete"]
    fields = ("joint_positions_rad", "joint_speeds_rad_s", "joint_min_positions_rad",
              "joint_max_positions_rad", "joint_max_speeds_rad_s", "joint_max_accelerations_rad_s2")
    for field in fields:
        if not finite_six(data.get(field)):
            errors.append("invalid_" + field)
    if not errors:
        q, lo, hi, v, a = (data[key] for key in (fields[0], fields[2], fields[3], fields[4], fields[5]))
        if any(l >= h for l, h in zip(lo, hi)):
            errors.append("incoherent_joint_position_limits")
        if any(x <= 0 for x in (*v, *a)):
            errors.append("nonpositive_physical_speed_or_acceleration_limit")
        if any(x < l - 1e-6 or x > h + 1e-6 for x, l, h in zip(q, lo, hi)):
            errors.append("joint_position_outside_reported_physical_limits")
    if not isinstance(data.get("controller_version"), str) or not data.get("controller_version"):
        errors.append("missing_controller_version")
    for field in ("controller_version_code", "interface_version", "robot_mode", "safety_mode"):
        if type(data.get(field)) is not int or data[field] < 0:
            errors.append("invalid_" + field)
    if type(data.get("servo_mode_select")) is not int or data["servo_mode_select"] not in (0, 1, 2):
        errors.append("invalid_servo_mode_select")
    if type(data.get("is_steady")) is not bool:
        errors.append("invalid_is_steady")
    return errors


def timing_summary(values):
    if not values:
        return {"count": 0}
    ordered = sorted(values)
    return {"count": len(values), "median_ms": statistics.median(values) * 1000,
            "p95_ms": ordered[math.ceil(.95 * len(values)) - 1] * 1000,
            "max_ms": max(values) * 1000}


def sample_queries(probe, args):
    """Finite serial getter benchmark. Skip elapsed slots; never replay a backlog."""
    result = {"method": "getJointPositions", "requested_hz": args.sample_hz,
              "requested_count": args.samples, "samples": [], "complete": True,
              "measures": "read-only RPC round trip, not ServoJoint or motion latency"}
    period = 1 / args.sample_hz
    started = time.monotonic()
    deadline = started + 30
    next_due = started
    durations, stamps = [], []
    for _ in range(args.samples):
        now = time.monotonic()
        if now >= deadline or next_due >= deadline:
            result.update(complete=False, error="sampling_time_limit")
            break
        if now < next_due:
            time.sleep(next_due - now)
        sample = probe.sample_joint_positions()
        result["samples"].append(sample)
        if not sample.get("complete") or not finite_six(sample.get("joint_positions_rad")):
            result.update(complete=False, error="invalid_or_failed_joint_sample")
            break
        query = sample["queries"][0]
        durations.append(query["duration_s"])
        stamps.append(query["host_started_monotonic_s"])
        next_due += period
        now = time.monotonic()
        if next_due < now:
            next_due += (math.floor((now - next_due) / period) + 1) * period
    result["rpc_duration"] = timing_summary(durations)
    result["request_interval"] = timing_summary([b - a for a, b in zip(stamps, stamps[1:])])
    result["actual_request_hz"] = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else None
    return result


@contextmanager
def report_file(args):
    output = args.output
    if output is None and args.connect:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
        output = Path(args.runtime) / "servoj-reports" / f"probe-{stamp}-{uuid.uuid4().hex[:8]}.json"
    if output is None:
        yield None, None
        return
    output = output.resolve()
    if not output.is_relative_to(Path("/data")) or output.suffix != ".json":
        raise ProbeError("Report output must be a new .json file under /data")
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        fd = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        raise ProbeError("Report already exists; choose a new output path") from None
    with os.fdopen(fd, "w") as stream:
        yield output, stream


def run_connected(args, report):
    native, manifest = load_probe(args.runtime)
    report["build"] = manifest
    probe = native.ServoJReadOnlyProbe(args.request_timeout_ms)
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("error", getpass.GetPassWarning)
            password = getpass.getpass("ARCS SDK password (not recorded): ")
        try:
            connection = probe.connect(args.host, args.port, args.user, password)
        finally:
            password = None
        report["connection"] = connection
        if connection["return_code"] != 0 or connection.get("exception"):
            report.update(status="connection_or_login_failed", failure_phase=connection["phase"])
            return 1
        report["sdk_connected"] = True
        print("已登录，正在逐项执行只读查询；不会进入伺服或发送运动指令。", file=sys.stderr)
        inspection = probe.inspect(args.robot)
        report["inspection"] = inspection
        errors = validate_inspection(inspection)
        report["validation_errors"] = errors
        report["read_only_compatibility_passed"] = not errors
        if errors:
            report["status"] = "inspection_failed"
            return 1
        report["status"] = "read_only_inspection_complete"
        if args.samples:
            report["sampling"] = sample_queries(probe, args)
            if not report["sampling"]["complete"]:
                report["status"] = "sampling_failed"
                return 1
        return 0
    finally:
        try:
            report["cleanup"] = probe.logout()
        except Exception:
            report["cleanup"] = {"exception": True}


def main(argv=None):
    args = parse_args(argv)
    report = dict(schema_version=1, status="offline_ready", mode="offline_plan", sdk_connected=False,
        created_utc=datetime.now(timezone.utc).isoformat(), sdk_package_version=SDK_VERSION,
        read_only=True, motion_calls_made=False, configuration_changed=False,
        servo_joint_supported_by_controller=None, ready_for_motion_test=False,
        physical_limits_are_configured_safety_limits=False,
        pending=["controller and ARCS compatibility", "RTDE feedback verification",
                 "version-specific servo entry, stop and no-command behavior", "finite motion comparison"])
    code = 0
    output = None
    try:
        with report_file(args) as (output, stream):
            try:
                if args.connect:
                    report.update(mode="read_only_controller_probe", host=args.host, port=args.port,
                                  request_timeout_ms=args.request_timeout_ms)
                    with session_lock():
                        code = run_connected(args, report)
                    cleanup = report.get("cleanup", {})
                    if cleanup and (cleanup.get("exception") or cleanup.get("logout_return_code") != 0 or
                                    cleanup.get("disconnect_return_code") != 0):
                        report["cleanup_failed"] = True
                        if code == 0:
                            report["status"] = "cleanup_failed"
                        code = 1
            except ProbeError as exc:
                report.update(status="probe_failed", error=str(exc))
                code = 1
            except (KeyboardInterrupt, EOFError):
                report.update(status="interrupted")
                code = 130
            except Exception:
                # Do not print vendor exceptions, tracebacks, or login payloads.
                report.update(status="probe_failed", error="Probe failed; inspect fixed-status fields and rebuild if needed")
                code = 1
            report["finished_utc"] = datetime.now(timezone.utc).isoformat()
            if output:
                report["report_path"] = str(output)
            rendered = json.dumps(json_safe(report), indent=2, ensure_ascii=False, allow_nan=False) + "\n"
            if stream:
                stream.write(rendered)
            print(rendered, end="")
    except (ProbeError, OSError, ValueError):
        print("无法创建或写入报告：请检查 /data 输出路径、写入权限及文件是否已存在。", file=sys.stderr)
        return 1
    return code


def json_safe(value):
    """Keep a diagnostic report writable even if the SDK returns NaN/Inf."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


if __name__ == "__main__":
    raise SystemExit(main())
