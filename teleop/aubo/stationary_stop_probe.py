"""One stationary stop request; default invocation is an offline plan.

This is a device diagnostic, not a hardware follower or validated stop path.
It deliberately shares only the reviewed SDK loader with the read-only entry.
"""
import argparse
from datetime import datetime
import getpass
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

from .feedback import joints
from .probe import json_safe, record_ok
from .readonly import load_native

METHODS = {
    "public_stop": "ServiceInterface::robotMoveStop()",
    "slow_control": "ServiceInterface::rootServiceRobotMoveControl(RobotMoveStop)",
}


def load_stop_native(runtime=None):
    root = Path(runtime or os.environ.get("AUBO_RUNTIME_DIR", "/data/VR_teleoperation_runtime"))
    # Load/bind the SDK in its own scope, with the original hash/path checks.
    readonly = load_native(root)
    manifest = json.loads((root / "stop-probe-build.json").read_text())
    if Path(manifest["release_library"]).resolve() != Path(readonly._isolated_sdk_handle._name).resolve():
        raise RuntimeError("Stop diagnostic uses a different SDK")
    library = Path(manifest["release_library"])
    if hashlib.sha256(library.read_bytes()).hexdigest() != manifest["release_sha256"]:
        raise RuntimeError("SDK changed since stop diagnostic build")
    binary = Path(manifest["module_path"])
    if hashlib.sha256(binary.read_bytes()).hexdigest() != manifest["module_sha256"]:
        raise RuntimeError("Stop diagnostic extension changed since build")
    name = "_aubo_stop_probe"
    module = sys.modules.get(name)
    if module is None:
        spec = importlib.util.spec_from_file_location(name, binary)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        sys.modules[name] = module
    elif Path(module.__file__).resolve() != binary.resolve():
        raise RuntimeError("A different stop diagnostic extension is already loaded")
    if tuple(module.stop_methods) != tuple(METHODS):
        raise RuntimeError("Unexpected stop diagnostic methods")
    return module


def run_diagnostic(client, report, *, host, port, user, password, method, checkpoint):
    """Finite serial diagnostic; never retry or promote the result to motion acceptance."""
    connected = False
    status = 1
    try:
        report["login"] = client.login(host, port, user, password)
        connected = record_ok(report["login"])
        checkpoint()
        if not connected:
            return 1
        report["before_state"] = client.state()
        checkpoint()
        report["before_joints"] = client.joints()
        checkpoint()
        if not all(record_ok(report[name]) for name in ("before_state", "before_joints")):
            report["refusal_reason"] = "precheck_query_failed"
            return 1
        q_before = joints(report["before_joints"]["data"])
        if report["before_state"]["data"] != 0:
            report["refusal_reason"] = "controller_not_stopped"
            return 1
        # Persist intent first: an exception/crash must not be described as no request.
        report["stop_call_phase"] = "in_flight_result_unknown"
        checkpoint()
        report["stop_result"] = client.stop_at_rest(method)
        report["stop_call_phase"] = "returned"
        checkpoint()
        report["after_state"] = client.state()
        checkpoint()
        report["after_joints"] = client.joints()
        checkpoint()
        if record_ok(report["after_joints"]):
            q_after = joints(report["after_joints"]["data"])
            report["joint_delta_rad"] = [b - a for a, b in zip(q_before, q_after)]
        stop = report["stop_result"]
        status = 0 if (stop.get("stop_invoked") is True and record_ok(stop) and
                       record_ok(report["after_state"]) and report["after_state"]["data"] == 0 and
                       record_ok(report["after_joints"])) else 1
    except Exception as exc:
        report["exception"] = {"type": type(exc).__name__, "message": str(exc)}
        status = 1
    finally:
        if connected:
            try:
                report["logout"] = client.logout()
                if not record_ok(report["logout"]):
                    status = 1
            except Exception as exc:
                report["logout_exception"] = {"type": type(exc).__name__, "message": str(exc)}
                status = 1
        checkpoint()
    return status


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", action="store_true", help="explicitly connect and test one stop request")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--user")
    parser.add_argument("--method", choices=METHODS)
    parser.add_argument("--operator-ready", action="store_true", help="operator is present with access to stop controls")
    parser.add_argument("--runtime")
    parser.add_argument("--output", help="required new finite report under /data")
    args = parser.parse_args(argv)
    if args.connect:
        if not all((args.host, args.port, args.user, args.method, args.operator_ready, args.output)):
            parser.error("--connect requires host, port, user, method, --operator-ready and --output")
        if not 1 <= args.port <= 65535:
            parser.error("port must be in 1..65535")
        target = Path(args.output).expanduser().resolve()
        if not target.is_relative_to(Path("/data")) or not target.parent.is_dir() or target.exists():
            parser.error("output must be a new file in an existing /data directory")
        args.output = target
    elif any((args.host, args.port, args.user, args.method, args.operator_ready, args.output)):
        parser.error("device options require --connect; omit all for the offline plan")
    return args


def main(argv=None):
    args = parse_args(argv)
    if not args.connect:
        print(json.dumps({"mode": "offline_plan", "sdk_loaded": False,
            "controller_connected": False, "motion_available": False,
            "stop_requests": "one explicit method only after stopped-state precheck",
            "methods": METHODS, "follow_stop_validated": False}, ensure_ascii=False, indent=2))
        return 0
    report = {"mode": "stationary_stop_diagnostic", "wall_time": datetime.now().astimezone().isoformat(),
        "host": args.host, "port": args.port, "method": args.method,
        "operator_present_confirmed": args.operator_ready, "motion_available": False,
        "follow_stop_validated": False, "stop_call_phase": "not_attempted",
        "scope": "Return behavior at reported rest only; not an in-motion stop or concurrency test"}
    with args.output.open("x") as file:
        def checkpoint():
            invalid = []
            safe = json_safe(report, invalid=invalid)
            safe["nonfinite_fields"] = invalid
            file.seek(0)
            file.write(json.dumps(safe, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
            file.truncate()
            file.flush()
            os.fsync(file.fileno())
        checkpoint()
        try:
            module = load_stop_native(args.runtime)
            password = getpass.getpass("SDK password (not recorded): ")
            client = module.StationaryStopClient()
            try:
                status = run_diagnostic(client, report, host=args.host, port=args.port, user=args.user,
                    password=password, method=args.method, checkpoint=checkpoint)
            finally:
                del password
        except Exception as exc:
            report["exception"] = {"type": type(exc).__name__, "message": str(exc)}
            status = 1
        checkpoint()
    safe = json.loads(args.output.read_text())
    print(json.dumps(safe, ensure_ascii=False, indent=2, allow_nan=False))
    return 1 if safe["nonfinite_fields"] else status


if __name__ == "__main__":
    sys.exit(main())
