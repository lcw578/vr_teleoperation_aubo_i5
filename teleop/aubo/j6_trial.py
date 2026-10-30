"""Supervised bounded J6 experiments. Default is offline preview only.

This deliberately does not assume safe concurrent SDK calls. The operator's
physical stop is the independent fallback during synchronous SDK requests.
"""
import argparse
from datetime import datetime
import getpass
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time

from .feedback import joints
from .probe import json_safe, record_ok
from .readonly import load_native

PLAN = {"joint": 6, "delta_deg": 1., "velocity_limit_deg_s": .5,
        "acceleration_limit_deg_s2": 1., "stop_after_follow_return_s": 1.,
        "host_poll_period_s": .05, "post_stop_observation_s": 1.,
        "scope": "Single supervised experimental target, not manufacturer recommended parameters",
        "concurrent_sdk_calls": False, "automatic_home": False, "automatic_startup": False}
SEQUENCE_PLAN = dict(PLAN, delta_deg=None, offset_range_deg=[0., .8],
    target_period_s=.2, max_submission_interval_s=.35, target_count=21,
    sequence="0 -> +0.8 -> 0 degrees relative to freshly measured J6; 0.08 degree steps",
    hold_after_last_target_s=.2, stop_after_follow_return_s=None)
SEQUENCE_10HZ_PLAN = dict(SEQUENCE_PLAN, offset_range_deg=[0., 2.],
    target_period_s=.1, max_submission_interval_s=.2, target_count=101,
    sequence="0 -> +2 -> 0 degrees relative to freshly measured J6; 0.04 degree steps")
SEQUENCE_20HZ_PLAN = dict(SEQUENCE_10HZ_PLAN, target_period_s=.05, target_count=201,
    sequence="0 -> +2 -> 0 degrees relative to freshly measured J6; 0.02 degree steps")
SEQUENCE_PLANS = {5: SEQUENCE_PLAN, 10: SEQUENCE_10HZ_PLAN, 20: SEQUENCE_20HZ_PLAN}
REVERSAL_PLAN = dict(PLAN, delta_deg=None, offset_range_deg=[0., 10.],
    target_count=2, target_offsets_deg=[10., 0.],
    return_trigger_measured_deg=3., outward_observation_timeout_s=9.,
    return_observation_timeout_s=12., return_tolerance_deg=.05,
    stop_after_follow_return_s=None,
    sequence="Send +10 degrees, replace with anchor at measured +3 degrees, then hold without resending")


def sequence_offsets(rate_hz=5):
    plan = SEQUENCE_PLANS[rate_hz]
    count = (plan["target_count"] - 1) // 2
    step = plan["offset_range_deg"][1] / count
    return [round(i * step, 8) for i in list(range(count + 1)) + list(range(count - 1, -1, -1))]


def load_trial_native(runtime=None):
    root = Path(runtime or os.environ.get("AUBO_RUNTIME_DIR", "/data/VR_teleoperation_runtime"))
    readonly = load_native(root)
    manifest = json.loads((root / "j6-trial-build.json").read_text())
    library, binary = Path(manifest["release_library"]), Path(manifest["module_path"])
    if (library.resolve() != Path(readonly._isolated_sdk_handle._name).resolve() or
            hashlib.sha256(library.read_bytes()).hexdigest() != manifest["release_sha256"] or
            hashlib.sha256(binary.read_bytes()).hexdigest() != manifest["module_sha256"]):
        raise RuntimeError("Trial binary/SDK differs from reviewed build")
    name = "_aubo_j6_trial"
    module = sys.modules.get(name)
    if module is None:
        spec = importlib.util.spec_from_file_location(name, binary)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        sys.modules[name] = module
    elif Path(module.__file__).resolve() != binary.resolve():
        raise RuntimeError("A different trial binary is already loaded")
    return module


def checked_snapshot(client):
    started = time.monotonic()
    snap = client.snapshot()
    snap["host_snapshot_elapsed_s"] = time.monotonic() - started
    snap["observation_budget_exceeded"] = snap["host_snapshot_elapsed_s"] > .2
    # Preserve raw SDK results even when a diagnostic check fails.
    try:
        if not all(record_ok(snap[name]) for name in ("joints", "state")):
            raise RuntimeError("State/feedback query failed")
        joints(snap["joints"]["data"])
        if type(snap["state"]["data"]) is not int or snap["state"]["data"] not in (0, 1):
            raise RuntimeError("Unexpected controller state")
    except (KeyError, TypeError, ValueError, RuntimeError) as exc:
        snap["validation_error"] = {"type": type(exc).__name__, "message": str(exc)}
    return snap


def require_snapshot(snap, *, enforce_budget):
    if "validation_error" in snap:
        raise RuntimeError(snap["validation_error"]["message"])
    if enforce_budget and snap["observation_budget_exceeded"]:
        raise RuntimeError("Snapshot exceeded experimental 200 ms observation budget")


def run_trial(client, report, *, host, port, user, password, checkpoint):
    connected = False
    stop_needed = False
    status = 1
    try:
        report["login"] = client.login(host, port, user, password)
        connected = record_ok(report["login"])
        checkpoint()
        if not connected:
            return 1
        report["before"] = checked_snapshot(client)
        checkpoint()
        require_snapshot(report["before"], enforce_budget=True)
        if report["before"]["state"]["data"] != 0:
            report["refusal_reason"] = "controller_not_stopped"
            return 1
        # The native method re-reads q, diagnosis, real state and joint ranges.
        report["follow_phase"] = "in_flight_result_unknown"
        stop_needed = True
        checkpoint()
        report["follow"] = client.follow_j6_once()
        report["follow_phase"] = "returned"
        checkpoint()
        follow = report["follow"]
        if not follow["follow_invoked"]:
            stop_needed = False
            report["refusal_reason"] = follow["refusal_reason"]
            return 1
        if follow["return_code"] != 0:
            raise RuntimeError("Follow request failed; no retry")
        if follow["duration_s"] > .2:
            raise RuntimeError("Follow request exceeded experimental 200 ms budget")
        anchor = joints(follow["anchor_rad"])
        target = joints(follow["target_rad"])
        report["observations_before_stop"] = []
        started = time.monotonic()
        while time.monotonic() - started < PLAN["stop_after_follow_return_s"]:
            snap = checked_snapshot(client)
            report["observations_before_stop"].append(snap)
            checkpoint()
            require_snapshot(snap, enforce_budget=True)
            measured = joints(snap["joints"]["data"])
            # Experimental movement envelope, not a general following-error policy.
            if any(abs(q - a) > math.radians(.1 if i < 5 else 1.1)
                   for i, (q, a) in enumerate(zip(measured, anchor))):
                raise RuntimeError("Measured motion outside trial envelope")
            if time.monotonic() - snap["joints"]["host_received_monotonic_s"] > .2:
                raise RuntimeError("Delayed feedback; stop trial")
            time.sleep(PLAN["host_poll_period_s"])
        report["stop_phase"] = "in_flight_result_unknown"
        checkpoint()
        stop_needed = False  # Exactly one request; exceptions preserve unknown outcome.
        report["stop"] = client.stop_once()
        report["stop_phase"] = "returned"
        checkpoint()
        report["observations_after_stop"] = []
        until = time.monotonic() + PLAN["post_stop_observation_s"]
        while time.monotonic() < until:
            snap = checked_snapshot(client)
            report["observations_after_stop"].append(snap)
            checkpoint()
            # Stop already requested: keep late observations and flag degradation.
            require_snapshot(snap, enforce_budget=False)
            time.sleep(PLAN["host_poll_period_s"])
        report["after"] = checked_snapshot(client)
        checkpoint()
        require_snapshot(report["after"], enforce_budget=False)
        q_after = joints(report["after"]["joints"]["data"])
        report["joint_delta_deg"] = [math.degrees(q - a) for q, a in zip(q_after, anchor)]
        report["remaining_to_target_deg"] = [math.degrees(t - q) for t, q in zip(target, q_after)]
        report["physical_stop_acceptance"] = "requires review of observations and operator report"
        late = any(s["observation_budget_exceeded"] for s in
                   report["observations_after_stop"] + [report["after"]])
        report["post_stop_observation_degraded"] = late
        status = 0 if record_ok(report["stop"]) and report["after"]["state"]["data"] == 0 and not late else 1
    except BaseException as exc:
        report["exception"] = {"type": type(exc).__name__, "message": str(exc)}
        status = 1
    finally:
        if connected and stop_needed:
            report["stop_phase"] = "in_flight_result_unknown"
            try:
                checkpoint()
            except BaseException as exc:
                report["checkpoint_exception"] = type(exc).__name__
            try:
                stop_needed = False
                report["stop"] = client.stop_once()
                report["stop_phase"] = "returned"
            except BaseException as exc:
                report["stop_exception"] = {"type": type(exc).__name__, "message": str(exc)}
            try:
                report["after_fault"] = checked_snapshot(client)
            except BaseException as exc:
                report["feedback_exception"] = type(exc).__name__
        if connected:
            try:
                report["logout"] = client.logout()
                if not record_ok(report["logout"]):
                    status = 1
            except BaseException as exc:
                report["logout_exception"] = type(exc).__name__
                status = 1
        checkpoint()
    return status


def run_sequence(client, report, *, host, port, user, password, checkpoint, reversal=False, rate_hz=5):
    """One bounded real sequence; serial calls, no catch-up, retry or auto recovery."""
    if reversal and rate_hz != 5:
        raise ValueError("Cannot combine reversal and continuous sequence")
    sequence_plan = SEQUENCE_PLANS[rate_hz]
    connected = stop_needed = False
    status = 1
    try:
        report["login"] = client.login(host, port, user, password)
        connected = record_ok(report["login"])
        checkpoint()
        if not connected:
            return 1
        report["before"] = checked_snapshot(client)
        checkpoint()
        require_snapshot(report["before"], enforce_budget=True)
        if report["before"]["state"]["data"] != 0:
            report["refusal_reason"] = "controller_not_stopped"
            return 1
        report["prepare"] = client.prepare()
        checkpoint()
        if not record_ok(report["prepare"]) or not report["prepare"]["prepared"]:
            report["refusal_reason"] = report["prepare"].get("refusal_reason", "prepare_failed")
            return 1
        anchor = joints(report["prepare"]["anchor_rad"])
        report["attempts"] = []
        report["observations"] = []

        def observe(phase, index):
            snap = checked_snapshot(client)
            snap["phase"] = phase
            snap["after_attempt_index"] = index
            report["observations"].append(snap)
            checkpoint()
            require_snapshot(snap, enforce_budget=True)
            q = joints(snap["joints"]["data"])
            upper_deg = 10.1 if reversal else sequence_plan["offset_range_deg"][1] + .1
            if any(abs(q[i] - anchor[i]) > math.radians(.1) for i in range(5)) or not (
                    -math.radians(.1) <= q[5] - anchor[5] <= math.radians(upper_deg)):
                raise RuntimeError("Measured motion outside sequence envelope")
            if time.monotonic() - snap["joints"]["host_received_monotonic_s"] > .2:
                raise RuntimeError("Feedback aged beyond 200 ms during observation")
            return math.degrees(q[5] - anchor[5]), snap

        def observe_until(phase, index, timeout, predicate):
            until = time.monotonic() + timeout
            while time.monotonic() < until:
                delta, snap = observe(phase, index)
                if predicate(delta, snap):
                    return
                time.sleep(PLAN["host_poll_period_s"])
            raise RuntimeError(phase + " observation deadline exceeded; stop without retry")

        last_start = None
        offsets = REVERSAL_PLAN["target_offsets_deg"] if reversal else sequence_offsets(rate_hz)
        for index, offset in enumerate(offsets):
            if reversal and index == 1:
                observe_until("outward_before_retarget", 0,
                    REVERSAL_PLAN["outward_observation_timeout_s"],
                    lambda delta, snap: delta >= REVERSAL_PLAN["return_trigger_measured_deg"])
            elif last_start is not None:
                time.sleep(max(0., last_start + sequence_plan["target_period_s"] - time.monotonic()))
                if time.monotonic() - last_start > sequence_plan["max_submission_interval_s"]:
                    raise RuntimeError("Submission interval exceeded trial budget; no catch-up")
            phase = ("outward" if index == 0 else "return") if reversal else ("outward" if index <= len(offsets) // 2 else "return")
            attempt = {"index": index, "phase": phase,
                       "requested_offset_deg": offset, "outcome": "in_flight_result_unknown",
                       "host_intent_monotonic_s": time.monotonic()}
            report["attempts"].append(attempt)
            report["follow_phase"] = "in_flight_result_unknown"
            stop_needed = True
            checkpoint()
            result = (client.send_outward() if index == 0 else client.return_to_anchor()) if reversal else client.send_offset(offset)
            attempt["sdk_result"] = result
            attempt["outcome"] = "sdk_accepted" if record_ok(result) else "sdk_rejected"
            report["follow_phase"] = "returned"
            last_start = result["host_started_monotonic_s"]
            checkpoint()
            if not record_ok(result) or result["duration_s"] > .2:
                raise RuntimeError("Follow failed or exceeded 200 ms budget; no retry")
            observe("after_target", index)
        if reversal:
            observe_until("hold_anchor_without_resending", 1,
                REVERSAL_PLAN["return_observation_timeout_s"],
                lambda delta, snap: abs(delta) <= REVERSAL_PLAN["return_tolerance_deg"] and snap["state"]["data"] == 0)
            report["return_to_anchor_observed"] = True
        else:
            time.sleep(sequence_plan["hold_after_last_target_s"])
        report["before_stop"] = checked_snapshot(client)
        checkpoint()
        require_snapshot(report["before_stop"], enforce_budget=True)
        report["stop_phase"] = "in_flight_result_unknown"
        checkpoint()
        stop_needed = False
        report["stop"] = client.stop_once()
        report["stop_phase"] = "returned"
        checkpoint()
        report["observations_after_stop"] = []
        until = time.monotonic() + PLAN["post_stop_observation_s"]
        while time.monotonic() < until:
            snap = checked_snapshot(client)
            report["observations_after_stop"].append(snap)
            checkpoint()
            require_snapshot(snap, enforce_budget=False)
            time.sleep(PLAN["host_poll_period_s"])
        report["after"] = checked_snapshot(client)
        checkpoint()
        require_snapshot(report["after"], enforce_budget=False)
        report["joint_delta_deg"] = [math.degrees(q - a) for q, a in
            zip(joints(report["after"]["joints"]["data"]), anchor)]
        late = any(s["observation_budget_exceeded"] for s in
                   report["observations_after_stop"] + [report["after"]])
        report["post_stop_observation_degraded"] = late
        report["physical_acceptance"] = "requires trace review and operator observation"
        status = 0 if record_ok(report["stop"]) and report["after"]["state"]["data"] == 0 and not late else 1
    except BaseException as exc:
        report["exception"] = {"type": type(exc).__name__, "message": str(exc)}
        status = 1
    finally:
        if connected and stop_needed:
            report["stop_phase"] = "in_flight_result_unknown"
            try:
                checkpoint()
            except BaseException as exc:
                report["checkpoint_exception"] = type(exc).__name__
            try:
                stop_needed = False
                report["stop"] = client.stop_once()
                report["stop_phase"] = "returned"
            except BaseException as exc:
                report["stop_exception"] = {"type": type(exc).__name__, "message": str(exc)}
            try:
                report["after_fault"] = checked_snapshot(client)
            except BaseException as exc:
                report["feedback_exception"] = type(exc).__name__
        if connected:
            try:
                report["logout"] = client.logout()
                if not record_ok(report["logout"]):
                    status = 1
            except BaseException as exc:
                report["logout_exception"] = type(exc).__name__
                status = 1
        checkpoint()
    return status


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="allow exactly the printed J6 experiment")
    parser.add_argument("--operator-ready", action="store_true")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--sequence", action="store_true", help="bounded 5 Hz J6 outward/return experiment")
    mode.add_argument("--sequence-10hz", action="store_true", help="bounded 10 Hz J6 0..2 degree outward/return experiment")
    mode.add_argument("--sequence-20hz", action="store_true", help="bounded 20 Hz J6 0..2 degree outward/return experiment")
    mode.add_argument("--reversal-10deg", action="store_true", help="fixed +10 degree target, retarget anchor at measured +3 degrees")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--user")
    parser.add_argument("--output")
    parser.add_argument("--runtime")
    args = parser.parse_args(argv)
    rate_hz = 20 if args.sequence_20hz else (10 if args.sequence_10hz else 5)
    continuous = args.sequence or args.sequence_10hz or args.sequence_20hz
    plan = REVERSAL_PLAN if args.reversal_10deg else (SEQUENCE_PLANS[rate_hz] if continuous else PLAN)
    if not args.execute:
        if any((args.operator_ready, args.host, args.port, args.user, args.output)):
            parser.error("Device options require --execute")
        print(json.dumps({"mode": "offline_preview", "sdk_loaded": False,
                          "controller_connected": False, "plan": plan,
                          "offsets_deg": REVERSAL_PLAN["target_offsets_deg"] if args.reversal_10deg else (sequence_offsets(rate_hz) if continuous else None)}, indent=2))
        return 0
    if not all((args.operator_ready, args.host, args.port, args.user, args.output)):
        parser.error("--execute requires --operator-ready, host, port, user and output")
    if not 1 <= args.port <= 65535:
        parser.error("Invalid port")
    output = Path(args.output).expanduser().resolve()
    if not output.is_relative_to(Path("/data")) or not output.parent.is_dir() or output.exists():
        parser.error("New report under an existing /data directory required")
    report = {"mode": "supervised_j6_reversal_10deg" if args.reversal_10deg else (f"supervised_j6_sequence_{rate_hz}hz" if continuous else "supervised_j6_micro_trial"),
              "wall_time": datetime.now().astimezone().isoformat(),
              "host": args.host, "port": args.port, "plan": plan,
              "follow_phase": "not_attempted", "stop_phase": "not_attempted"}
    with output.open("x") as file:
        def checkpoint():
            invalid = []
            safe = json_safe(report, invalid=invalid)
            safe["nonfinite_fields"] = invalid
            file.seek(0); file.write(json.dumps(safe, indent=2, allow_nan=False) + "\n")
            file.truncate(); file.flush(); os.fsync(file.fileno())
        checkpoint()
        try:
            module = load_trial_native(args.runtime)
            password = getpass.getpass("SDK password (not recorded): ")
            try:
                runner = run_sequence if continuous or args.reversal_10deg else run_trial
                client = module.J6ReversalTrial() if args.reversal_10deg else (module.J6StreamTrial(rate_hz) if continuous else module.J6Trial())
                status = runner(client, report, host=args.host, port=args.port, user=args.user,
                    password=password, checkpoint=checkpoint,
                    **({"reversal": True} if args.reversal_10deg else ({"rate_hz": rate_hz} if continuous else {})))
            finally:
                del password
        except BaseException as exc:
            report["exception"] = {"type": type(exc).__name__, "message": str(exc)}
            status = 1
        checkpoint()
    print(json.dumps(json.loads(output.read_text()), indent=2))
    return status


if __name__ == "__main__":
    sys.exit(main())
