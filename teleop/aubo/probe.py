"""Explicit read-only connection probe; default invocation is an offline plan."""
import argparse
from datetime import datetime
import getpass
import json
import math
from pathlib import Path
import sys
import time

from .readonly import (DEFAULT_QUERY_EXCLUSIONS, DEFAULT_QUERY_NAMES, QUERY_NAMES,
                       ReadOnlyFollower, load_native)


def record_ok(record):
    return record.get("return_code") == 0 and "validation_error" not in record


def json_safe(value, path="", invalid=None):
    if invalid is None:
        invalid = []
    if isinstance(value, float) and not math.isfinite(value):
        invalid.append(path)
        return None
    if isinstance(value, dict):
        return {k: json_safe(v, f"{path}.{k}", invalid) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v, f"{path}[{i}]", invalid) for i, v in enumerate(value)]
    return value


def read_report(follower, query_names):
    """Finite, serial queries; no retry and no automatic configuration changes."""
    records = []
    for name in query_names:
        try:
            record = follower.query(name)
        except Exception as exc:
            records.append({"operation": name, "return_code": None,
                            "exception": {"type": type(exc).__name__, "message": str(exc)}})
            break
        records.append(record)
        # Communication failures: stop probing, do not retry or reconnect.
        if record["return_code"] in (10004, 10007, 10008) or "validation_error" in record:
            break
    measured = next((r for r in reversed(records) if r["return_code"] == 0 and
                     r["operation"] in ("joint_angles", "current_waypoint", "joint_status")), None)
    if measured and len(records) == len(query_names) and all(record_ok(r) for r in records):
        try:
            q = measured["data"]["joint_positions_rad"]
            fk = dict(follower.client.fk(q))
            fk["input_joint_positions_rad"] = q
            fk["data_source"] = "sdk_local_fk"
            records.append(fk)
        except Exception as exc:
            records.append({"operation": "fk", "return_code": None,
                            "exception": {"type": type(exc).__name__, "message": str(exc)}})
    return records


def sample_feedback(follower, *, count, period_s, clock=time.monotonic, wait=time.sleep):
    """Explicit bounded serial Get sampling, never a controller-rate assertion."""
    if type(count) is not int or not 1 <= count <= 10000 or not math.isfinite(period_s) or period_s <= 0:
        raise ValueError("Explicit sample count (1..10000) and positive host period required")
    records, intervals = [], []
    previous = None
    for _ in range(count):
        started = clock()
        try:
            result = follower.query("joint_angles")
        except Exception as exc:
            records.append({"operation":"joint_angles", "return_code":None,
                            "exception":{"type":type(exc).__name__, "message":str(exc)}})
            break
        records.append(result)
        if not record_ok(result):
            break
        received = result["host_received_monotonic_s"]
        if previous is not None:
            intervals.append(received - previous)
        previous = received
        if len(records) < count:
            wait(max(0., period_s - (clock() - started)))
    return {"mode":"bounded_serial_get", "requested_host_period_s":period_s,
            "requested_samples":count, "records":records,
            "host_reception_interval_s": {"count":len(intervals),
                "min":min(intervals) if intervals else None,
                "max":max(intervals) if intervals else None,
                "mean":sum(intervals)/len(intervals) if intervals else None},
            "controller_sample_time":None,
            "notes":"Host query/reception intervals do not establish controller sampling frequency"}


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--connect", action="store_true", help="explicitly allow SDK login and Get calls")
    ap.add_argument("--host", help="actual controller address; no historic IP default")
    ap.add_argument("--port", type=int, help="verified SDK port; no default")
    ap.add_argument("--user", help="SDK account; password is prompted, never passed in argv")
    ap.add_argument("--runtime", help="compiled extension runtime directory")
    ap.add_argument("--query", choices=QUERY_NAMES, action="append",
                    help="explicit query selection; defaults exclude joint_position_limits "
                         "after a measured controller timeout (see report)")
    ap.add_argument("--output", help="optional new JSON report under /data; never overwrite")
    ap.add_argument("--feedback-samples", type=int, help="optional bounded Get sampling count, 1..10000")
    ap.add_argument("--feedback-period", type=float, help="explicit host polling period seconds; not a hardware recommendation")
    args = ap.parse_args(argv)
    if args.connect and (not args.host or not args.port or not args.user):
        ap.error("--connect requires explicit --host, --port and --user")
    if args.port is not None and not 1 <= args.port <= 65535:
        ap.error("port must be in 1..65535")
    if (args.feedback_samples is None) != (args.feedback_period is None):
        ap.error("--feedback-samples and --feedback-period must be specified together")
    if args.feedback_samples is not None and (not 1 <= args.feedback_samples <= 10000 or
            not math.isfinite(args.feedback_period) or args.feedback_period <= 0):
        ap.error("feedback sample count must be 1..10000 and host period finite/positive")
    if not args.connect and any((args.host, args.port, args.user, args.output, args.feedback_samples)):
        ap.error("connection/output options require --connect; omit them for the offline plan")
    if args.output:
        target = Path(args.output).expanduser().resolve()
        if not target.is_relative_to(Path("/data")) or not target.parent.is_dir() or target.exists():
            ap.error("--output requires a new file in an existing /data directory")
    return args


def main(argv=None):
    args = parse_args(argv)
    names = tuple(args.query) if args.query else DEFAULT_QUERY_NAMES
    selection = {"query_selection": "explicit" if args.query else "default",
                 "requested_queries": names,
                 "default_excluded_queries": dict(DEFAULT_QUERY_EXCLUSIONS) if not args.query else {}}
    if not args.connect:
        print(json.dumps({"mode": "offline_plan", "sdk_loaded": False,
                          "controller_connected": False, "motion_available": False,
                          **selection,
                          "queries": names, "feedback": "host reception time; sample time unavailable",
                          "next": "Only use --connect with verified device parameters during the read-only stage"},
                         ensure_ascii=False, indent=2))
        return 0
    report = {"mode": "read_only", "wall_time": datetime.now().astimezone().isoformat(),
              "host": args.host, "port": args.port, "motion_available": False,
              "controller_sample_time": None, "queries": [], **selection}
    follower = None
    status = 1
    try:
        module = load_native(args.runtime)
        follower = ReadOnlyFollower(module.ReadOnlyClient())
        password = getpass.getpass("SDK password (not recorded): ")
        report["login"] = follower.connect(host=args.host, port=args.port, user=args.user, password=password)
        del password
        if report["login"]["return_code"] == 0:
            report["queries"] = read_report(follower, names)
            status = 0 if all(record_ok(r) for r in report["queries"]) else 1
            if status == 0 and args.feedback_samples is not None:
                report["feedback_sampling"] = sample_feedback(follower,
                    count=args.feedback_samples, period_s=args.feedback_period)
                if not all(record_ok(r) for r in report["feedback_sampling"]["records"]):
                    status = 1
    except Exception as exc:
        report["exception"] = {"type": type(exc).__name__, "message": str(exc)}
    finally:
        try:
            report["logout"] = follower.disconnect() if follower is not None else None
            if report["logout"] and report["logout"]["return_code"] != 0:
                status = 1
        except Exception as exc:
            report["logout_exception"] = type(exc).__name__
            status = 1
    invalid = []
    safe = json_safe(report, invalid=invalid)
    safe["nonfinite_fields"] = invalid
    if invalid:
        status = 1
    text = json.dumps(safe, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    print(text, end="")
    if args.output:
        with Path(args.output).expanduser().open("x") as file:
            file.write(text)
    return status


if __name__ == "__main__":
    sys.exit(main())
