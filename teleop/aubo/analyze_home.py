"""Reproduce home-return timelines from saved files only; no SDK/device imports."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from .analyze_follow import timing


def transitions(lines):
    for line in lines:
        if 'Hardware transition: ' in line:
            yield json.loads(line.split('Hardware transition: ', 1)[1])


def _unique(values, keys):
    seen = set()
    for value in values:
        identity = tuple(value.get(key) for key in keys)
        if identity not in seen:
            seen.add(identity)
            yield value


def summarize(rows, hardware):
    """Group lifecycle events by request, not by the number of state messages.

    Snapshot fields repeat in stdout. Deduplicate calls and first-cause latches,
    preserving SDK error codes. No missing joint sample is reconstructed.
    """
    groups = {}
    for row in rows:
        if row.get('kind') == 'home_return' and row.get('requested_monotonic_s') is not None:
            groups.setdefault(row['requested_monotonic_s'], []).append(row)
    requests = sorted(groups)
    attempts = [r['attempt'] for r in rows
                if r.get('kind') == 'sdk_command' and r.get('phase') == 'returned']
    attempts += [h['last_attempt'] for h in hardware if h.get('last_attempt')]
    calls = list(_unique((dict(a['sdk_result'], session=a.get('session'),
        control_source=a.get('control_source'), outcome=a.get('outcome'))
        for a in attempts if a.get('sdk_result')), (
            'session', 'host_started_monotonic_s', 'host_received_monotonic_s', 'operation')))
    stops = list(_unique((h['last_stop'] for h in hardware if h.get('last_stop')), (
        'host_started_monotonic_s', 'host_received_monotonic_s', 'operation')))
    first_stops = list(_unique((h['first_stop_trigger'] for h in hardware
        if h.get('first_stop_trigger')), ('host_monotonic_s', 'reason')))
    fatals = list(_unique((h['first_fatal'] for h in hardware if h.get('first_fatal')),
        ('host_monotonic_s', 'reason')))
    rounds = []
    for i, requested in enumerate(requests):
        end = requests[i+1] if i+1 < len(requests) else float('inf')
        events = sorted(groups[requested], key=lambda r: r['host_monotonic_s'])
        final = events[-1]
        def inside(value, key):
            stamp = value.get(key)
            return isinstance(stamp, (int, float)) and requested <= stamp < end
        sent = sorted((c for c in calls if c.get('control_source') == 'home_return'
            and inside(c, 'host_started_monotonic_s')), key=lambda c: c['host_started_monotonic_s'])
        intervals = [b['host_started_monotonic_s']-a['host_started_monotonic_s']
            for a, b in zip(sent, sent[1:]) if a['session'] == b['session']]
        failure = final.get('failure_evidence')
        limitations = []
        if final.get('error') == 'home_tracking_error' and not (
                failure and failure.get('decision_feedback')):
            limitations.append('trigger_feedback_not_saved; pre_send_is_not_trigger_feedback')
        if final['state'] == 'stopping':
            limitations.append('at_target_is_not_stop_confirmed; lifecycle_unfinished')
        if not sent and any(e['state'] == 'moving' for e in events):
            limitations.append('sdk_calls_not_saved_or_recording_paused')
        round_stops = [s for s in stops if inside(s, 'host_started_monotonic_s')]
        if any(s.get('return_code') == 10007 for s in round_stops):
            limitations.append('10007_request_response_timeout; pcap_not_part_of_this_audit; transport_location_unresolved')
        rounds.append(dict(requested_monotonic_s=requested,
            trigger=events[0].get('trigger', 'not_saved'),
            target_speed_deg_s=events[0].get('target_speed_deg_s'),
            target_acceleration_deg_s2=events[0].get('target_acceleration_deg_s2'),
            motion_started=any(e['state'] == 'moving' for e in events),
            lifecycle=[dict(state=e['state'], host_monotonic_s=e['host_monotonic_s'],
                error=e.get('error'), distance_to_home_max_deg=e.get('max_error_deg')) for e in events],
            final_state=final['state'], error=final.get('error'),
            completed_monotonic_s=final.get('completed_monotonic_s'),
            # This value is distance to the final home, NOT tracking error.
            final_distance_to_home_max_deg=final.get('max_error_deg'),
            observed_home_sdk_calls=len(sent), observed_home_sdk_sessions=sorted(
                {c['session'] for c in sent if c['session'] is not None}),
            observed_send_interval=timing(intervals),
            observed_sdk_duration=timing(c.get('duration_s') for c in sent),
            last_observed_home_sdk_call=sent[-1] if sent else None,
            first_stop_triggers=[s for s in first_stops if inside(s, 'host_monotonic_s')],
            first_fatals=[f for f in fatals if inside(f, 'host_monotonic_s')],
            stop_calls=round_stops, failure_evidence=failure, limitations=limitations))
    return dict(schema='aubo_home_audit_v1', request_count=len(rounds),
        moving_request_count=sum(r['motion_started'] for r in rounds),
        final_states=dict(Counter(r['final_state'] for r in rounds)),
        errors=dict(Counter(r['error'] for r in rounds if r['error'])), rounds=rounds,
        interpretation='Observed files only; paused recordings can omit SDK calls. '
            'Do not pool different deployed versions into one success rate.')


def file_evidence(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024*1024), b''):
            digest.update(block)
    return dict(path=str(path.resolve()), size_bytes=path.stat().st_size,
                sha256=digest.hexdigest())


def audit(session, log):
    streams = session/'streams.jsonl'
    with streams.open() as source:
        rows = [json.loads(line) for line in source if line.strip()]
    with log.open() as source:
        hardware = list(transitions(source))
    report = summarize(rows, hardware)
    paths = [streams, log]
    for name in ('session_status.json', 'manifest.json'):
        path = session/name
        if path.exists():
            paths.append(path)
            report[name.removesuffix('.json')] = json.loads(path.read_text())
    report['sources'] = [file_evidence(path) for path in paths]
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--output', type=Path, help='Default: JSON on stdout; never edits input')
    args = parser.parse_args(argv)
    content = json.dumps(audit(args.session, args.log), ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    if args.output:
        args.output.write_text(content)
    else:
        print(content, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
