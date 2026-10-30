"""Offline joint/time diagnostic summary. Never imports the SDK or opens a robot."""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import statistics


def timing(values):
    """Milliseconds; nearest-rank p95, no assumption of a normal distribution."""
    values = sorted(v for v in values if isinstance(v, (int, float)) and math.isfinite(v) and v >= 0)
    if not values:
        return dict(count=0)
    return dict(count=len(values), min_ms=values[0]*1000,
        mean_ms=statistics.mean(values)*1000, median_ms=statistics.median(values)*1000,
        p95_ms=values[math.ceil(.95*len(values))-1]*1000, max_ms=values[-1]*1000)


def summarize(rows):
    sends = [r['attempt'] for r in rows if r.get('event') == 'send_returned']
    # A release/re-engagement gap is not a command interval. Also exclude any
    # locally refused request that never entered the SDK.
    sessions = defaultdict(list)
    invoked = []
    for attempt in sends:
        result = attempt.get('sdk_result', {})
        if result.get('sdk_invoked') is not False and isinstance(result.get('host_started_monotonic_s'), (int, float)):
            sessions[attempt['session']].append(result['host_started_monotonic_s'])
            invoked.append(attempt)
    intervals = [b-a for times in sessions.values() for a,b in zip(times,times[1:]) if b > a]
    queries = [r for r in rows if r.get('event') == 'feedback']
    # Parallel feedback reads only copy a cache. Count the actual B requests,
    # not repeated appearances of the same result in motion-loop snapshots.
    parallel_queries = [r for r in rows if r.get('event') == 'state_query_returned']
    if parallel_queries:
        queries = [dict(r, snapshot=dict(state=r['result'])) for r in parallel_queries]
    else:
        queries = [r for r in queries if r['snapshot']['state'].get('source') != 'independent_sdk_state_cache']
    active_queries = [r for r in queries if r.get('phase') == 'active']
    stop_pairs = []
    pending = None
    for row in rows:
        event = row.get('event')
        if event == 'stop_requested':
            pending = dict(reason=row['trigger']['reason'], requested_s=row['host_monotonic_s'])
        elif pending is not None and event in ('slow_stop_call_started','stop_returned','stop_confirmed'):
            key = {'slow_stop_call_started':'call_started_s','stop_returned':'returned_s',
                   'stop_confirmed':'confirmed_s'}[event]
            pending[key] = row['host_monotonic_s']
            if event == 'stop_confirmed':
                stop_pairs.append(pending)
                pending = None
    if pending is not None:
        stop_pairs.append(pending)
    end = next((r for r in reversed(rows) if r.get('event') == 'session_end'), {})
    final = end.get('final_status', {})
    start = next((r for r in rows if r.get('event') == 'session_start'), {})
    query_durations = lambda items: [r['snapshot']['state']['duration_s'] for r in items]
    return dict(
        source_kind='host_joint_time_diagnostic_not_controller_sample_clock',
        session_mode=start.get('mode'), dataset_recording=start.get('dataset_recording'),
        command_mode=start.get('command_mode', final.get('command_mode')),
        events=dict(Counter(r.get('event', 'unknown') for r in rows)),
        session_end_present=bool(end), dropped_events=end.get('dropped_events'),
        requested_hz=final.get('requested_frequency_hz'),
        invoked_follow_count=len(invoked),
        send_outcomes=dict(Counter(a.get('outcome') for a in sends)),
        within_engagement_send_intervals=timing(intervals),
        aggregate_within_engagement_hz=(1/statistics.mean(intervals) if intervals else None),
        follow_call_duration=timing([a['sdk_result']['duration_s'] for a in invoked]),
        state_query_duration=timing(query_durations(queries)),
        active_state_query_duration=timing(query_durations(active_queries)),
        candidate_age_at_sdk_start=timing([a['sdk_result']['host_started_monotonic_s']-a['generated_monotonic_s'] for a in invoked]),
        slow_stop_call_duration=timing([r['result']['duration_s'] for r in rows if r.get('event') == 'stop_returned']),
        stop_request_to_call_start=timing([p['call_started_s']-p['requested_s'] for p in stop_pairs if 'call_started_s' in p]),
        stop_request_to_confirm=timing([p['confirmed_s']-p['requested_s'] for p in stop_pairs if 'confirmed_s' in p]),
        stops=stop_pairs,
        final_status={k:final.get(k) for k in ('fatal','stop_confirmed','stop_result_unknown','state_query_policy',
            'active_state_query_period_s','idle_state_query_period_s')})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='existing diagnostic JSONL')
    parser.add_argument('--output', type=Path, help='optional new summary JSON under /data')
    args = parser.parse_args(argv)
    output = args.output.expanduser().resolve() if args.output else None
    if output and (Path('/data') not in output.parents or not output.parent.is_dir()):
        parser.error('Output requires an existing parent directory under /data')
    rows = [json.loads(line) for line in args.input.read_text().splitlines() if line.strip()]
    summary = dict(input=str(args.input.resolve()), **summarize(rows))
    text = json.dumps(summary, ensure_ascii=False, allow_nan=False, indent=2)+'\n'
    if output:
        with output.open('x') as stream:
            stream.write(text)
    print(text, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
