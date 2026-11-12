"""Explicit stationary ACK experiment. Robot API scope: login/Get/logout only.

Default is a file-free offline plan. Each invocation owns one fresh SDK client.
Capture starts AFTER authentication; no reconnect and no motion API is used.
"""
import argparse
from datetime import datetime
import getpass
import json
import math
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import tempfile
import time

from .readonly import load_native
from .tcp_ack import MODES, TcpAckExperiment
from .wire_timing import FrameStream, packets


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--connect', action='store_true')
    parser.add_argument('--host')
    parser.add_argument('--port', type=int)
    parser.add_argument('--user')
    parser.add_argument('--runtime', default=os.environ.get('AUBO_RUNTIME_DIR', '/data/VR_teleoperation_runtime'))
    parser.add_argument('--mode', choices=MODES, default='baseline')
    parser.add_argument('--duration', type=int, default=90)
    parser.add_argument('--label', choices=('A1', 'B', 'A2', 'NODELAY', 'COMBINED'), default='A1')
    parser.add_argument('--capture-interface', help='optional capture after login; needs prior sudo -v')
    args = parser.parse_args(argv)
    if not 30 <= args.duration <= 120:
        parser.error('duration must be 30..120 seconds')
    if args.connect and (not args.host or not args.port or not args.user):
        parser.error('--connect requires explicit --host, --port, --user')
    if args.port is not None and not 1 <= args.port <= 65535:
        parser.error('port must be 1..65535')
    expected = {'A1': 'baseline', 'A2': 'baseline', 'B': 'quickack',
                'NODELAY': 'nodelay', 'COMBINED': 'combined'}
    if expected[args.label] != args.mode:
        parser.error('label and intervention mode do not agree')
    if args.connect:
        root = Path(args.runtime).expanduser().resolve()
        if not root.is_dir() or not any(root.is_relative_to(p) for p in
                                        (Path('/data'), Path('/home/xc/code/lcw'))):
            parser.error('runtime must exist under /data or /home/xc/code/lcw')
        if args.capture_interface and not (Path('/sys/class/net') / args.capture_interface).is_dir():
            parser.error('capture interface does not exist')
    return args


def _write_event(file, event, **values):
    record = dict(event=event, monotonic_s=time.monotonic(), wall_time_ns=time.time_ns(), **values)
    file.write(json.dumps(record, allow_nan=False) + '\n')
    file.flush()


def _start_capture(args, directory, report):
    if not args.capture_interface:
        return None, None
    # Only this exact authenticated TCP stream. No login packets or reconnect.
    local_host, local_port = report['local']
    packet_filter = (f'tcp and host {args.host} and port {args.port} '
                     f'and host {local_host} and port {local_port}')
    log = (directory / 'capture.log').open('x')
    command = ['sudo', '-n', 'timeout', '--signal=INT', '--kill-after=3', str(args.duration + 30),
               'tcpdump', '-i', args.capture_interface, '-nn', '-s', '0', '-B', '4096', '-U',
               '--immediate-mode',
               '-Z', getpass.getuser(), '-w', str(directory / 'wire.pcap'), packet_filter]
    process = None
    try:
        process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=log)
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('Capture exited before sampling; see capture.log')
            if 'listening on ' in (directory / 'capture.log').read_text():
                return process, log
            time.sleep(.05)
        raise RuntimeError('Capture did not become ready; see capture.log')
    except BaseException:
        if process is not None:
            _stop_capture(process, directory / 'wire.pcap')
        log.close()
        raise


def _stop_capture(process, capture_path):
    if process is None:
        return None
    if process.poll() is None:
        # sudo/timeout may not relay signals sent to their outer PID. tcpdump
        # drops privileges with -Z; signal only our UID's exact capture writer.
        # Do not signal unrelated captures or depend on inherited process groups.
        for path in Path('/proc').iterdir():
            if not path.name.isdigit():
                continue
            try:
                if path.stat().st_uid != os.getuid():
                    continue
                args = (path / 'cmdline').read_bytes().split(b'\0')
                if (args and Path(os.fsdecode(args[0])).name == 'tcpdump' and b'-w' in args
                        and args[args.index(b'-w') + 1] == os.fsencode(capture_path)):
                    os.kill(int(path.name), signal.SIGINT)
            except (OSError, ValueError, IndexError):
                continue
        process.wait(timeout=5)
    return process.returncode


def _capture_queries(path, host, expected):
    """Require every logged Get request AND response, not just zero kernel drops.

    This launcher captures exactly one TCP stream and never reconnects. It uses
    Get commands 10 and 26 only, with no subscription/configuration traffic.
    """
    streams = {'pc': FrameStream(), 'cabinet': FrameStream()}
    counts = {'pc': {10: 0, 26: 0}, 'cabinet': {10: 0, 26: 0}}
    for packet in packets(path):
        direction = 'cabinet' if packet['src'] == host else 'pc'
        for frame in streams[direction].feed(packet['seq'], packet['payload'], packet['wall_s']):
            command = frame['command']
            if command not in counts[direction]:
                raise RuntimeError(f'Unexpected command in stationary query capture: {command}')
            counts[direction][command] += 1
    wanted = {10: expected.get('joint_angles', 0), 26: expected.get('robot_state', 0)}
    complete = all(counts[direction] == wanted and not streams[direction].waiting
                   and not streams[direction].buffer for direction in streams)
    return dict(complete=complete, expected=wanted, captured=counts,
                stream_stats={key: dict(value.stats) for key, value in streams.items()})


def sample_queries(client, experiment, duration_s, file, *, clock=time.monotonic, wait=time.sleep):
    """25 total Get calls/s requested, serial and no catch-up; every 5th is state."""
    started = clock()
    progress = started
    count = 0
    latencies = []
    operations = {}
    while clock() - started < duration_s:
        report = experiment.status()
        if not report['alive'] or report['error'] is not None:
            raise RuntimeError(f'ACK experiment stopped unexpectedly: {report}')
        operation = 'robot_state' if count % 5 == 0 else 'joint_angles'
        before = clock()
        result = client.query(operation)
        _write_event(file, 'query', record=result)
        if result.get('return_code') != 0 or 'validation_error' in result:
            raise RuntimeError(f'Read-only {operation} query failed; no retry')
        if operation == 'robot_state' and result['data'] != 0:
            raise RuntimeError('Controller no longer reports RobotStopped; stationary trial ended')
        latencies.append(result['duration_s'] * 1000.)
        operations[operation] = operations.get(operation, 0) + 1
        count += 1
        now = clock()
        if now - progress >= 5:
            _write_event(file, 'ack_status', status=experiment.status())
            print(f'Sampling {now-started:.0f}/{duration_s}s: {count} Get calls, '
                  f'QUICKACK success={report["quickack_success"]}', flush=True)
            progress = now
        wait(max(0., .04 - (clock() - before)))
    ordered = sorted(latencies)
    return dict(query_count=count, operations=operations, elapsed_s=clock() - started,
                sdk_median_ms=statistics.median(latencies) if latencies else None,
                sdk_p95_ms=ordered[math.ceil(.95*len(ordered))-1] if ordered else None,
                sdk_40_60ms=sum(40 <= value <= 60 for value in latencies),
                requested_total_get_hz=25, query_policy='serial_4_joint_angles_1_robot_state',
                waypoint_push_registered=False,
                caveat='Get latency is not Follow latency or controller sampling rate')


def main(argv=None):
    args = parse_args(argv)
    if not args.connect:
        print(json.dumps(dict(mode='offline_plan', intervention=args.mode, label=args.label,
                              sdk_loaded=False, controller_connected=False, motion_available=False,
                              duration_s=args.duration, query_policy='25Hz serial Get requests',
                              next='Use the stationary launcher at the vehicle terminal; no headset required'),
                         indent=2))
        return 0
    directory = Path(tempfile.mkdtemp(prefix=f'ack-stationary-{args.label}-', dir=args.runtime))
    print(f'Evidence directory: {directory}', flush=True)
    summary = dict(scope='stationary_get_only', label=args.label, mode=args.mode,
                   wall_time=datetime.now().astimezone().isoformat(), motion_api_called=False,
                   controller_follow_improvement_verified=False, completed=False)
    client = experiment = process = capture_log = None
    logged_in = False
    status = 1
    with (directory / 'events.jsonl').open('x') as events:
        try:
            module = load_native(args.runtime)
            client = module.ReadOnlyClient()
            password = getpass.getpass('SDK password (not recorded): ')
            login = client.login(args.host, args.port, args.user, password)
            del password
            _write_event(events, 'login_result', record=login)
            if login['return_code'] != 0:
                raise RuntimeError('SDK login failed; no retry')
            logged_in = True
            state = client.query('robot_state')
            _write_event(events, 'initial_state', record=state)
            if state['return_code'] != 0 or state['data'] != 0:
                raise RuntimeError('Controller must report RobotStopped before this trial')
            experiment = TcpAckExperiment(host=args.host, port=args.port, mode=args.mode).start()
            summary['ack_initial'] = experiment.status()
            process, capture_log = _start_capture(args, directory, summary['ack_initial'])
            _write_event(events, 'prepare', status=experiment.status(), capture_ready=process is not None)
            print('READY: stationary Get sampling only; no Grip/Y/X or headset needed.', flush=True)
            summary['sampling'] = sample_queries(client, experiment, args.duration, events)
            if process is not None and process.poll() is not None:
                raise RuntimeError('Capture exited during sampling')
            summary['completed'] = True
            status = 0
        except KeyboardInterrupt:
            summary['exception'] = dict(type='KeyboardInterrupt', message='Operator interrupted')
        except Exception as exc:
            summary['exception'] = dict(type=type(exc).__name__, message=str(exc))
        finally:
            worker_stopped = True
            if experiment is not None:
                try:
                    summary['ack_final'] = experiment.stop()
                    if summary['ack_final']['stop_reason'] != 'requested' or summary['ack_final']['error']:
                        status = 1
                except Exception as exc:
                    worker_stopped = False
                    summary['ack_cleanup_error'] = str(exc)
                    status = 1
            try:
                summary['capture_exit_code'] = _stop_capture(process, directory / 'wire.pcap')
                if process is not None and summary['capture_exit_code'] not in (0, 130):
                    status = 1
            except Exception as exc:
                summary['capture_cleanup_error'] = str(exc)
                status = 1
            if capture_log is not None:
                capture_log.close()
                capture_text = (directory / 'capture.log').read_text()
                dropped = re.search(r'(\d+) packets dropped by kernel', capture_text)
                summary['capture_kernel_dropped'] = int(dropped.group(1)) if dropped else None
                if summary['capture_kernel_dropped'] != 0:
                    status = 1
                if 'sampling' in summary:
                    try:
                        summary['capture_queries'] = _capture_queries(
                            directory / 'wire.pcap', args.host, summary['sampling']['operations'])
                        if not summary['capture_queries']['complete']:
                            status = 1
                    except Exception as exc:
                        summary['capture_validation_error'] = str(exc)
                        status = 1
            if logged_in and worker_stopped:
                try:
                    summary['logout'] = client.logout()
                    if summary['logout']['return_code'] != 0:
                        status = 1
                except Exception as exc:
                    summary['logout_error'] = str(exc)
                    status = 1
            summary['completed'] = summary['completed'] and status == 0
            _write_event(events, 'session_end', summary=summary)
    with (directory / 'summary.json').open('x') as file:
        json.dump(summary, file, indent=2, allow_nan=False)
        file.write('\n')
    print(f'Finished: completed={summary["completed"]}; report={directory / "summary.json"}', flush=True)
    return status


if __name__ == '__main__':
    raise SystemExit(main())
