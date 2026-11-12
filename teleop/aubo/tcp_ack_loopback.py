"""Synthetic ACK/Nagle A/B/A on 127.0.0.1 only. Never loads or calls the SDK.

The receiver deliberately requests delayed ACK (QUICKACK=0) before EACH trial;
the sender deliberately uses two small writes with NODELAY=0. This is a
mechanism demonstration, NOT a reproduction of the real cabinet workload.
"""
import argparse
import json
import math
from pathlib import Path
import queue
import socket
import statistics
import threading
import time

from .tcp_ack import TcpAckExperiment


def run_group(mode, count):
    trigger = queue.Queue()
    failures = []
    payload = b'H' + b'body' * 8
    durations = []
    with socket.socket() as server:
        server.bind(('127.0.0.1', 0))
        server.listen()
        server.settimeout(3)

        def send():
            try:
                peer, _ = server.accept()
                with peer:
                    peer.settimeout(3)
                    peer.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 0)
                    for _ in range(count):
                        if trigger.get(timeout=3) is None:
                            return
                        peer.sendall(payload[:1])
                        peer.sendall(payload[1:])
                    # Keep the TCP established until the guard has stopped.
                    trigger.get(timeout=3)
            except Exception as exc:
                failures.append(repr(exc))

        thread = threading.Thread(target=send, daemon=True)
        thread.start()
        try:
            with socket.create_connection(server.getsockname(), timeout=3) as client:
                with TcpAckExperiment(host='127.0.0.1', port=server.getsockname()[1],
                                      mode=mode) as experiment:
                    for _ in range(count):
                        # Intentionally create the condition in BOTH groups.
                        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 0)
                        started = time.monotonic()
                        trigger.put(True)
                        data = b''
                        while len(data) < len(payload):
                            block = client.recv(len(payload) - len(data))
                            if not block:
                                raise RuntimeError('Loopback peer closed early')
                            data += block
                        if data != payload:
                            raise RuntimeError('Payload changed')
                        durations.append((time.monotonic() - started) * 1000.)
                report = experiment.status()
        finally:
            trigger.put(None)
            thread.join(timeout=4)
        if thread.is_alive() or failures:
            raise RuntimeError(f'Loopback peer failed: {failures}')
    ordered = sorted(durations)
    return dict(mode=mode, samples=count, duration_ms=durations,
                median_ms=statistics.median(durations),
                p95_ms=ordered[math.ceil(.95*len(ordered))-1],
                over_30ms=sum(value > 30 for value in durations), experiment=report)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=12)
    parser.add_argument('--output', help='new JSON file in an existing runtime directory')
    args = parser.parse_args(argv)
    if not 5 <= args.samples <= 100:
        parser.error('samples must be 5..100 per group')
    target = None
    if args.output:
        target = Path(args.output).expanduser().resolve()
        # Large/runtime outputs follow local /data or the vehicle's explicit root.
        roots = (Path('/data'), Path('/home/xc/code/lcw'))
        if not any(target.is_relative_to(root) for root in roots) or not target.parent.is_dir() or target.exists():
            parser.error('output must be new and under /data or /home/xc/code/lcw')
    report = dict(scope='synthetic_loopback_only', sdk_loaded=False, cabinet_connected=False,
                  deliberately_delayed_ack=True, sender_split_writes=True,
                  warning='Synthetic timing does not establish real SDK or cabinet improvement',
                  groups=[dict(label=label, **run_group(mode, args.samples)) for label, mode in
                          (('A1', 'baseline'), ('B', 'quickack'), ('A2', 'baseline'))])
    text = json.dumps(report, indent=2, allow_nan=False) + '\n'
    if target:
        with target.open('x') as file:
            file.write(text)
    for group in report['groups']:
        print(f"{group['label']} {group['mode']}: median={group['median_ms']:.3f}ms, "
              f"p95={group['p95_ms']:.3f}ms, >30ms={group['over_30ms']}/{group['samples']}, "
              f"QUICKACK success={group['experiment']['quickack_success']}")
    if target:
        print(f'Synthetic-only report: {target}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
