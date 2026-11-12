"""Opt-in Linux TCP ACK experiment on an existing socket in this process.

No connect, send, recv, SDK loading, private SDK fields or libc interposition.
A duplicate pins the socket identity; the original descriptor is checked on
every tick. Stop this experiment BEFORE SDK logout. Each A/B/A run must use a
fresh connection: QUICKACK is a transient hint, not a restorable boolean mode.
"""
import ipaddress
import math
import os
from pathlib import Path
import socket
import stat
import threading
import time
from collections import deque


MODES = ("baseline", "quickack", "nodelay", "combined")


def _identity(fd):
    value = os.fstat(fd)
    if not stat.S_ISSOCK(value.st_mode):
        raise ValueError("Descriptor is not a socket")
    return value.st_dev, value.st_ino


def _pin(fd, host, port):
    """Pin before inspecting, then check the source still denotes that socket."""
    before = _identity(fd)
    duplicate = os.dup(fd)
    try:
        sock = socket.socket(fileno=duplicate)
    except BaseException:
        os.close(duplicate)
        raise
    try:
        if _identity(duplicate) != before or _identity(fd) != before:
            raise RuntimeError("Source socket changed during identification")
        if (sock.family != socket.AF_INET or
                sock.getsockopt(socket.SOL_SOCKET, socket.SO_TYPE) != socket.SOCK_STREAM or
                sock.getsockopt(socket.SOL_SOCKET, socket.SO_PROTOCOL) != socket.IPPROTO_TCP or
                sock.getpeername() != (host, port)):
            raise ValueError("Descriptor does not match the exact IPv4 TCP peer")
        # Linux TCP_ESTABLISHED == 1. A connected UDP or closing TCP is refused.
        if sock.getsockopt(socket.IPPROTO_TCP, socket.TCP_INFO, 1)[0] != 1:
            raise ValueError("Target TCP socket is not established")
        return sock, before
    except BaseException:
        sock.close()
        raise


def _select(host, port, source_fd):
    if source_fd is not None:
        sock, identity = _pin(source_fd, host, port)
        return source_fd, sock, identity
    matches = []
    # Materialize first: our own duplicate descriptors must not enter the scan.
    descriptors = []
    for path in Path('/proc/self/fd').iterdir():
        if not path.name.isdigit():
            continue
        fd = int(path.name)
        try:
            descriptors.append((fd, _identity(fd)))
        except (OSError, ValueError):
            continue
    try:
        for fd, snapshot_identity in descriptors:
            try:
                if _identity(fd) != snapshot_identity:
                    raise RuntimeError("Socket descriptor changed since discovery snapshot")
                sock, identity = _pin(fd, host, port)
            except (OSError, ValueError):
                continue
            if identity != snapshot_identity:
                sock.close()
                raise RuntimeError("Socket descriptor changed during discovery")
            matches.append((fd, sock, identity))
        if len(matches) != 1:
            raise RuntimeError(f"Expected exactly one target TCP descriptor; found {len(matches)}")
        return matches.pop()
    finally:
        for _, sock, _ in matches:
            sock.close()


def _set_option(sock, option, value):
    sock.setsockopt(socket.IPPROTO_TCP, option, value)


class TcpAckExperiment:
    """Finite-owner worker; start explicitly after login, stop before logout.

    Discovery refuses ambiguity, including aliased descriptors. Specify a known
    source fd only when its ownership has been established by the caller.
    Baseline performs the same periodic checks without setting either option.
    """

    def __init__(self, *, host, port, mode, interval_s=.005, source_fd=None):
        self.host = str(ipaddress.IPv4Address(host))
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("TCP port must be 1..65535")
        if mode not in MODES:
            raise ValueError("Explicit baseline/quickack/nodelay/combined mode required")
        if not math.isfinite(interval_s) or not .001 <= interval_s <= .020:
            raise ValueError("Experimental re-arm interval must be 1..20ms")
        if source_fd is not None and (type(source_fd) is not int or source_fd < 0):
            raise ValueError("Source descriptor must be a nonnegative integer")
        if not hasattr(socket, 'TCP_QUICKACK'):
            raise RuntimeError("Linux TCP_QUICKACK support required")
        self.port, self.mode, self.interval_s = port, mode, interval_s
        self.source_fd = source_fd
        self._lock = threading.Lock()
        self._quit = threading.Event()
        self._ready = threading.Event()
        self._thread = None
        self._gaps = deque(maxlen=2048)
        self._report = dict(mode=mode, peer=[self.host, port], interval_s=interval_s,
                            method="pinned_socket_periodic_hint", started=False, alive=False,
                            ticks=0, quickack_success=0, option_failures=0,
                            nodelay_changed=False, nodelay_restored=False,
                            stop_reason=None, error=None, interval_max_s=0.,
                            interval_sum_s=0., interval_count=0,
                            worker_cpu_s=0., fresh_connection_required=False)

    def start(self):
        if self._thread is not None or self._report['started']:
            raise RuntimeError("Experiment is single-use")
        fd, sock, identity = _select(self.host, self.port, self.source_fd)
        try:
            original = sock.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY)
            with self._lock:
                self._report.update(source_fd=fd, socket_inode=identity[1],
                                    local=list(sock.getsockname()), nodelay_original=original,
                                    started=True, alive=True,
                                    started_monotonic_s=time.monotonic())
            self._thread = threading.Thread(target=self._run, args=(fd, sock, identity, original),
                                            name='aubo-tcp-ack', daemon=True)
            self._thread.start()
        except BaseException:
            sock.close()
            with self._lock:
                self._report['alive'] = False
            raise
        if not self._ready.wait(2.):
            self.stop()
            raise RuntimeError("ACK worker did not initialize")
        if not self.status()['alive']:
            report = self.stop()
            raise RuntimeError(f"ACK worker initialization failed: {report}")
        return self

    def _option(self, sock, option, value):
        try:
            _set_option(sock, option, value)
        except OSError:
            with self._lock:
                self._report['option_failures'] += 1
            raise

    def _run(self, fd, sock, identity, original):
        cpu_started = time.thread_time()
        changed = False
        previous = None
        reason = 'requested'
        try:
            if self.mode in ('nodelay', 'combined') and original != 1:
                self._option(sock, socket.TCP_NODELAY, 1)
                changed = True
                with self._lock:
                    self._report['nodelay_changed'] = True
            with self._lock:
                self._report['nodelay_active'] = sock.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY)
            while not self._quit.is_set():
                try:
                    same = _identity(fd) == identity
                except (OSError, ValueError):
                    same = False
                if not same:
                    reason = 'source_descriptor_closed_or_replaced'
                    break
                if sock.getsockopt(socket.IPPROTO_TCP, socket.TCP_INFO, 1)[0] != 1:
                    reason = 'tcp_no_longer_established'
                    break
                now = time.monotonic()
                if self.mode in ('quickack', 'combined'):
                    self._option(sock, socket.TCP_QUICKACK, 1)
                with self._lock:
                    self._report['ticks'] += 1
                    if self.mode in ('quickack', 'combined'):
                        self._report['quickack_success'] += 1
                        self._report['fresh_connection_required'] = True
                    if previous is not None:
                        gap = now - previous
                        self._gaps.append(gap)
                        self._report['interval_max_s'] = max(self._report['interval_max_s'], gap)
                        self._report['interval_sum_s'] += gap
                        self._report['interval_count'] += 1
                previous = now
                self._ready.set()
                # Never catch up missed ticks. Timing is measured, not promised.
                self._quit.wait(self.interval_s)
        except Exception as exc:
            reason = 'worker_error'
            with self._lock:
                self._report['error'] = dict(type=type(exc).__name__, message=str(exc),
                                             errno=getattr(exc, 'errno', None))
        finally:
            try:
                if changed:
                    self._option(sock, socket.TCP_NODELAY, original)
                    with self._lock:
                        self._report['nodelay_restored'] = True
            except OSError as exc:
                reason = 'restore_error'
                with self._lock:
                    self._report['error'] = dict(type=type(exc).__name__, message=str(exc), errno=exc.errno)
            finally:
                sock.close()
            with self._lock:
                self._report.update(alive=False, stop_reason=reason,
                                    worker_cpu_s=time.thread_time() - cpu_started,
                                    ended_monotonic_s=time.monotonic())
            self._ready.set()

    def status(self):
        with self._lock:
            report = dict(self._report)
            gaps = sorted(self._gaps)
        count = report['interval_count']
        report['interval_mean_s'] = report['interval_sum_s'] / count if count else None
        report['recent_interval_p95_s'] = gaps[math.ceil(.95*len(gaps))-1] if gaps else None
        report['recent_interval_count'] = len(gaps)
        return report

    def stop(self):
        self._quit.set()
        if self._thread is not None:
            self._thread.join(timeout=2.)
            if self._thread.is_alive():
                raise RuntimeError("ACK worker did not stop; do not logout concurrently")
        return self.status()

    def __enter__(self):
        return self.start()

    def __exit__(self, *unused):
        self.stop()
