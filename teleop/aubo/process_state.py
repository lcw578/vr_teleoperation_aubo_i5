"""Independent read-only SDK with separate state RPC and joint telemetry.

Credentials use a private request pipe. A bounded nonblocking datagram channel
carries only joint samples; query completion never gates their publication.
Both channels preserve the child SDK reception timestamps. No reconnect.
"""
import multiprocessing
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path


_JOINT_PACKET = struct.Struct("!Qd6d")


def _forward_waypoints(client, telemetry, quit_event):
    sequence = 0
    while not quit_event.is_set():
        raw = client.waypoint_feedback_since(sequence)
        for sample in raw["samples"]:
            current = sample["sequence"]
            if current <= sequence:
                continue
            packet = _JOINT_PACKET.pack(current, sample["host_received_monotonic_s"],
                                        *sample["q_rad"])
            # A slow parent may lose samples, but never backpressures the SDK.
            # Dropped packets are not replayed and cannot become fresh later.
            try:
                telemetry.send(packet)
            except BlockingIOError:
                pass
            sequence = current
        quit_event.wait(.005)


def _serve(connection, runtime, telemetry=None):
    from .follow import load_follow_native
    client = None
    connected = False
    quit_event = threading.Event()
    publisher = None
    try:
        native = load_follow_native(runtime)
        client = native.RobotStateObserver()
        while True:
            operation, arguments = connection.recv()
            try:
                if operation == "login":
                    result = client.login(*arguments)
                    connected = result["return_code"] == 0
                    arguments = None
                elif operation == "start_feedback":
                    if telemetry is None or publisher is not None:
                        raise RuntimeError("Joint telemetry unavailable or already started")
                    result = client.start_waypoint_feedback()
                    if result["return_code"] == 0:
                        publisher = threading.Thread(target=_forward_waypoints,
                            args=(client, telemetry, quit_event),
                            name="aubo-joint-publisher", daemon=True)
                        publisher.start()
                elif operation == "query":
                    result = client.query_state()
                elif operation == "logout":
                    quit_event.set()
                    if publisher is not None:
                        publisher.join(timeout=1.)
                        if publisher.is_alive():
                            raise RuntimeError("Joint publisher did not finish; no concurrent SDK logout")
                    result = client.logout() if connected else None
                    connected = False
                else:
                    raise ValueError("Unknown read-only process request")
                connection.send((True, result))
                if operation == "logout":
                    break
            except Exception as exc:
                connection.send((False, type(exc).__name__ + ": " + str(exc)))
    except (EOFError, BrokenPipeError):
        pass
    finally:
        quit_event.set()
        if publisher is not None:
            publisher.join(timeout=1.)
        if connected and client is not None and (publisher is None or not publisher.is_alive()):
            client.logout()
        if telemetry is not None:
            telemetry.close()
        connection.close()


class ProcessStateObserver:
    process_isolated = True

    def __init__(self, native, runtime, *, waypoint_feedback=False, waypoint_source="B"):
        if waypoint_source not in ('B', 'C'):
            raise ValueError('Independent waypoint source must be B or C')
        self._waypoint_source = waypoint_source
        self.cache = native.RobotStateCache()
        self._generation = self.cache.invalidate()
        self.waypoint_cache = ((native.WaypointFeedbackCache() if waypoint_source == 'B'
                               else native.WaypointFeedbackCache(waypoint_source))
                              if waypoint_feedback else None)
        self._waypoint_generation = self.waypoint_cache.invalidate() if waypoint_feedback else None
        self._guard = threading.Lock()
        self._io = threading.Lock()
        self._context = multiprocessing.get_context("spawn")
        self._pipe, self._child_pipe = self._context.Pipe()
        self._telemetry = self._child_telemetry = None
        if waypoint_feedback:
            self._telemetry, self._child_telemetry = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
            self._telemetry.settimeout(.05)
            self._child_telemetry.setblocking(False)
        self._telemetry_quit = threading.Event()
        self._telemetry_ready = threading.Event()
        self._telemetry_thread = None
        self._telemetry_error = None
        self._telemetry_sequence = 0
        self._telemetry_received = None
        runtime = runtime or os.environ.get("AUBO_RUNTIME_DIR", "/data/VR_teleoperation_runtime")
        self._runtime = str(Path(runtime).resolve())
        self._process = None
        self._started = self._broken = False
        self._sequence = 0

    def invalidate(self):
        # State worker invalidation does not manufacture a new joint sample.
        with self._guard:
            self._generation = self.cache.invalidate()

    def _invalidate_waypoints(self, error=None):
        with self._guard:
            if self.waypoint_cache is not None:
                self._waypoint_generation = self.waypoint_cache.invalidate()
            self._telemetry_error = error

    def _receive_waypoints(self):
        generation = self._waypoint_generation
        try:
            while not self._telemetry_quit.is_set():
                try:
                    packet = self._telemetry.recv(_JOINT_PACKET.size+1)
                except socket.timeout:
                    if self._process is not None and self._process.poll() is not None:
                        raise RuntimeError("Read-only joint SDK process exited")
                    continue
                if len(packet) != _JOINT_PACKET.size:
                    raise RuntimeError("Invalid joint telemetry packet size")
                sequence, received, *q = _JOINT_PACKET.unpack(packet)
                # Rejection includes duplicate/regressed/future/nonfinite data.
                with self._guard:
                    if not self.waypoint_cache.publish(q, received, sequence, generation):
                        raise RuntimeError("Invalid or out-of-session joint telemetry")
                    self._telemetry_sequence = sequence
                    self._telemetry_received = time.monotonic()
                self._telemetry_ready.set()
        except Exception as exc:
            if not self._telemetry_quit.is_set():
                self._invalidate_waypoints(type(exc).__name__+": "+str(exc))
                self.invalidate()
                self._telemetry_ready.set()

    def start_waypoint_feedback(self):
        if self.waypoint_cache is None or self._telemetry_thread is not None:
            raise RuntimeError("Joint telemetry unavailable or already started")
        self._telemetry_thread = threading.Thread(target=self._receive_waypoints,
            name="aubo-joint-receiver", daemon=True)
        self._telemetry_thread.start()
        result = self._request("start_feedback")
        if result["return_code"] != 0 or result["duration_s"] > .5:
            raise RuntimeError("Joint telemetry subscription failed or late")
        if not self._telemetry_ready.wait(1.):
            self._invalidate_waypoints("initial_joint_telemetry_timeout")
            raise RuntimeError("Independent joint feedback did not become ready")
        if self.status()["telemetry_error"] is not None:
            raise RuntimeError(self.status()["telemetry_error"])
        return result

    def status(self):
        with self._guard:
            return {
                "process_isolated": True,
                "process_id": self._process.pid if self._process else None,
                "alive": self._process.poll() is None if self._process else False,
                "generation": self._generation,
                "joint_feedback_source": self._waypoint_source if self.waypoint_cache is not None else "A",
                "telemetry_alive": self._telemetry_thread.is_alive() if self._telemetry_thread else False,
                "telemetry_error": self._telemetry_error,
                "telemetry_sequence": self._telemetry_sequence,
                "telemetry_ipc_received_monotonic_s": self._telemetry_received,
            }

    def _request(self, operation, arguments=(), timeout=.75):
        with self._io:
            if self._broken or self._process is None or self._process.poll() is not None:
                self.invalidate()
                raise RuntimeError("Read-only SDK process unavailable")
            try:
                self._pipe.send((operation, arguments))
                if not self._pipe.poll(timeout):
                    raise TimeoutError("Read-only SDK process response timeout")
                ok, value = self._pipe.recv()
                if not ok:
                    raise RuntimeError(value)
                return value
            except Exception:
                self._broken = True
                self.invalidate()
                raise

    def login(self, host, port, user, password):
        if self._started:
            raise RuntimeError("New observer required; no reconnect")
        self._started = True
        child_fd = self._child_pipe.fileno()
        command = [sys.executable, "-m", __name__, "--child-fd", str(child_fd),
                   "--runtime", self._runtime]
        descriptors = [child_fd]
        if self._child_telemetry is not None:
            telemetry_fd = self._child_telemetry.fileno()
            command += ["--telemetry-fd", str(telemetry_fd)]
            descriptors.append(telemetry_fd)
        self._process = subprocess.Popen(command, pass_fds=tuple(descriptors), start_new_session=True)
        self._child_pipe.close()
        if self._child_telemetry is not None:
            self._child_telemetry.close()
        try:
            result = self._request("login", (host, port, user, password), timeout=3.)
            return dict(result, process_id=self._process.pid,
                        parent_process_id=os.getpid(), process_isolated=True)
        except Exception:
            self.abandon_cleanup()
            raise

    def query_state(self):
        with self._guard:
            generation = self._generation
        raw = self._request("query")
        with self._guard:
            self._sequence += 1
            published = self.cache.publish(raw["data"] if raw["data"] is not None else -1,
                raw["return_code"], raw["host_started_monotonic_s"],
                raw["host_received_monotonic_s"], generation)
            return dict(raw, generation=generation, sequence=self._sequence,
                cache_published=published, process_id=self._process.pid,
                ipc_received_monotonic_s=time.monotonic())

    def logout(self):
        self.invalidate()
        if not self._started:
            return None
        try:
            result = self._request("logout", timeout=2.)
            self._process.wait(timeout=1.)
            return result
        finally:
            self.abandon_cleanup()

    def abandon_cleanup(self):
        self.invalidate()
        self._telemetry_quit.set()
        if self._telemetry_thread is not None:
            self._telemetry_thread.join(timeout=1.)
            if self._telemetry_thread.is_alive():
                raise RuntimeError("Joint telemetry receiver did not finish")
        self._invalidate_waypoints()
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=1.)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=1.)
        self._pipe.close()
        self._child_pipe.close()
        if self._telemetry is not None:
            self._telemetry.close()
            self._child_telemetry.close()


class SplitProcessStateObserver:
    """A sends; B only queries state; C only subscribes to joint telemetry.

    Each observer owns a fresh SDK process. C has no state query scheduler;
    stopping/freshness checks keep the same state and joint reception stamps.
    """
    process_isolated = True
    sdk_connections = 3

    def __init__(self, native, runtime):
        self.state = ProcessStateObserver(native, runtime)
        self.joints = ProcessStateObserver(native, runtime, waypoint_feedback=True, waypoint_source='C')
        self.cache = self.state.cache
        self.waypoint_cache = self.joints.waypoint_cache

    def login(self, host, port, user, password):
        result = self.state.login(host, port, user, password)
        if result['return_code'] != 0:
            return result
        try:
            joint_result = self.joints.login(host, port, user, password)
            if joint_result['return_code'] != 0:
                self.state.logout()
                return joint_result
        except Exception:
            self.state.logout()
            raise
        return dict(result, waypoint_login=joint_result, sdk_connections=3)

    def start_waypoint_feedback(self):
        return self.joints.start_waypoint_feedback()

    def query_state(self):
        return self.state.query_state()

    def invalidate(self):
        self.state.invalidate()

    def status(self):
        state = self.state.status()
        joints = self.joints.status()
        return dict(state, joint_feedback_source='C', sdk_connections=3,
                    joint_process_id=joints['process_id'], telemetry_alive=joints['telemetry_alive'],
                    telemetry_error=joints['telemetry_error'],
                    telemetry_sequence=joints['telemetry_sequence'],
                    telemetry_ipc_received_monotonic_s=joints['telemetry_ipc_received_monotonic_s'])

    def logout(self):
        # Motion/stop confirmation has finished before either owner logs out.
        # Always release C too if B logout reports a failure.
        try:
            state = self.state.logout()
        finally:
            joints = self.joints.logout()
        return dict(state, waypoint_logout=joints,
                    return_code=state['return_code'] or joints['return_code'])

    def abandon_cleanup(self):
        try:
            self.state.abandon_cleanup()
        finally:
            self.joints.abandon_cleanup()


if __name__ == "__main__":
    import argparse
    from multiprocessing.connection import Connection
    parser = argparse.ArgumentParser()
    parser.add_argument("--child-fd", type=int, required=True)
    parser.add_argument("--telemetry-fd", type=int)
    parser.add_argument("--runtime", required=True)
    args = parser.parse_args()
    telemetry = socket.socket(fileno=args.telemetry_fd) if args.telemetry_fd is not None else None
    _serve(Connection(args.child_fd), args.runtime, telemetry)
