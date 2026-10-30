"""One owner for the independent read-only SDK connection; no reconnect."""
import math
import threading
import time

from .probe import record_ok


def validate_state_result(value, now, *, budget_s=.5, max_age_s=.2):
    detail = f"rc={value.get('return_code')!r}, state={value.get('data')!r}"
    if not record_ok(value):
        raise RuntimeError("state_query_failed: "+detail)
    # Observation validity is distinct from permission to move. The SDK also
    # defines Paused=2 and Resumed=3; stop confirmation must keep observing them.
    if type(value.get("data")) is not int or value["data"] not in (0, 1, 2, 3):
        raise RuntimeError("state_query_unknown_state: "+detail)
    started = float(value["host_started_monotonic_s"])
    received = float(value["host_received_monotonic_s"])
    duration = float(value["duration_s"])
    if (not all(math.isfinite(v) for v in (started, received, duration)) or
            not 0 <= duration <= budget_s or not 0 <= received-started <= budget_s or
            not math.isfinite(now) or now < received or
            (max_age_s is not None and now-received > max_age_s)):
        raise RuntimeError("state_query_failed_or_expired")
    return value


class StateQueryWorker:
    """Cache is in the native observer. This thread owns query scheduling only."""
    def __init__(self, client, *, on_failure, is_active, diagnose, clock=time.monotonic,
                 active_period_s=.1, idle_period_s=.05, budget_s=.5):
        self.client, self.clock = client, clock
        self.on_failure, self.is_active, self.diagnose = on_failure, is_active, diagnose
        self.active_period_s, self.idle_period_s, self.budget_s = active_period_s, idle_period_s, budget_s
        self._lock = threading.Lock()
        self._quit, self._wake, self._first = threading.Event(), threading.Event(), threading.Event()
        self._thread = threading.Thread(target=self._run, name="aubo-state-sdk", daemon=True)
        self._inflight = self._last_result = self._last_attempt = self._error = None

    def status(self):
        with self._lock:
            return dict(sdk_inflight=dict(self._inflight) if self._inflight else None,
                sdk_inflight_age_s=self.clock()-self._inflight["started_s"] if self._inflight else None,
                last_result=dict(self._last_result) if self._last_result else None,
                last_attempt=dict(self._last_attempt) if self._last_attempt else None,
                error=self._error, thread_alive=self._thread.is_alive())

    def fail(self, reason):
        with self._lock:
            if self._error is not None:
                return
            self._error = str(reason)
        self.client.invalidate()  # Local generation change; no SDK I/O.
        self.on_failure("state_observer: "+str(reason))
        self._wake.set()

    def query_once(self):
        with self._lock:
            if self._error is not None or self._quit.is_set():
                return
            self._inflight = dict(operation="robot_state", started_s=self.clock(), budget_s=self.budget_s)
        try:
            raw = self.client.query_state()
            with self._lock:
                self._last_attempt = dict(raw)
            # Preserve failures too, before validation can invalidate the cache.
            self.diagnose("state_query_returned", result=raw, connection="B",
                          phase="active" if self.is_active() else "idle_or_stopping")
            # Successful observation and permission to move are distinct.
            # Delayed host consumption must not permanently kill this channel;
            # the motion owner separately enforces reception freshness.
            validate_state_result(raw, self.clock(), budget_s=self.budget_s, max_age_s=None)
            with self._lock:
                if self._quit.is_set() or self._error is not None:
                    self.client.invalidate()
                    return
                self._last_result = dict(raw)
        except Exception as exc:
            self.fail(type(exc).__name__+": "+str(exc))
        finally:
            with self._lock:
                self._inflight = None
            self._first.set()

    def request_refresh(self):
        self._wake.set()

    def start(self):
        self._thread.start()
        if not self._first.wait(1.):
            self.fail("initial_query_timeout")
            raise RuntimeError("Independent state query did not become ready")
        if self.status()["error"] is not None:
            raise RuntimeError("Independent state query failed: "+self.status()["error"])

    def _run(self):
        while not self._quit.is_set() and self.status()["error"] is None:
            self._wake.clear()
            self.query_once()
            state = self.status()
            if state["error"] is not None or self._quit.is_set():
                break
            period = self.active_period_s if self.is_active() else self.idle_period_s
            # Period is start-to-start, not an extra sleep after the reply.
            # A slow query consumes its own period; keep one call in flight
            # and start the next immediately, without replaying missed ticks.
            started = state["last_result"]["host_started_monotonic_s"]
            self._wake.wait(max(0., started+period-self.clock()))

    def close(self):
        self._quit.set()
        self._wake.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=1.)
        self.client.invalidate()
        if self._thread.is_alive():
            raise RuntimeError("State SDK still in flight; no concurrent logout or destruction")
