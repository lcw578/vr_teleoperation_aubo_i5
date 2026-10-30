"""Bounded event handoff; only this background owner serializes live diagnostics."""
from collections import deque
import gc
import json
import logging
import threading
import time

from .probe import json_safe


class DiagnosticStream:
    def __init__(self, output, worker, *, snapshot, period_s=.1, followup_s=2.):
        self.output, self.worker, self.snapshot = output, worker, snapshot
        self.period_s, self.followup_s = period_s, followup_s
        self._quit = threading.Event()
        self._thread = threading.Thread(target=self._run, name="aubo-diagnostic-writer", daemon=True)
        self._cursor = 0
        self.written_events = self.missing_events = 0
        self._seen = None
        self._pending = []
        self._gc_started = {}
        self._gc_pauses = deque(maxlen=32)
        self.error = None
        self.worker.enable_diagnostic_stream()

    def start(self):
        gc.callbacks.append(self._gc_callback)
        self._thread.start()

    def _gc_callback(self, phase, info):
        # No I/O or worker locks inside a GC callback. Record slow collections
        # for the writer to correlate with control-loop stalls on the next run.
        generation = info['generation']
        now = time.monotonic()
        if phase == 'start':
            self._gc_started[generation] = now
        else:
            started = self._gc_started.pop(generation, None)
            if started is not None and now-started >= .005:
                self._gc_pauses.append(dict(event='python_gc_pause', generation=generation,
                    host_monotonic_s=now, started_monotonic_s=started, duration_s=now-started,
                    thread_id=threading.get_ident()))

    def _write(self, row):
        self.output.write(json.dumps(json_safe(row), allow_nan=False, separators=(',', ':'))+'\n')

    def poll(self):
        for _ in range(len(self._gc_pauses)):
            self._write(self._gc_pauses.popleft())
        rows = self.worker.drain_diagnostic_stream()
        for row in rows:
            sequence = row['event_sequence']
            if sequence != self._cursor+1:
                self.missing_events += sequence-self._cursor-1
                self._write(dict(event="diagnostic_stream_gap", after_sequence=self._cursor,
                                 next_sequence=sequence, missing_events=sequence-self._cursor-1))
            self._write(row)
            self.written_events += 1
            self._cursor = sequence
            if row['event'] == 'stop_requested' and row['trigger']['reason'] != 'grip_released':
                trigger = row['trigger']
                self._write(dict(event="incident_trigger", trigger=trigger,
                                 evidence=self.snapshot()))
                self._pending.append((trigger, time.monotonic()+self.followup_s))
        now = time.monotonic()
        evidence = self.snapshot()
        stop = evidence.get('input_stop')
        if stop:
            stamp = stop['trigger']['host_monotonic_s']
            if stamp != self._seen:
                self._write(dict(event="incident_input_frozen", evidence=evidence))
                self._seen = stamp
        pending = []
        for trigger, deadline in self._pending:
            if now >= deadline or self._quit.is_set():
                self._write(dict(event="incident_followup", trigger=trigger, evidence=evidence,
                                 host_monotonic_s=now, complete_window=now >= deadline))
            else:
                pending.append((trigger, deadline))
        self._pending = pending
        self._write(dict(event="clock_sync", host_monotonic_s=time.monotonic(),
                         wall_time_ns=time.time_ns()))
        self.output.flush()
        return len(rows)

    def _run(self):
        try:
            while not self._quit.wait(self.period_s):
                self.poll()
            while self.poll() == 256:
                pass
        except Exception as exc:
            self.error = type(exc).__name__+": "+str(exc)
            logging.getLogger(__name__).error("Diagnostic writer failed: %s", self.error)

    def close(self):
        self._quit.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=3.)
        if self._gc_callback in gc.callbacks:
            gc.callbacks.remove(self._gc_callback)
        if self._thread.is_alive():
            raise RuntimeError("Diagnostic writer still active")
        if self.error:
            raise RuntimeError("Diagnostic writer failed: "+self.error)
