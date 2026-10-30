"""Bounded input timing evidence; no poses, SDK calls or disk I/O."""
from collections import deque
import math
import threading


def _number(value):
    try:
        return value if type(value) in (int, float) and math.isfinite(value) else None
    except OverflowError:
        return None


class InputTiming:
    def __init__(self, capacity=128):
        self._lock = threading.Lock()
        self._received = deque(maxlen=capacity)
        self._processed = deque(maxlen=capacity)

    def received(self, frame, stamp):
        frame = frame if isinstance(frame, dict) else {}
        controllers = frame.get("controllers")
        right = controllers.get("right") if isinstance(controllers, dict) else None
        buttons = right.get("buttons") if isinstance(right, dict) else None
        button = buttons[1] if isinstance(buttons, list) and len(buttons) > 1 else None
        epoch = frame.get("epoch")
        event = dict(received_monotonic_s=_number(stamp),
            epoch=epoch[:160] if isinstance(epoch, str) else None,
            seq=frame.get("seq") if type(frame.get("seq")) is int else None,
            t_client_ms=_number(frame.get("t_client")),
            right_present=isinstance(right, dict),
            grip=button.get("p") if isinstance(button, dict) and type(button.get("p")) is bool else None,
            receive_gap_s=None, source_gap_s=None, seq_gap=None,
            relative_delay_change_s=None)
        with self._lock:
            prior = self._received[-1] if self._received else None
            if prior and event["received_monotonic_s"] is not None and prior["received_monotonic_s"] is not None:
                event["receive_gap_s"] = event["received_monotonic_s"]-prior["received_monotonic_s"]
            if prior and event["epoch"] is not None and event["epoch"] == prior["epoch"]:
                if event["t_client_ms"] is not None and prior["t_client_ms"] is not None:
                    event["source_gap_s"] = (event["t_client_ms"]-prior["t_client_ms"])/1000.
                    if event["receive_gap_s"] is not None:
                        # A change in transit/host delay, NOT an absolute
                        # one-way latency: source and host clocks differ.
                        event["relative_delay_change_s"] = event["receive_gap_s"]-event["source_gap_s"]
                if event["seq"] is not None and prior["seq"] is not None:
                    event["seq_gap"] = event["seq"]-prior["seq"]
            self._received.append(event)

    def processed(self, event):
        with self._lock:
            self._processed.append(dict(event))

    def snapshot(self, *, recent=False):
        with self._lock:
            result = dict(last_received=dict(self._received[-1]) if self._received else None,
                last_processed=dict(self._processed[-1]) if self._processed else None)
            if recent:
                result.update(recent_received=[dict(e) for e in self._received],
                              recent_processed=[dict(e) for e in self._processed])
            return result
