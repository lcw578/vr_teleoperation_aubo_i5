"""Independent RealSense RGB owners. No robot SDK or control-loop disk I/O."""
import json
import math
from pathlib import Path
import queue
import threading
import time

from .recording import CAMERAS, atomic_json

CAMERA_QUEUE_CAPACITY = 60


def load_camera_config(path):
    config = json.loads(Path(path).read_text())
    specs = config.get('cameras') if isinstance(config, dict) and 'cameras' in config else config
    if not isinstance(specs, dict) or not specs or set(specs)-set(CAMERAS):
        raise ValueError('Camera config requires a nonempty subset of scene_left, scene_right and wrist')
    serials = []
    active = {}
    for key, spec in specs.items():
        if not isinstance(spec, dict):
            raise ValueError(f'{key}: camera configuration must be an object')
        enabled = spec.get('enabled', True)
        if type(enabled) is not bool:
            raise ValueError(f'{key}: enabled must be a boolean')
        if not enabled:
            continue
        serial = spec.get('serial', spec.get('serial_number'))
        if (not isinstance(serial, str) or not serial.strip()
                or spec.get('fps', 30) != 30
                or any(type(spec.get(k)) is not int or spec[k] <= 0 for k in ('width', 'height'))):
            raise ValueError('Each enabled camera needs serial/serial_number, positive width/height and fps=30')
        spec['serial'] = serial
        spec.update(fps=30, video=f'cameras/{key}.avi', index=f'cameras/{key}.jsonl',
                    status=f'cameras/{key}.status.json', color='RGB',
                    timestamp='host_received_monotonic_s; device timestamp retained separately')
        serials.append(spec['serial'])
        active[key] = spec
    if not active:
        raise ValueError('At least one camera must be enabled')
    if len(set(serials)) != len(active):
        raise ValueError('Enabled cameras must have unique serials')
    return active


class CameraCapture:
    def __init__(self, root, key, spec, *, session_id, on_error, capacity=CAMERA_QUEUE_CAPACITY):
        self.root, self.key, self.spec = Path(root), key, spec
        self.session_id, self.on_error = session_id, on_error
        self.queue = queue.Queue(maxsize=capacity)
        self.stop_event = threading.Event()
        self.capture_done = threading.Event()
        self.ready = threading.Event()
        self.rows = self.dropped = 0
        self.error = None
        self._threads = []

    def fail(self, error):
        if self.error is None:
            self.error = str(error)
            self.on_error(f'camera {self.key}: {error}')
        self.stop_event.set()

    def submit(self, rgb, *, frame_number, device_timestamp_ms, timestamp_domain, stamp):
        """Capture handoff, also usable by isolated synthetic sources."""
        if not math.isfinite(stamp):
            raise ValueError('Finite capture reception timestamp required')
        item = (rgb, dict(session_id=self.session_id, camera=self.key, serial=self.spec['serial'],
                frame_number=int(frame_number), device_timestamp_ms=float(device_timestamp_ms),
                timestamp_domain=timestamp_domain, host_received_monotonic_s=stamp))
        try:
            self.queue.put_nowait(item)
            return True
        except queue.Full:
            self.dropped += 1
            self.fail('capture_queue_overflow')
            return False

    def _capture(self):
        pipe = None
        try:
            import numpy as np
            import pyrealsense2 as rs
            pipe = rs.pipeline()
            config = rs.config()
            config.enable_device(self.spec['serial'])
            config.enable_stream(rs.stream.color, self.spec['width'], self.spec['height'], rs.format.rgb8, 30)
            pipe.start(config)
            warmup_until = time.monotonic()+2.
            while not self.stop_event.is_set():
                frames = pipe.wait_for_frames(1000)
                stamp = time.monotonic()
                color = frames.get_color_frame()
                if stamp < warmup_until or not color:
                    continue
                self.submit(np.asanyarray(color.get_data()).copy(),
                    frame_number=color.get_frame_number(), device_timestamp_ms=color.get_timestamp(),
                    timestamp_domain=str(color.get_frame_timestamp_domain()), stamp=stamp)
        except Exception as exc:
            self.fail(f'{type(exc).__name__}: {exc}')
        finally:
            if pipe is not None:
                try:
                    pipe.stop()
                except Exception as exc:
                    self.fail(str(exc))
            self.capture_done.set()

    def _write(self):
        writer = None
        try:
            import cv2
            target = self.root/self.spec['video']
            target.parent.mkdir(parents=True, exist_ok=True)
            writer = cv2.VideoWriter(str(target), cv2.VideoWriter_fourcc(*'MJPG'), 30,
                                     (self.spec['width'], self.spec['height']))
            if not writer.isOpened():
                raise RuntimeError('Cannot open camera video writer')
            with (self.root/self.spec['index']).open('x') as index:
                while not self.capture_done.is_set() or not self.queue.empty():
                    try:
                        rgb, row = self.queue.get(timeout=.05)
                    except queue.Empty:
                        continue
                    if rgb.shape != (self.spec['height'], self.spec['width'], 3):
                        raise ValueError('Camera image shape changed')
                    writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                    row['video_frame_index'] = self.rows
                    index.write(json.dumps(row, allow_nan=False)+'\n')
                    self.rows += 1
                    self.ready.set()
                    self.queue.task_done()
                index.flush()
                import os
                os.fsync(index.fileno())
        except Exception as exc:
            self.fail(f'{type(exc).__name__}: {exc}')
        finally:
            if writer is not None:
                writer.release()
            atomic_json(self.root/self.spec['status'], dict(session_id=self.session_id,
                complete=self.error is None, error=self.error, written_frames=self.rows,
                dropped_queue=self.dropped))

    def start(self):
        self._threads = [threading.Thread(target=fn, daemon=True, name=f'camera-{self.key}-{name}')
                         for fn, name in ((self._write, 'writer'), (self._capture, 'capture'))]
        for thread in self._threads:
            thread.start()

    def close(self):
        self.stop_event.set()
        for thread in reversed(self._threads):
            thread.join(timeout=5.)
        if any(thread.is_alive() for thread in self._threads):
            self.fail('camera_shutdown_timeout')
            raise RuntimeError(self.error)


class CameraGroup:
    def __init__(self, recorder):
        self.recorder = recorder
        def failed(reason):
            recorder.record('camera_error', reason=reason)
            recorder.error = reason
        self.cameras = [CameraCapture(recorder.path, key, spec, session_id=recorder.session_id,
                                     on_error=failed) for key, spec in recorder.cameras.items()]

    def start(self):
        for camera in self.cameras:
            camera.start()
        deadline = time.monotonic()+6.
        while not all(camera.ready.is_set() for camera in self.cameras):
            if self.recorder.error:
                raise RuntimeError(self.recorder.error)
            if time.monotonic() >= deadline:
                raise RuntimeError('Three-camera initialization timed out')
            time.sleep(.01)

    def close(self):
        errors = []
        for camera in self.cameras:
            try:
                camera.close()
            except RuntimeError as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError('; '.join(errors))
