"""Synthetic three-camera harvesting cycle; no device imports or SDK calls."""
import json
from pathlib import Path
import threading

from teleop.aubo.cameras import CameraCapture
from teleop.aubo.feedback import Feedback
from teleop.aubo.recording import CAMERAS, RawRecorder


def make_recording(root, *, ticks=90, camera_gap=True, camera_keys=CAMERAS, retry=False):
    import numpy as np
    root = Path(root)
    specs = {key:dict(serial='synthetic-'+key, width=64, height=48, fps=30, color='RGB',
        video=f'cameras/{key}.avi', index=f'cameras/{key}.jsonl', status=f'cameras/{key}.status.json')
        for key in camera_keys}
    now = [1000.]
    recorder = RawRecorder(root, task='SYNTHETIC ONLY: cut tomato bunch, transport and release into basket',
        fps=30, cameras=specs, metadata=dict(synthetic=True, hardware_connected=False), clock=lambda:now[0])
    for cam_index, (key, spec) in enumerate(specs.items()):
        capture = CameraCapture(root, key, spec, session_id=recorder.session_id,
            on_error=lambda reason:None, capacity=ticks+1)
        for n in range(ticks):
            rgb = np.zeros((48, 64, 3), dtype=np.uint8)
            rgb[:, :, cam_index] = 60+n % 150
            rgb[8:16, 8:16, :] = 180
            capture.submit(rgb, frame_number=n+1+(int(camera_gap and key=='scene_right' and n>=15)),
                device_timestamp_ms=n*1000/30, timestamp_domain='synthetic',
                stamp=1000.+n/30-.005-cam_index*.001)
        capture.capture_done.set()
        capture._write()
        assert capture.error is None, capture.error
    recorder.begin()
    for n in range(ticks):
        now[0] = 1000.+n/30+1e-9
        stage = 'approach' if n<15 else 'clamp_cut' if n<30 else 'transport' if n<60 else 'release' if n<75 else 'verify'
        if not retry:
            recorder.set_task_stage(stage)
        if n == 20:
            recorder.pause_episode('grip_released')
        if n == 25:
            recorder.resume_episode()
        if n == 45:
            recorder.pause_episode('tool_wait')
        if n == 48:
            recorder.resume_episode()
        goal = [.1+n*.001]*6
        recorder.planner_target(goal, control_source='placement' if 30<=n<60 else 'teleop',
            mode='holding' if 20<=n<25 else 'active', control_session=1)
        closed = (15<=n<25 or 40<=n<60) if retry else 15<=n<60
        if n in ((15, 25, 40, 60) if retry else (15, 60)):
            recorder.record('gripper_event', event='command_sent', closed=closed)
        if retry and n in (17, 42):
            recorder.record('gripper_event', event='target_reached', closed=True)
        gripper = dict(settings=dict(open_position=834, close_position=3500),
            position=3500 if closed else 834, feedback_received_s=now[0]-.01,
            commanded_closed=closed, hardware_error=0, fault=None)
        recorder.sample(feedback=Feedback((.05+n*.0001,)*6, now[0]-.003, 'synthetic', True, n),
            hardware=dict(state='holding' if 20<=n<25 else 'active'), gripper=gripper,
            control_source='placement' if 30<=n<60 else 'teleop')
    recorder.end()
    recorder.close()
    return recorder
