"""Offline, causal state/action export with the installed LeRobot writer API."""
import argparse
import bisect
import json
import math
from pathlib import Path

from .recording import SCHEMA, LEGACY_SCHEMA, JOINTS, CAMERAS, FPS, TASK_STAGES, atomic_json


def finite_joints(value):
    return (isinstance(value, (list, tuple)) and len(value) == 6 and
            all(isinstance(v, (int, float)) and math.isfinite(v) for v in value))


def load_recording(path):
    path = Path(path)
    manifest = json.loads((path/'manifest.json').read_text())
    status = json.loads((path/'session_status.json').read_text())
    if manifest['schema'] not in (SCHEMA, LEGACY_SCHEMA) or not status.get('closed') or not status.get('complete') or status.get('dropped_records'):
        raise ValueError('Only closed, complete, loss-free raw sessions can be exported')
    summaries = {v['episode_id']:v for p in sorted((path/'episodes').glob('episode_*.json'))
                 for v in [json.loads(p.read_text())]}
    episodes = {key:[] for key,v in summaries.items()
                if v.get('complete') and v['disposition'] == 'saved' and
                (manifest['schema'] == SCHEMA or not v.get('abnormal_stop'))}
    previous = 0
    with (path/'streams.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            if row['sequence'] != previous+1:
                raise ValueError('Raw sequence gap; refusing silent export')
            previous = row['sequence']
            if row['episode_id'] in episodes:
                episodes[row['episode_id']].append(row)
    if previous != status['written_records']:
        raise ValueError('Raw record count does not match session status')
    for episode, rows in episodes.items():
        if (len(rows) != summaries[episode]['records'] or not rows or
                rows[0]['kind'] != 'episode_start' or rows[-1]['kind'] != 'episode_end'):
            raise ValueError('Episode boundary/count mismatch')
    return manifest, summaries, episodes


def segments(rows, fps, *, feedback_max_age_s=.2, command_max_age_s=.65):
    """Legacy v1 audit helper only; never used by the multimodal exporter."""
    samples = sorted((r for r in rows if r['kind'] == 'sample'), key=lambda r:r['host_monotonic_s'])
    if not samples:
        return []
    times = [r['host_monotonic_s'] for r in samples]
    events = sorted((r for r in rows if r['kind'] in ('sdk_command','hardware_event','pause','resume')),
                    key=lambda r:(r['host_monotonic_s'], r['sequence']))
    cursor = 0
    command = None
    result, current = [], []
    start, end = times[0], times[-1]
    for index in range(math.floor((end-start)*fps+1e-8)+1):
        stamp = start+index/fps
        while cursor < len(events) and events[cursor]['host_monotonic_s'] <= stamp:
            event = events[cursor]
            cursor += 1
            if event['kind'] in ('hardware_event', 'pause', 'resume'):
                command = None
            elif event['phase'] == 'returned':
                attempt = event['attempt']
                sdk = attempt.get('sdk_result', {})
                if (attempt['outcome'] == 'sdk_accepted' and sdk.get('sdk_invoked') is not False
                        and attempt.get('control_source', 'teleop') == 'teleop'
                        and sdk.get('return_code') == 0 and finite_joints(sdk.get('target_rad'))
                        and attempt.get('recording_episode_id', event['episode_id']) == event['episode_id']):
                    command = dict(target=sdk['target_rad'],
                        started=sdk['host_started_monotonic_s'], returned=sdk['host_received_monotonic_s'],
                        known=event['host_monotonic_s'])
                else:
                    command = None
            elif event['phase'] == 'exception':
                command = None
        sample = samples[bisect.bisect_right(times, stamp)-1]
        feedback_stamp = sample.get('feedback_received_monotonic_s')
        good = (sample.get('feedback_valid') and finite_joints(sample.get('measured_q_rad'))
                and sample.get('control_source', 'teleop') == 'teleop'
                and sample['hardware_state'] == 'active' and not sample['operator_paused']
                and not sample.get('fatal') and not sample.get('stop_result_unknown')
                and feedback_stamp is not None and 0 <= stamp-feedback_stamp <= feedback_max_age_s
                and 0 <= stamp-sample['host_monotonic_s'] <= 2./fps
                and command is not None and 0 <= stamp-command['returned'] <= command_max_age_s
                and command['started'] <= command['returned'] <= command['known'] <= stamp)
        if good:
            current.append(dict(state=sample['measured_q_rad'], action=command['target'],
                host_monotonic_s=stamp, feedback_received_monotonic_s=feedback_stamp,
                command_started_monotonic_s=command['started'], command_returned_monotonic_s=command['returned'],
                raw_sample_sequence=sample['sequence'], state_age_s=stamp-feedback_stamp,
                action_age_s=stamp-command['returned']))
        elif current:
            result.append(current)
            current = []
    if current:
        result.append(current)
    return result


SOURCES = ('teleop', 'placement', 'home_return')
PAUSES = ('none', 'clutch_reanchor', 'task_wait', 'automatic_motion', 'operator_pause', 'input_or_system')
DEFAULT_IMAGE_MAX_AGE_S = .15
DEFAULT_TASK = '采摘下画面里面所有的成熟番茄串并放入收集筐'
STAGE_PROTOCOL = 'harvest_retry_v2'


def infer_task_stages(rows):
    """Retrospective task annotations, independent of causal state/action alignment.

    One raw episode is assumed to concern one bunch. Earlier sent close/open
    pairs are retry candidates, the terminal pair a release candidate. Neither
    actuator arrival nor the last pair proves a successful harvest. Every
    fallback and boundary retains its evidence; raw rows remain untouched.
    """
    if not rows:
        return [], dict(protocol=STAGE_PROTOCOL, attempts=[], boundaries=[], warnings=['Empty episode'])
    ordered = sorted(rows, key=lambda r: (r['host_monotonic_s'], r['sequence']))
    start, end = ordered[0]['host_monotonic_s'], ordered[-1]['host_monotonic_s']
    marks = [r for r in ordered if r['kind'] == 'mark' or
             (r['kind'] == 'operator_event' and r.get('command') == 'mark')]
    samples = [r for r in ordered if r['kind'] == 'sample']
    precise = []
    for row in ordered:
        if row['kind'] != 'xr':
            continue
        buttons = row.get('frame', {}).get('controllers', {}).get('right', {}).get('buttons', [])
        if len(buttons) > 4 and buttons[4].get('p') is True:
            precise.append(row)
    attempts, duplicates, unmatched_opens = [], [], []
    previous_closed = None
    for row in ordered:
        if (row['kind'] != 'gripper_event' or row.get('event') != 'command_sent'
                or type(row.get('closed')) is not bool):
            continue
        closed = row['closed']
        if closed == previous_closed:
            duplicates.append(row['sequence'])
            continue
        previous_closed = closed
        if closed:
            attempts.append(dict(attempt_id=len(attempts)+1, close=row, open=None))
        elif attempts and attempts[-1]['open'] is None:
            attempts[-1]['open'] = row
        else:
            unmatched_opens.append(row['sequence'])
    audit = dict(protocol=STAGE_PROTOCOL, mode='retrospective_retry_protocol', mark_count=len(marks),
        marks=[dict(sequence=r['sequence'], host_monotonic_s=r['host_monotonic_s']) for r in marks],
        precision_sample_count=len(precise), attempts=[], boundaries=[], warnings=[],
        duplicate_command_sequences=duplicates, unmatched_open_sequences=unmatched_opens,
        parameters=dict(align_fallback_window_s=1., transport_fallback_hold_s=.5, motion_threshold_rad=.02),
        retry_count=max(0, len(attempts)-1),
        completed_command_cycles=sum(a['open'] is not None for a in attempts),
        final_release_candidate=bool(attempts and attempts[-1]['open']),
        physical_success_verified=False,
        scope='one bunch per raw episode; earlier cycles are retry candidates; no visual outcome classifier')
    if not attempts or attempts[-1]['open'] is None:
        audit['warnings'].append('No terminal close/open pair; task closure is incomplete, not a successful release.')
    if unmatched_opens:
        audit['warnings'].append('Open command without a recorded preceding close; initial held state is not inferred as a successful attempt.')
    if len(attempts) > 1:
        audit['warnings'].append('Retry labels use future command order; in-place failure and single-bunch identity require operator/visual verification.')
    plan = []
    def add(stamp, stage, attempt_id, evidence, source=None, assumed=False):
        plan.append(dict(host_monotonic_s=stamp, stage=stage, attempt_id=attempt_id,
            inference_source=evidence, source_sequence=source['sequence'] if source else None,
            assumed=assumed))
    add(start, 'approach', 1 if attempts else 0, 'episode_start_assumption', ordered[0], True)
    first_close = attempts[0]['close']['host_monotonic_s'] if attempts else end
    cues = [(r, 'precision_button') for r in precise if start <= r['host_monotonic_s'] < first_close]
    cues += [(r, 'mark_before_close') for r in marks if start <= r['host_monotonic_s'] < first_close]
    if cues:
        row, evidence = min(cues, key=lambda item:(item[0]['host_monotonic_s'], item[0]['sequence']))
        add(row['host_monotonic_s'], 'align', 1 if attempts else 0, evidence, row)
    elif attempts and first_close > start:
        at = first_close-min(1., (first_close-start)/2)
        add(at, 'align', 1, 'preclose_time_window_assumption', assumed=True)
        audit['warnings'].append('No preclose A/mark cue; align uses an assumed preclose window.')
    for index, attempt in enumerate(attempts):
        aid, close, opened = attempt['attempt_id'], attempt['close'], attempt['open']
        ct = close['host_monotonic_s']
        ot = opened['host_monotonic_s'] if opened else end
        earlier = index < len(attempts)-1
        info = dict(attempt_id=aid, close_monotonic_s=ct,
            open_monotonic_s=opened['host_monotonic_s'] if opened else None,
            close_sequence=close['sequence'], open_sequence=opened['sequence'] if opened else None,
            outcome='retry_candidate' if earlier else 'final_candidate' if opened else 'incomplete',
            physical_success_verified=False, transport_monotonic_s=None, transport_evidence=None)
        audit['attempts'].append(info)
        add(ct, 'retry' if earlier else 'clamp_cut', aid, 'earlier_close_command' if earlier else 'final_close_command', close)
        if earlier:
            if opened:
                add(ot, 'align', aid+1, 'retry_open_command', opened)
            continue
        reached = next((r for r in ordered if r['kind'] == 'gripper_event'
            and r.get('event') == 'target_reached' and r.get('closed') is True
            and (ct, close['sequence']) < (r['host_monotonic_s'], r['sequence'])
            and r['host_monotonic_s'] < ot), None)
        rt = reached['host_monotonic_s'] if reached else ct
        transport_cues = [(r['host_monotonic_s'], 'closed_mark', r, False) for r in marks
                          if ct < r['host_monotonic_s'] < ot and r['host_monotonic_s'] >= rt]
        eligible = [r for r in samples if rt <= r['host_monotonic_s'] < ot
                    and r.get('feedback_valid') and finite_joints(r.get('measured_q_rad'))
                    and (r.get('planner_target') or {}).get('control_source', 'teleop') != 'home_return'
                    and r.get('pause_type') != 'automatic_motion']
        if reached and eligible:
            anchor = eligible[0]['measured_q_rad']
            moved = next((r for r in eligible[1:] if max(abs(a-b) for a,b in
                          zip(anchor, r['measured_q_rad'])) >= .02), None)
            if moved:
                transport_cues.append((moved['host_monotonic_s'], 'motion_after_closed_feedback', moved, False))
        if transport_cues:
            tt, evidence, row, assumed = min(transport_cues, key=lambda item:item[0])
        elif opened and ot > ct:
            tt = rt if ct < rt < ot else ct+min(.5, (ot-ct)/2)
            evidence, row, assumed = 'transport_after_close_assumption', reached, True
            audit['warnings'].append('No closed mark/motion cue; transport boundary is assumed, not measured fruit retention.')
        else:
            tt = None
        if tt is not None:
            info.update(transport_monotonic_s=tt, transport_evidence=evidence)
            add(tt, 'transport', aid, evidence, row, assumed)
        if opened:
            add(ot, 'release', aid, 'terminal_open_command', opened)
    # Overlay actual home-return intervals; do not label automatic motion as a
    # bunch attempt. Business export can independently filter this metadata.
    home = []
    for row in ordered:
        if row['kind'] == 'pause' and row.get('reason') == 'home_return':
            home.append((row['host_monotonic_s'], True, row))
        elif row['kind'] == 'home_return':
            home.append((row['host_monotonic_s'], row.get('state') in
                         ('waiting_for_stop', 'ready', 'moving', 'stopping'), row))
    changes = [(v['host_monotonic_s'], 0, v) for v in plan]
    changes += [(t, 1, (active, row)) for t, active, row in home]
    changes.sort(key=lambda item:(item[0], item[1]))
    sequence = max(r['sequence'] for r in rows)
    inferred, current, base, home_active, home_source = [], None, None, False, None
    for at, kind, value in changes:
        if kind == 0:
            base = value
        else:
            home_active, home_source = value
        annotation = dict(base)
        if home_active:
            annotation.update(stage='reset', attempt_id=0, inference_source='automatic_home_return',
                source_sequence=home_source['sequence'], assumed=False)
        annotation['host_monotonic_s'] = at
        identity = (annotation['stage'], annotation['attempt_id'], home_active)
        if identity == current:
            continue
        current = identity
        sequence += 1
        event = dict(kind='task_annotation', sequence=sequence, **annotation)
        inferred.append(event)
        audit['boundaries'].append(dict(event))
    return list(rows)+inferred, audit


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def active_camera_keys(manifest):
    cameras = manifest.get('cameras', {})
    keys = manifest.get('active_cameras', list(cameras))
    if (not isinstance(keys, list) or not keys or not all(isinstance(key, str) for key in keys)
            or len(keys) != len(set(keys))
            or set(keys)-set(CAMERAS) or set(keys) != set(cameras)):
        raise ValueError('Manifest active_cameras must be a nonempty list matching registered camera specs')
    return [key for key in CAMERAS if key in keys]


class CameraReader:
    """Audit raw RGB/index correspondence and select causal frames, bounded RAM."""
    def __init__(self, root, key, spec, session_id):
        import cv2
        self.cv2, self.key, self.spec = cv2, key, spec
        if spec.get('fps') != 30 or spec.get('color') != 'RGB':
            raise ValueError(f'{key}: expected RGB at 30 fps')
        root = Path(root).resolve()
        def member(name):
            path = (root/spec[name]).resolve()
            if root not in path.parents:
                raise ValueError('Camera member escapes recording directory')
            return path
        self.rows = [json.loads(line) for line in member('index').read_text().splitlines()]
        status = json.loads(member('status').read_text())
        if (status.get('session_id') != session_id or not status.get('complete')
                or status.get('error') or status.get('written_frames') != len(self.rows) or not self.rows):
            raise ValueError(f'{key}: incomplete/mismatched camera capture')
        for index, row in enumerate(self.rows):
            if (row.get('session_id') != session_id or row.get('serial') != spec['serial']
                    or row.get('camera') != key or row.get('video_frame_index') != index
                    or type(row.get('frame_number')) is not int
                    or not _finite(row.get('host_received_monotonic_s'))):
                raise ValueError(f'{key}: camera identity/index mismatch')
            if index and (row['host_received_monotonic_s'] <= self.rows[index-1]['host_received_monotonic_s']
                          or row['frame_number'] <= self.rows[index-1]['frame_number']):
                raise ValueError(f'{key}: nonmonotonic camera time/frame counter')
        self.times = [row['host_received_monotonic_s'] for row in self.rows]
        gaps = sum(b['frame_number']-a['frame_number']-1 for a, b in zip(self.rows, self.rows[1:]))
        self.audit = dict(serial=spec['serial'], received_frames=len(self.rows),
            missing_frame_numbers=gaps, frame_number_drop_rate=gaps/(len(self.rows)+gaps),
            queue_dropped_frames=status.get('dropped_queue', 0),
            drop_rate_scope='between first and last received frame counters; no exposure/hardware sync claim')
        self.video = member('video')
        self.cap = cv2.VideoCapture(str(self.video))
        count = 0
        while True:
            ok, frame = self.cap.read()
            if not ok:
                break
            if frame.shape != (spec['height'], spec['width'], 3):
                self.cap.release()
                raise ValueError(f'{key}: raw video dimensions changed')
            count += 1
        self.cap.release()
        if count != len(self.rows):
            raise ValueError(f'{key}: raw video/index frame counts differ')
        self.audit['decoded_raw_frames'] = count
        self.cap = cv2.VideoCapture(str(self.video))
        self.cursor, self.rgb = -1, None

    def select(self, stamp, max_age):
        index = bisect.bisect_right(self.times, stamp)-1
        if index < 0 or stamp-self.times[index] > max_age:
            return None
        return self.rows[index]

    def image(self, index):
        if index < self.cursor:
            raise ValueError('Video selection moved backwards')
        while self.cursor < index:
            ok, frame = self.cap.read()
            if not ok:
                raise ValueError(f'{self.key}: failed decoding selected raw frame')
            self.cursor += 1
            self.rgb = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
        return self.rgb.copy()

    def close(self):
        self.cap.release()


def multimodal_ticks(rows, fps, cameras, *, feedback_max_age_s=.2, image_max_age_s=DEFAULT_IMAGE_MAX_AGE_S,
                     tool_max_age_s=.25, exclude_pause_types=(), exclude_stages=()):
    """Yield (tick or None, reason), never inferring action from SDK endpoints."""
    samples = sorted((r for r in rows if r['kind'] == 'sample'), key=lambda r:r['host_monotonic_s'])
    if not samples:
        return
    times = [s['host_monotonic_s'] for s in samples]
    events = sorted((row for row in rows if row['kind'] in
                    ('pause', 'resume', 'task_stage', 'task_annotation', 'planner_target', 'planner_invalid')),
                    key=lambda row:(row['host_monotonic_s'], row['sequence']))
    cursor = 0
    pause_override = stage_override = None
    annotation = None
    planner_override = None
    planner_seen = False
    # Endpoints use real sample times. Missing observations split, no fabricated catchup.
    for index in range(math.floor((times[-1]-times[0])*fps+1e-7)+1):
        stamp = times[0]+index/fps
        sample = samples[bisect.bisect_right(times, stamp)-1]
        planner, tool = sample.get('planner_target'), sample.get('tool')
        pause = sample.get('pause_type', 'none')
        stage = sample.get('task_stage', 'unknown')
        while cursor < len(events) and events[cursor]['host_monotonic_s'] <= stamp:
            event = events[cursor]
            cursor += 1
            if event['kind'] == 'pause':
                pause_override = event['pause_type']
            elif event['kind'] == 'resume':
                pause_override = 'none'
            elif event['kind'] == 'task_stage':
                stage_override = event['stage']
            elif event['kind'] == 'task_annotation':
                annotation = event
            else:
                planner_seen = True
                planner_override = event if event['kind'] == 'planner_target' else None
        if pause_override is not None:
            pause = pause_override
        if stage_override is not None:
            stage = stage_override
        stage_evidence = 'online_label' if stage != 'unknown' else 'unlabeled'
        attempt_id = 0
        if annotation is not None:
            attempt_id = annotation['attempt_id']
            if stage == 'unknown':
                stage = annotation['stage']
                stage_evidence = annotation['inference_source']
        if planner_seen:
            planner = planner_override
        reason = None
        if pause in exclude_pause_types or stage in exclude_stages:
            reason = 'filtered_metadata'
        elif (sample.get('fatal') or sample.get('stop_result_unknown')
                or sample.get('hardware_state') in ('fault', 'stop_unknown', 'stopping', 'stop_requested')):
            reason = 'unsafe_or_unknown_control'
        elif not sample.get('feedback_valid') or not finite_joints(sample.get('measured_q_rad')):
            reason = 'invalid_joint_feedback'
        elif stamp-sample['host_monotonic_s'] > 2./fps+1e-9:
            reason = 'missing_raw_sample'
        elif (not _finite(sample.get('feedback_received_monotonic_s')) or
                not 0 <= stamp-sample['feedback_received_monotonic_s'] <= feedback_max_age_s):
            reason = 'stale_joint_feedback'
        elif (not planner or not finite_joints(planner.get('q_rad'))
                or planner.get('control_source') not in SOURCES
                or not _finite(planner.get('admitted_monotonic_s'))
                or planner['admitted_monotonic_s'] > stamp):
            reason = 'missing_admitted_planner_target'
        elif (not tool or not all(_finite(tool.get(k)) for k in ('position', 'target', 'received_monotonic_s'))
                or not 0 <= tool['position'] <= 1 or tool['target'] not in (0., 1.)
                or not 0 <= stamp-tool['received_monotonic_s'] <= tool_max_age_s):
            reason = 'invalid_or_stale_tool'
        selected = {key:camera.select(stamp, image_max_age_s) for key, camera in cameras.items()}
        if reason is None and any(row is None for row in selected.values()):
            reason = 'missing_or_stale_image'
        if reason:
            yield None, dict(host_monotonic_s=stamp, reason=reason, pause_type=pause,
                task_stage=stage, attempt_id=attempt_id, stage_evidence=stage_evidence,
                raw_sample_sequence=sample['sequence'])
            continue
        yield dict(state=sample['measured_q_rad']+[tool['position']],
            action=planner['q_rad']+[tool['target']], host_monotonic_s=stamp,
            feedback_received_monotonic_s=sample['feedback_received_monotonic_s'],
            tool_received_monotonic_s=tool['received_monotonic_s'],
            planner_admitted_monotonic_s=planner['admitted_monotonic_s'],
            raw_sample_sequence=sample['sequence'], planner_revision=planner['revision'],
            control_source=planner['control_source'], planner_mode=planner['mode'],
            pause_type=pause, task_stage=stage, attempt_id=attempt_id,
            stage_evidence=stage_evidence, cameras=selected), None


def export_recording(source, output, *, repo_id, fps=None, dataset_class=None,
                     feedback_max_age_s=.2, image_max_age_s=DEFAULT_IMAGE_MAX_AGE_S, tool_max_age_s=.25,
                     exclude_pause_types=(), exclude_stages=(), stitch_filtered=False,
                     infer_stages=False, task=DEFAULT_TASK):
    """v2 full-task seven-dimensional export. Legacy SDK-only data is never upgraded."""
    from contextlib import ExitStack
    from collections import Counter
    import numpy as np
    manifest, summaries, episodes = load_recording(source)
    if manifest['schema'] != SCHEMA:
        raise ValueError('Legacy SDK-only recordings cannot supply admitted planner/tool/image data')
    camera_keys = active_camera_keys(manifest)
    fps = manifest['sample_fps'] if fps is None else fps
    if fps not in FPS or fps > manifest['sample_fps']:
        raise ValueError('Export FPS must be 20, 25, 30 or 50 and cannot exceed raw FPS')
    if any(not _finite(v) or v <= 0 for v in (feedback_max_age_s, image_max_age_s, tool_max_age_s)):
        raise ValueError('Positive finite freshness limits required')
    if set(exclude_pause_types)-set(PAUSES) or set(exclude_stages)-set(TASK_STAGES):
        raise ValueError('Unknown pause/stage filter')
    if not isinstance(task, str) or not task.strip():
        raise ValueError('Nonempty dataset task prompt required')
    output = Path(output).expanduser().resolve()
    from .recording import _allowed_roots
    if not any(root in output.parents for root in _allowed_roots()) or output.exists():
        raise ValueError('Export requires a new directory under /data or AUBO_ALLOWED_ROOTS')
    if dataset_class is None:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        dataset_class = LeRobotDataset
    features = {key:dict(dtype='float32', shape=(7,), names=JOINTS+['gripper'])
                for key in ('observation.state', 'action')}
    for key in ('host_monotonic_s', 'feedback_received_monotonic_s', 'tool_received_monotonic_s',
                'planner_admitted_monotonic_s'):
        features['source.'+key] = dict(dtype='float64', shape=(1,), names=[key])
    for key in ('episode_id', 'sample_sequence', 'planner_revision', 'pause_type', 'task_stage', 'control_source', 'attempt_id'):
        features['source.'+key] = dict(dtype='int64', shape=(1,), names=[key])
    for key in camera_keys:
        spec = manifest['cameras'][key]
        features['observation.images.'+key] = dict(dtype='video',
            shape=(spec['height'], spec['width'], 3), names=['height', 'width', 'channels'])
        for field, dtype in (('host_monotonic_s', 'float64'), ('frame_number', 'int64'),
                             ('video_frame_index', 'int64')):
            features[f'source.{key}.{field}'] = dict(dtype=dtype, shape=(1,), names=[field])
    report = dict(schema=SCHEMA, source=str(Path(source).resolve()), repo_id=repo_id, fps=fps,
        complete=False, active_cameras=camera_keys, cameras={}, action_semantics=manifest['action_semantics'],
        tool_semantics=manifest['tool_semantics'], session_id=manifest['session_id'],
        task=task, raw_tasks={key:value['task'] for key,value in summaries.items()},
        filters=dict(exclude_pause_types=list(exclude_pause_types), exclude_stages=list(exclude_stages),
                     stitch_filtered=stitch_filtered),
        freshness_s=dict(state=feedback_max_age_s, image=image_max_age_s, tool=tool_max_age_s),
        enums=dict(pause_type=PAUSES, task_stage=TASK_STAGES, control_source=SOURCES),
        skipped_raw_episodes=[key for key in summaries if key not in episodes], segments=[], rejected_ticks={})
    report['stage_inference'] = dict(enabled=infer_stages, protocol=STAGE_PROTOCOL,
        attempt_id_semantics='episode-local, one-based; retry opening starts next attempt; reset/unassigned=0',
        retrospective=True, physical_success_verified=False, episodes={})
    report['tool_events'] = dict(Counter(
        row.get('event', 'unknown') for rows in episodes.values() for row in rows
        if row['kind'] == 'gripper_event'))
    with ExitStack() as stack:
        cameras = {}
        for key in camera_keys:
            camera = CameraReader(source, key, manifest['cameras'][key], manifest['session_id'])
            cameras[key] = camera
            stack.callback(camera.close)
            report['cameras'][key] = camera.audit
        # Explicit single-thread CPU encoding; no training or network upload.
        from lerobot.configs.video import RGBEncoderConfig
        dataset = dataset_class.create(repo_id=repo_id, fps=fps, root=output, robot_type='aubo_i5_clip_cut',
            features=features, use_videos=True, video_backend='pyav',
            rgb_encoder=RGBEncoderConfig(vcodec='h264', pix_fmt='yuv420p'), encoder_threads=1)
        atomic_json(output/'export_report.json', report)
        rejected = Counter()
        segment = None
        def flush():
            nonlocal segment
            if segment is not None:
                dataset.save_episode(parallel_encoding=False)
                report['segments'].append(segment)
                segment = None
        try:
            with (output/'alignment.jsonl').open('x') as alignment, (output/'rejected_ticks.jsonl').open('x') as rejects:
                for episode, rows in episodes.items():
                    if infer_stages:
                        rows, stage_audit = infer_task_stages(rows)
                        report['stage_inference']['episodes'][episode] = stage_audit
                    for tick, failure in multimodal_ticks(rows, fps, cameras,
                            feedback_max_age_s=feedback_max_age_s, image_max_age_s=image_max_age_s,
                            tool_max_age_s=tool_max_age_s, exclude_pause_types=exclude_pause_types,
                            exclude_stages=exclude_stages):
                        if failure:
                            rejected[failure['reason']] += 1
                            rejects.write(json.dumps(dict(raw_episode=episode, **failure))+'\n')
                            if not (stitch_filtered and failure['reason'] == 'filtered_metadata'):
                                flush()
                            continue
                        if segment is None:
                            segment = dict(lerobot_episode=len(report['segments']), raw_episode=episode, frames=0,
                                started_monotonic_s=tick['host_monotonic_s'], discontinuities=[])
                        frame = {'observation.state':np.asarray(tick['state'], dtype=np.float32),
                                 'action':np.asarray(tick['action'], dtype=np.float32),
                                 'task':task}
                        values = {key:tick[key] for key in ('host_monotonic_s', 'feedback_received_monotonic_s',
                                  'tool_received_monotonic_s', 'planner_admitted_monotonic_s', 'planner_revision')}
                        values.update(episode_id=episode, sample_sequence=tick['raw_sample_sequence'],
                            pause_type=PAUSES.index(tick['pause_type']), task_stage=TASK_STAGES.index(tick['task_stage']),
                            control_source=SOURCES.index(tick['control_source']), attempt_id=tick['attempt_id'])
                        for key, value in values.items():
                            dtype = features['source.'+key]['dtype']
                            frame['source.'+key] = np.asarray([value], dtype=dtype)
                        for key, selected in tick['cameras'].items():
                            frame['observation.images.'+key] = cameras[key].image(selected['video_frame_index'])
                            for field in ('host_monotonic_s', 'frame_number', 'video_frame_index'):
                                value = selected['host_received_monotonic_s'] if field == 'host_monotonic_s' else selected[field]
                                frame[f'source.{key}.{field}'] = np.asarray([value], dtype=features[f'source.{key}.{field}']['dtype'])
                        if segment['frames'] and tick['host_monotonic_s']-segment['ended_monotonic_s'] > 1.5/fps:
                            segment['discontinuities'].append(dict(after_frame=segment['frames']-1,
                                raw_gap_s=tick['host_monotonic_s']-segment['ended_monotonic_s']))
                        dataset.add_frame(frame)
                        alignment.write(json.dumps(dict(raw_episode=episode, exported_episode=segment['lerobot_episode'],
                            exported_frame=segment['frames'], **tick), allow_nan=False)+'\n')
                        segment['frames'] += 1
                        segment['ended_monotonic_s'] = tick['host_monotonic_s']
                    flush()
        finally:
            dataset.finalize()
        report['rejected_ticks'] = dict(rejected)
        report['complete'] = bool(report['segments'])
        atomic_json(output/'export_report.json', report)
        if not report['complete']:
            raise ValueError('No complete multimodal ticks; inspect rejected_ticks.jsonl; raw retained')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--repo-id', required=True)
    parser.add_argument('--fps', type=int, choices=FPS)
    parser.add_argument('--task', default=DEFAULT_TASK, help='unified task prompt; original prompts retained in export report')
    parser.add_argument('--image-max-age-s', type=float, default=DEFAULT_IMAGE_MAX_AGE_S,
                        help='maximum causal image age in seconds (default: 0.15)')
    parser.add_argument('--state-max-age-s', type=float, default=.2)
    parser.add_argument('--tool-max-age-s', type=float, default=.25)
    parser.add_argument('--exclude-pause-type', action='append', choices=PAUSES, default=[])
    parser.add_argument('--exclude-stage', action='append', choices=TASK_STAGES, default=[])
    parser.add_argument('--stitch-filtered', action='store_true',
                        help='explicitly compress only metadata-filtered gaps; record all joins')
    parser.add_argument('--infer-task-stages', action='store_true',
                        help='infer unknown stages and attempts from tool cycles, A/mark cues and closed motion; retain audit')
    args = parser.parse_args(argv)
    print(json.dumps(export_recording(args.input, args.output, repo_id=args.repo_id, fps=args.fps,
        image_max_age_s=args.image_max_age_s, feedback_max_age_s=args.state_max_age_s,
        tool_max_age_s=args.tool_max_age_s, exclude_pause_types=args.exclude_pause_type,
        exclude_stages=args.exclude_stage, stitch_filtered=args.stitch_filtered,
        infer_stages=args.infer_task_stages, task=args.task), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
