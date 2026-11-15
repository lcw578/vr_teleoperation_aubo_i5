"""Offline quality audit by decoding every tick with the real LeRobot v3 reader."""
import argparse
import bisect
import json
import math
from pathlib import Path

from .recording import CAMERAS, atomic_json


def distribution(values):
    import numpy as np
    if not values:
        return dict(count=0, p50_ms=None, p95_ms=None, max_ms=None)
    array = np.asarray(values, dtype=float)*1000
    return dict(count=len(values), p50_ms=float(np.percentile(array, 50)),
                p95_ms=float(np.percentile(array, 95)), max_ms=float(array.max()))


def validate_retry_audit(audit):
    """Check command chronology, not an assumed one-close/one-open success rule."""
    attempts = audit['attempts']
    previous_open = None
    for index, attempt in enumerate(attempts):
        aid, closed, opened = attempt['attempt_id'], attempt['close_monotonic_s'], attempt['open_monotonic_s']
        if aid != index+1 or not isinstance(closed, (int, float)) or not math.isfinite(closed):
            raise ValueError('Invalid retry attempt chronology')
        if previous_open is not None and closed < previous_open:
            raise ValueError('Overlapping retry attempts')
        if opened is not None and (not math.isfinite(opened) or opened < closed):
            raise ValueError('Open command precedes attempt close')
        if index < len(attempts)-1 and opened is None:
            raise ValueError('Retry attempt lacks reopening')
        previous_open = opened
        transport = attempt.get('transport_monotonic_s')
        if transport is not None and (transport < closed or (opened is not None and transport >= opened)):
            raise ValueError('Transport boundary outside closed interval')
    if audit['retry_count'] != max(0,len(attempts)-1):
        raise ValueError('Retry count mismatch')
    if audit['completed_command_cycles'] != sum(a['open_monotonic_s'] is not None for a in attempts):
        raise ValueError('Completed command cycle count mismatch')
    if audit['final_release_candidate'] != bool(attempts and attempts[-1]['open_monotonic_s'] is not None):
        raise ValueError('Final release candidate mismatch')
    previous_time = -math.inf
    for boundary in audit['boundaries']:
        at, aid = boundary['host_monotonic_s'], boundary['attempt_id']
        if not math.isfinite(at) or at < previous_time or not 0 <= aid <= len(attempts):
            raise ValueError('Invalid retry annotation boundary')
        previous_time = at


def check_dataset(root, *, report_path=None, dataset_class=None):
    import numpy as np
    if dataset_class is None:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        dataset_class = LeRobotDataset
    root = Path(root).resolve()
    export = json.loads((root/'export_report.json').read_text())
    if not export.get('complete'):
        raise ValueError('Export not confirmed complete')
    dataset = dataset_class(export['repo_id'], root=root, download_videos=False, video_backend='pyav')
    # HF's default tensor transform casts float64 arrays to float32. Audit
    # original parquet scalars, otherwise long-running monotonic clocks lose ms.
    precise_rows = dataset.hf_dataset.with_format(None)
    info = dataset.meta.info
    version = info.get('codebase_version') if isinstance(info, dict) else info.codebase_version
    if version != 'v3.0' or dataset.fps != export['fps']:
        raise ValueError('Expected matching LeRobot v3 fps')
    image_features = {key for key in dataset.features if key.startswith('observation.images.')}
    camera_keys = [key for key in CAMERAS if 'observation.images.'+key in image_features]
    if (not camera_keys or image_features != {'observation.images.'+key for key in camera_keys}
            or set(export.get('active_cameras', [])) != set(camera_keys)
            or set(dataset.meta.camera_keys) != image_features):
        raise ValueError('Dataset image features must match the exported active camera set')
    for key in ('action', 'observation.state'):
        if tuple(dataset.features[key]['shape']) != (7,):
            raise ValueError('Seven-dimensional state and action required')
    state_ages, tool_ages, spreads = [], [], []
    image_ages = {key:[] for key in camera_keys}
    used = {key:set() for key in camera_keys}
    stage_counts, pause_counts, control_counts, episode_reports, episode_ages = {}, {}, {}, {}, {}
    opens = closes = 0
    previous_tool = None
    previous_raw_episode = None
    current_episode = None
    previous_t = previous_raw_t = None
    count = 0
    inference = export.get('stage_inference', {})
    annotation_audits = inference.get('episodes', {}) if inference.get('enabled') else {}
    for audit in annotation_audits.values():
        if audit.get('protocol') == 'harvest_retry_v2':
            validate_retry_audit(audit)
    attempt_feature = 'source.attempt_id' in dataset.features
    if inference.get('protocol') == 'harvest_retry_v2' and inference.get('enabled') and not attempt_feature:
        raise ValueError('Retry protocol requires source.attempt_id')
    attempt_coverage = {}
    def scalar(row, key):
        value = row[key]
        if isinstance(value, (list, tuple)):
            value = value[0]
        elif hasattr(value, 'item'):
            value = value.item()
        if not math.isfinite(value):
            raise ValueError(f'Nonfinite metadata: {key}')
        return value
    with (root/'tick_quality.jsonl').open('w') as ticks, (root/'alignment.jsonl').open() as aligned:
        for index in range(dataset.num_frames):
            row = dataset[index]  # Force real parquet + MP4 decoding for all cameras.
            metadata = precise_rows[index]
            for key in ('action', 'observation.state'):
                value = np.asarray(row[key])
                if value.shape != (7,) or not np.isfinite(value).all():
                    raise ValueError(f'Invalid {key} at frame {index}')
            audit_line = aligned.readline()
            if not audit_line:
                raise ValueError('Alignment provenance shorter than dataset')
            provenance = json.loads(audit_line)
            host = scalar(metadata, 'source.host_monotonic_s')
            episode = int(scalar(row, 'episode_index'))
            raw_episode = int(scalar(metadata, 'source.episode_id'))
            timestamp = scalar(row, 'timestamp')
            if provenance['raw_episode'] != raw_episode or provenance['exported_episode'] != episode:
                raise ValueError('Episode provenance mismatch')
            if abs(host-provenance['host_monotonic_s']) > 1e-8:
                raise ValueError('Provenance time mismatch')
            if not np.allclose(np.asarray(row['action']), provenance['action'], atol=1e-6):
                raise ValueError('Admitted action differs from exported provenance')
            if not np.allclose(np.asarray(row['observation.state']), provenance['state'], atol=1e-6):
                raise ValueError('Measured state differs from exported provenance')
            if export.get('task') is not None and row['task'] != export['task']:
                raise ValueError('Unified task prompt differs from export configuration')
            if episode == current_episode:
                if abs(timestamp-previous_t-1/dataset.fps) > 1e-4 or host <= previous_raw_t:
                    raise ValueError('Dataset tick spacing/order mismatch')
            elif abs(timestamp) > 1e-4:
                raise ValueError('Episode does not start at time zero')
            current_episode, previous_t, previous_raw_t = episode, timestamp, host
            state_age = host-scalar(metadata, 'source.feedback_received_monotonic_s')
            tool_age = host-scalar(metadata, 'source.tool_received_monotonic_s')
            planner_time = scalar(metadata, 'source.planner_admitted_monotonic_s')
            if not 0 <= state_age <= export['freshness_s']['state']+1e-9:
                raise ValueError('State freshness violation')
            if not 0 <= tool_age <= export['freshness_s']['tool']+1e-9 or planner_time > host:
                raise ValueError('Tool/planner causality violation')
            state_ages.append(state_age)
            tool_ages.append(tool_age)
            camera_times, ages = [], {}
            for key in camera_keys:
                img = np.asarray(row['observation.images.'+key])
                if img.ndim != 3 or img.shape[0] != 3 or not np.isfinite(img).all():
                    raise ValueError(f'Image decode failure: {key} frame {index}')
                stamp = scalar(metadata, f'source.{key}.host_monotonic_s')
                age = host-stamp
                if not 0 <= age <= export['freshness_s']['image']+1e-9:
                    raise ValueError('Image freshness/causality violation')
                camera_times.append(stamp)
                ages[key] = age
                image_ages[key].append(age)
                used[key].add(int(scalar(metadata, f'source.{key}.video_frame_index')))
            spread = max(camera_times)-min(camera_times) if len(camera_times) > 1 else None
            if spread is not None:
                spreads.append(spread)
            pause = export['enums']['pause_type'][int(scalar(row, 'source.pause_type'))]
            stage = export['enums']['task_stage'][int(scalar(row, 'source.task_stage'))]
            attempt_id = int(scalar(metadata, 'source.attempt_id')) if attempt_feature else None
            if attempt_feature:
                if attempt_id < 0 or scalar(metadata, 'source.attempt_id') != attempt_id:
                    raise ValueError('Invalid attempt ID')
                if attempt_id != provenance.get('attempt_id'):
                    raise ValueError('Attempt provenance mismatch')
            stage_audit = annotation_audits.get(str(raw_episode), annotation_audits.get(raw_episode))
            if stage_audit and stage_audit.get('protocol') == 'harvest_retry_v2':
                boundaries = stage_audit['boundaries']
                at = bisect.bisect_right([b['host_monotonic_s'] for b in boundaries], host)-1
                if at < 0 or attempt_id != boundaries[at]['attempt_id']:
                    raise ValueError('Attempt ID differs from retrospective boundary audit')
                if provenance.get('stage_evidence') != 'online_label' and stage != boundaries[at]['stage']:
                    raise ValueError('Inferred task stage differs from boundary audit')
            if provenance.get('task_stage', stage) != stage:
                raise ValueError('Task stage provenance mismatch')
            pause_counts[pause] = pause_counts.get(pause, 0)+1
            stage_counts[stage] = stage_counts.get(stage, 0)+1
            source = export['enums']['control_source'][int(scalar(metadata, 'source.control_source'))]
            control_counts[source] = control_counts.get(source, 0)+1
            summary = episode_reports.setdefault(episode, dict(raw_episode=raw_episode, frames=0,
                open_transitions=0, close_transitions=0, stage_ticks={}, pause_ticks={}, control_source_ticks={},
                attempt_ticks={}))
            if attempt_id is not None:
                key = str(attempt_id)
                summary['attempt_ticks'][key] = summary['attempt_ticks'].get(key, 0)+1
                coverage = attempt_coverage.setdefault(str(raw_episode), {})
                detail = coverage.setdefault(key, dict(frames=0, stage_ticks={}))
                detail['frames'] += 1
                detail['stage_ticks'][stage] = detail['stage_ticks'].get(stage, 0)+1
            age_samples = episode_ages.setdefault(episode, dict(state=[], tool=[], spread=[],
                cameras={key:[] for key in camera_keys}))
            age_samples['state'].append(state_age)
            age_samples['tool'].append(tool_age)
            if spread is not None:
                age_samples['spread'].append(spread)
            for key, age in ages.items():
                age_samples['cameras'][key].append(age)
            summary['frames'] += 1
            for field, name in (('stage_ticks', stage), ('pause_ticks', pause), ('control_source_ticks', source)):
                summary[field][name] = summary[field].get(name, 0)+1
            target = float(row['action'][6])
            if target not in (0., 1.) or not 0 <= float(row['observation.state'][6]) <= 1:
                raise ValueError('Tool normalized state/absolute goal invalid')
            if raw_episode != previous_raw_episode or provenance['exported_frame'] == 0:
                previous_tool = None
            if previous_tool is not None and target != previous_tool:
                closes += target == 1.
                opens += target == 0.
                summary['close_transitions'] += int(target == 1.)
                summary['open_transitions'] += int(target == 0.)
            previous_tool, previous_raw_episode = target, raw_episode
            ticks.write(json.dumps(dict(index=index, episode=episode, raw_episode=raw_episode,
                host_monotonic_s=host, state_age_s=state_age, tool_age_s=tool_age,
                image_age_s=ages, camera_spread_s=spread, pause_type=pause, task_stage=stage,
                attempt_id=attempt_id, stage_evidence=provenance.get('stage_evidence'),
                tool_goal=target), allow_nan=False)+'\n')
            count += 1
        if aligned.readline():
            raise ValueError('Alignment provenance longer than dataset')
    expected = sum(segment['frames'] for segment in export['segments'])
    if count != expected or dataset.num_episodes != len(export['segments']):
        raise ValueError('Dataset/export episode/frame count mismatch')
    for episode, summary in episode_reports.items():
        samples = episode_ages[episode]
        summary.update(state_age=distribution(samples['state']), tool_age=distribution(samples['tool']),
            camera_selection_spread=distribution(samples['spread']),
            image_age={key:distribution(values) for key, values in samples['cameras'].items()})
    retry_report = {}
    for raw_episode, audit in annotation_audits.items():
        if audit.get('protocol') != 'harvest_retry_v2':
            continue
        raw_key = str(raw_episode)
        coverage = attempt_coverage.get(raw_key, {})
        details = []
        for attempt in audit['attempts']:
            aid = str(attempt['attempt_id'])
            details.append(dict(attempt, exported_coverage=coverage.get(aid, dict(frames=0, stage_ticks={}))))
        retry_report[raw_key] = dict(attempt_count=len(details), retry_count=audit['retry_count'],
            completed_command_cycles=audit['completed_command_cycles'],
            final_release_candidate=audit['final_release_candidate'], physical_success_verified=False,
            attempts=details, warnings=audit['warnings'],
            note='Multiple close/open cycles are supported retries, not a dataset integrity error; labels do not verify task success')
    report = dict(complete=True, codebase_version=version, fps=dataset.fps,
        verified_frames=count, verified_episodes=dataset.num_episodes, active_cameras=camera_keys,
        state_age=distribution(state_ages), tool_age=distribution(tool_ages),
        camera_selection_spread=distribution(spreads),
        cameras={key:dict(export['cameras'][key], image_age=distribution(image_ages[key]),
            selected_unique_frames=len(used[key]), selected_repeat_fraction=1-len(used[key])/count)
            for key in camera_keys},
        tool_events=dict(exported_open_transitions=opens, exported_close_transitions=closes,
                         raw_events=export.get('tool_events', {}),
                         scope='absolute target transitions within exported episodes; initial state is not an event'),
        stage_ticks=stage_counts, pause_ticks=pause_counts, control_source_ticks=control_counts,
        episodes=episode_reports, stage_inference=export.get('stage_inference', {'enabled':False}),
        retry_analysis=retry_report, task=export.get('task'),
        rejected_ticks=export['rejected_ticks'], filters=export['filters'],
        tick_report='tick_quality.jsonl', raw_camera_audit='frame-number gaps verified during export',
        note='Host availability ages; neither exposure alignment nor robot physical execution latency')
    atomic_json(Path(report_path) if report_path else root/'quality_report.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', required=True, type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args(argv)
    print(json.dumps(check_dataset(args.dataset, report_path=args.report), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
