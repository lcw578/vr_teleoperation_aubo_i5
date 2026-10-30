"""Offline, causal state/action export with the installed LeRobot writer API."""
import argparse
import bisect
import json
import math
from pathlib import Path

from .recording import SCHEMA, JOINTS, atomic_json


def finite_joints(value):
    return (isinstance(value, (list, tuple)) and len(value) == 6 and
            all(isinstance(v, (int, float)) and math.isfinite(v) for v in value))


def load_recording(path):
    path = Path(path)
    manifest = json.loads((path/'manifest.json').read_text())
    status = json.loads((path/'session_status.json').read_text())
    if manifest['schema'] != SCHEMA or not status.get('complete') or status.get('dropped_records'):
        raise ValueError('Only closed, complete, loss-free raw sessions can be exported')
    summaries = {v['episode_id']:v for p in sorted((path/'episodes').glob('episode_*.json'))
                 for v in [json.loads(p.read_text())]}
    episodes = {key:[] for key,v in summaries.items()
                if v.get('complete') and v['disposition'] == 'saved' and not v.get('abnormal_stop')}
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
    """No future samples, interpolation or joining across invalid/paused gaps."""
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


def export_recording(source, output, *, repo_id, fps=None, dataset_class=None):
    manifest, summaries, episodes = load_recording(source)
    fps = manifest['sample_fps'] if fps is None else fps
    if fps not in (20,25,50) or fps > manifest['sample_fps']:
        raise ValueError('Export FPS must be 20, 25 or 50 and cannot exceed raw sample FPS')
    output = Path(output).expanduser().resolve()
    if Path('/data') not in output.parents or output.exists():
        raise ValueError('Export requires a new directory under /data')
    prepared = [(episode, part) for episode, rows in episodes.items()
                for part in segments(rows, fps) if len(part) >= 2]
    if not prepared:
        raise ValueError('No continuous valid state/action segments with at least two frames; raw data retained')
    if dataset_class is None:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        dataset_class = LeRobotDataset
    import numpy as np
    features = {key:dict(dtype='float32', shape=(6,), names=JOINTS)
                for key in ('observation.state','action')}
    for key in ('observation.state_age_s','action.age_s'):
        features[key] = dict(dtype='float32', shape=(1,), names=[key])
    for key in ('host_monotonic_s','feedback_received_monotonic_s',
                'command_started_monotonic_s','command_returned_monotonic_s'):
        features['source.'+key] = dict(dtype='float64', shape=(1,), names=[key])
    for key in ('episode_id','sample_sequence'):
        features['source.'+key] = dict(dtype='int64', shape=(1,), names=[key])
    dataset = dataset_class.create(repo_id=repo_id, fps=fps, root=output,
        robot_type='aubo_i5', features=features, use_videos=False)
    report = dict(source=str(Path(source).resolve()), schema=SCHEMA, fps=fps,
        complete=False, cameras=False, joint_unit='rad',
        action_semantics=manifest['action_semantics'],
        resampling='causal previous sample and SDK-accepted command; no interpolation; invalid gaps split',
        skipped_raw_episodes=[key for key in summaries if key not in episodes], segments=[])
    atomic_json(output/'export_report.json', report)
    try:
        for episode, part in prepared:
            for row in part:
                frame = {'observation.state':np.asarray(row['state'],dtype=np.float32),
                         'action':np.asarray(row['action'],dtype=np.float32),
                         'task':summaries[episode]['task'],
                         'observation.state_age_s':np.asarray([row['state_age_s']],dtype=np.float32),
                         'action.age_s':np.asarray([row['action_age_s']],dtype=np.float32),
                         'source.episode_id':np.asarray([episode],dtype=np.int64),
                         'source.sample_sequence':np.asarray([row['raw_sample_sequence']],dtype=np.int64)}
                for key in ('host_monotonic_s','feedback_received_monotonic_s',
                            'command_started_monotonic_s','command_returned_monotonic_s'):
                    frame['source.'+key] = np.asarray([row[key]],dtype=np.float64)
                dataset.add_frame(frame)
            dataset.save_episode()
            report['segments'].append(dict(lerobot_episode=len(report['segments']), raw_episode=episode,
                frames=len(part), started_monotonic_s=part[0]['host_monotonic_s'],
                ended_monotonic_s=part[-1]['host_monotonic_s']))
    finally:
        dataset.finalize()
    report['complete'] = True
    atomic_json(output/'export_report.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--repo-id', required=True, help='local LeRobot dataset identity; nothing is uploaded')
    parser.add_argument('--fps', type=int, choices=(20,25,50))
    args = parser.parse_args(argv)
    print(json.dumps(export_recording(args.input,args.output,repo_id=args.repo_id,fps=args.fps),
                     ensure_ascii=False,indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
