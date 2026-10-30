"""Local synthetic-only raw -> real LeRobot v3.0 -> reload acceptance check."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from teleop.aubo.recording import RawRecorder
from teleop.aubo.feedback import Feedback
from teleop.aubo.export_lerobot import export_recording


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    now=[1000.]
    recorder=RawRecorder(args.output/'raw',task='ISOLATED SYNTHETIC TEST: joint trajectory; no hardware',
        metadata=dict(synthetic=True, hardware_connected=False),clock=lambda:now[0])
    recorder.begin()
    for n in range(40):
        now[0]=1000.+n*.05
        q=[.001*n]*6
        attempt=dict(outcome='sdk_accepted',recording_episode_id=0,sdk_result=dict(return_code=0,
            sdk_invoked=True,target_rad=q,host_started_monotonic_s=now[0]-.01,
            host_received_monotonic_s=now[0]-.001))
        recorder.record('sdk_command',phase='returned',attempt=attempt)
        recorder.sample(feedback=Feedback(tuple(q),now[0]-.002,'isolated_synthetic',True,0),
            hardware=dict(state='active'),paused=n in (17,18))
    recorder.end()
    # Wait only for this local writer's completed save, never for hardware.
    import time
    deadline=time.monotonic()+2.
    while recorder.status()['pending_saves']:
        if time.monotonic()>deadline:raise AssertionError('Synthetic save timed out')
        time.sleep(.01)
    recorder.begin();recorder.end(disposition='invalid');recorder.close()
    report=export_recording(args.output/'raw',args.output/'lerobot',repo_id='local/aubo_synthetic_acceptance')
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    dataset=LeRobotDataset('local/aubo_synthetic_acceptance',root=args.output/'lerobot',
                          download_videos=False)
    assert dataset.meta.info.codebase_version=='v3.0'
    assert dataset.num_episodes==2,report
    assert dataset.num_frames>10
    assert not dataset.meta.camera_keys
    assert dataset.features['observation.state']['shape']==(6,)
    assert dataset.features['action']['shape']==(6,)
    for index in range(dataset.num_frames):
        row=dataset[index]
        assert row['action'].shape==(6,)
        assert row['observation.state'].shape==(6,)
        assert row['source.command_returned_monotonic_s'].item()<=row['source.host_monotonic_s'].item()
        assert int(row['source.episode_id'].item())==0
    report.update(verified_frames=dataset.num_frames,verified_episodes=dataset.num_episodes,
                  codebase_version=dataset.meta.info.codebase_version,synthetic=True)
    (args.output/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
