"""Synthetic-only three RGB streams + admitted planner/tool -> real LeRobot v3 -> QA."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from multimodal_fixture import make_recording
from teleop.aubo.export_lerobot import DEFAULT_TASK, export_recording
from teleop.aubo.check_dataset import check_dataset


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    recorder=make_recording(args.output/'raw')
    report=export_recording(recorder.path,args.output/'lerobot',repo_id='local/aubo_multimodal_synthetic')
    quality=check_dataset(args.output/'lerobot')
    assert quality['verified_frames']>=85,quality
    assert quality['verified_episodes']==1,quality
    assert quality['fps']==30
    assert quality['pause_ticks']['clutch_reanchor']>=4
    assert quality['stage_ticks']['transport']>=25
    assert quality['stage_ticks']['release']>=12
    assert quality['tool_events']['exported_close_transitions']==1
    assert quality['tool_events']['exported_open_transitions']==1
    assert quality['cameras']['scene_right']['missing_frame_numbers']==1
    assert quality['cameras']['wrist']['missing_frame_numbers']==0
    assert abs(quality['camera_selection_spread']['max_ms']-2.)<1e-4
    # A one-camera raw session exports and validates without synthetic views.
    single = make_recording(args.output/'raw_single', camera_keys=('scene_left',))
    single_report=export_recording(single.path,args.output/'lerobot_single',
        repo_id='local/aubo_multimodal_single_camera')
    single_quality=check_dataset(args.output/'lerobot_single')
    assert single_report['active_cameras']==['scene_left']
    assert single_quality['active_cameras']==['scene_left']
    assert list(single_quality['cameras'])==['scene_left']
    assert single_quality['camera_selection_spread']['count']==0
    assert all(json.loads(line)['camera_spread_s'] is None
               for line in (args.output/'lerobot_single/tick_quality.jsonl').read_text().splitlines())
    # Exercise real writer split vs explicitly stitched metadata filters as well.
    filtered=export_recording(recorder.path,args.output/'filtered',repo_id='local/aubo_multimodal_filtered',
                              exclude_pause_types=['clutch_reanchor'])
    filtered_quality=check_dataset(args.output/'filtered')
    assert filtered_quality['verified_episodes']==2,filtered_quality
    assert filtered_quality['verified_frames']<quality['verified_frames']
    stitched=export_recording(recorder.path,args.output/'stitched',repo_id='local/aubo_multimodal_stitched',
                              exclude_pause_types=['clutch_reanchor'],stitch_filtered=True)
    stitched_quality=check_dataset(args.output/'stitched')
    assert stitched_quality['verified_episodes']==1
    assert stitched['segments'][0]['discontinuities']
    # Real retry export must retain both cycles without requiring four marks.
    retry_raw=make_recording(args.output/'raw_retry', camera_keys=('wrist',), retry=True)
    retry_export=export_recording(retry_raw.path,args.output/'retry',repo_id='local/aubo_retry_synthetic',
                                 infer_stages=True)
    retry_quality=check_dataset(args.output/'retry')
    assert retry_quality['task']==DEFAULT_TASK
    assert retry_quality['verified_episodes']==1
    assert retry_quality['tool_events']['exported_close_transitions']==2
    assert retry_quality['tool_events']['exported_open_transitions']==2
    assert retry_quality['retry_analysis']['0']['attempt_count']==2
    assert retry_quality['retry_analysis']['0']['retry_count']==1
    assert retry_quality['retry_analysis']['0']['physical_success_verified'] is False
    assert set(retry_quality['stage_ticks'])=={'approach','align','retry','clamp_cut','transport','release'}
    assert set(retry_quality['episodes'][0]['attempt_ticks'])=={'1','2'}
    # Provenance tampering must fail even when state/action/video are valid.
    alignment=args.output/'retry/alignment.jsonl'
    original=alignment.read_text()
    lines=original.splitlines()
    first=json.loads(lines[0]);first['attempt_id']=99
    lines[0]=json.dumps(first)
    alignment.write_text('\n'.join(lines)+'\n')
    try:
        check_dataset(args.output/'retry')
    except ValueError as error:
        assert 'Attempt provenance mismatch' in str(error),error
    else:
        raise AssertionError('Corrupted attempt provenance accepted')
    finally:
        alignment.write_text(original)
    check_dataset(args.output/'retry')
    report.update(verified_frames=quality['verified_frames'], verified_episodes=quality['verified_episodes'],
                  synthetic=True, hardware_connected=False, quality=quality,
                  single_camera=single_quality,
                  filtered_episodes=filtered_quality['verified_episodes'],
                  stitched_episodes=stitched_quality['verified_episodes'], retry=retry_quality)
    (args.output/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
