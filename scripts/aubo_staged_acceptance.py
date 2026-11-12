"""Finite production-worker acceptance. No free VR; no automatic repetition.

Run from the project via python -m scripts.aubo_staged_acceptance. Stage B
requires a passed stage A report from exactly the same source/native build.
"""
import argparse
from datetime import datetime,timezone,timedelta
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from teleop.aubo.follow import J6FollowWorker,load_follow_native
from teleop.aubo.offline_session import Candidate
from teleop.aubo.process_state import SplitProcessStateObserver
from teleop.aubo.probe import json_safe,record_ok


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--fixed-tick-batch',action='store_true')
    parser.add_argument('--wall-clock-batch',action='store_true')
    parser.add_argument('--refined-stream',action='store_true')
    parser.add_argument('--high-flow',action='store_true')
    parser.add_argument('--batch-points',type=int,choices=range(1,9),default=4)
    parser.add_argument('--motion-profile',choices=('conservative','balanced','practical','advanced'),default=None)
    parser.add_argument('--axis',type=int,choices=range(1,7),default=6)
    parser.add_argument('--tcp-ack-mode',choices=('baseline','quickack','nodelay','combined'))
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--stage',choices=('A','B','CLUTCH'),required=True)
    parser.add_argument('--runtime',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--stage-a-report')
    args=parser.parse_args()
    if args.motion_profile is not None and not args.refined_stream:parser.error('Motion profiles require refined stream')
    if args.refined_stream and not args.wall_clock_batch:parser.error('Refined stream requires wall clock batches')
    if args.wall_clock_batch and (not args.fixed_tick_batch or args.batch_points<4 or args.tcp_ack_mode not in ('nodelay','combined')):
        parser.error('Wall clock test requires fixed tick, 4..8 maximum and explicit NODELAY mode')
    if not args.execute:
        print('Plan only: A=5s stationary; B=J6 +/-2deg; CLUTCH=5 hot cycles, moving J6 +/-0.8deg sinusoidal target, 1.5s motion/0.75s release, bounded return.');return
    project=Path.cwd()
    runtime=Path(args.runtime).resolve();output=Path(args.output).resolve()
    output.mkdir(parents=True,exist_ok=True)
    source_paths=['teleop/aubo/follow.py','teleop/aubo/native/follow.cpp',
        'teleop/aubo/trajectory.py','teleop/aubo/motion_profile.py','teleop/aubo/quest_follow.py','teleop/aubo/shadow.py',
        'scripts/aubo_staged_acceptance.py','teleop/aubo/native/passthrough.hpp','scripts/aubo_program_sine.py','teleop/aubo/tcp_ack.py','teleop/aubo/state_query.py']
    sources={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in source_paths}
    from teleop.aubo.motion_profile import motion_profile
    profile=motion_profile(args.motion_profile or 'conservative')
    accelerated=args.motion_profile in ('balanced','practical','advanced')
    sine_amplitude=12. if profile.velocity_deg_s>=45 else 8. if accelerated else 6.
    sine_peak=.96*profile.velocity_deg_s if accelerated else 6.*2*math.pi/4.2
    sine_period=2*math.pi*sine_amplitude/sine_peak
    measured_envelope=20. if profile.velocity_deg_s>=45 else 13. if accelerated else 9.
    manifest=json.loads((runtime/'follow-build.json').read_text())
    if args.stage in ('B','CLUTCH'):
        if not args.stage_a_report:raise RuntimeError('Stage A success required')
        a=json.loads(Path(args.stage_a_report).read_text())
        if not a.get('passed') or a['stage']!='A' or a['sources']!=sources or a['native_manifest']!=manifest:
            raise RuntimeError('Stage A missing/failed or build changed; no Stage B motion')
        if (a.get('motion_profile')!=args.motion_profile or bool(a.get('refined_stream'))!=args.refined_stream or bool(a.get('wall_clock_batch'))!=args.wall_clock_batch or
                a.get('batch_point_limit',4)!=args.batch_points or
                bool(a.get('fixed_tick_batch'))!=args.fixed_tick_batch or
                a.get('tcp_ack_start',{}).get('mode')!=args.tcp_ack_mode):
            raise RuntimeError('Stage A pacing/batch/socket configuration differs; no motion')
    lock=(project/'.teleop_session.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if subprocess.check_output(['ss','-Htn','state','established','dst','192.168.1.5:8899'],text=True).strip():
        raise RuntimeError('Existing SDK connection; no test')
    with (output/'started').open('x') as f:f.write(datetime.now(timezone(timedelta(hours=8))).isoformat())
    report=dict(stage=args.stage,started=datetime.now(timezone(timedelta(hours=8))).isoformat(),
        sources=sources,native_manifest=manifest,requested_active_s=5.,fixed_tick_batch=args.fixed_tick_batch,high_flow=args.high_flow,axis=args.axis,
        input='programmatic_continuous_permit_no_VR_no_IK',
        pass_scope='finite_program_sequence_feedback_and_confirmed_stop_only',
        execution_quality_qualified=False,
        wall_clock_batch=args.wall_clock_batch,batch_point_limit=args.batch_points,
        refined_stream=args.refined_stream,
        execution_quality_note='Program completion does not certify consumption period, queue freshness, physical derivatives or perceived smoothness.',
        raw_target=('fixed_anchor' if args.stage=='A' else (f'J{args.axis} +/-{sine_amplitude:g}deg fixed-tick sine, certified entry, {sine_peak:.3f}deg/s peak, moving release x5' if args.high_flow else 'J6 +/-0.8deg sinusoid, moving release x5') if args.stage=='CLUTCH' else 'J6 2deg*sin(2*pi*t/5)^3, then anchor'),
        events=[],monitoring=[],pushes=[],errors=[],passed=False)
    client=observer=worker=ack_experiment=None;la=lb=False;signalled=False;active_begin=active_end=None
    last_push=0
    def interrupt(signum,frame):
        nonlocal signalled
        signalled=True
        if worker:worker.revoke('operator_signal',fatal=True)
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,interrupt)
    def network(tag):
        report[tag+'_nic']={k:int(Path('/sys/class/net/eno1/statistics',k).read_text())
            for k in ('tx_errors','rx_errors','tx_dropped','rx_dropped','collisions')}
        report[tag+'_sockets']=subprocess.check_output(['ss','-tin','dst','192.168.1.5:8899'],text=True)
    def sample():
        nonlocal last_push
        status=worker.status();raw=observer.waypoint_cache.feedback()
        news=[v for v in raw['samples'] if v['sequence']>last_push]
        if news:
            report['pushes'].extend(news);last_push=news[-1]['sequence']
        state=status['state_observer']['last_result']
        latest=raw['samples'][-1] if raw['samples'] else None
        now=time.monotonic()
        report['monitoring'].append(dict(time_s=now,phase=status['state'],
            state_received_s=state['host_received_monotonic_s'] if state else None,
            joint_received_s=latest['host_received_monotonic_s'] if latest else None,
            q_rad=latest['q_rad'] if latest else None,
            query=status['state_observer']['sdk_inflight']))
        report['events'].extend(worker.drain_diagnostic_stream(1024))
        return status,latest
    report['motion_profile']=args.motion_profile
    report['motion_limits']=profile.metadata()
    report['sine_config']=dict(amplitude_deg=sine_amplitude,period_s=sine_period,peak_velocity_deg_s=sine_peak)
    network('before')
    try:
        native=load_follow_native(str(runtime))
        if not getattr(native,'supports_graded_hot_feedback',False):raise RuntimeError('Hot-clutch binding missing')
        client=native.J6Follower(six_axis=True,passthrough=True)
        observer=SplitProcessStateObserver(native,str(runtime))
        password=os.environ.get('AUBO_SDK_PASSWORD','123456')
        report['login_a']=client.login('192.168.1.5',8899,'aubo',password);la=record_ok(report['login_a'])
        if not la:raise RuntimeError('A login failed')
        if args.tcp_ack_mode:
            # A is the sole existing SDK socket here. The helper pins its
            # identity before B/C connect; it never guesses among live peers.
            from teleop.aubo.tcp_ack import TcpAckExperiment
            ack_experiment=TcpAckExperiment(host='192.168.1.5',port=8899,mode=args.tcp_ack_mode).start()
            report['tcp_ack_start']=ack_experiment.status()
            if args.wall_clock_batch and report['tcp_ack_start'].get('nodelay_active')!=1:
                raise RuntimeError('NODELAY did not become active on A')
        report['login_bc']=observer.login('192.168.1.5',8899,'aubo',password);lb=record_ok(report['login_bc'])
        password=None
        if not lb:raise RuntimeError('B/C login failed')
        report['subscription_c']=observer.start_waypoint_feedback()
        if not record_ok(report['subscription_c']):raise RuntimeError('C subscription failed')
        report['initial_b']=observer.query_state()
        if not record_ok(report['initial_b']) or report['initial_b']['data']!=0:
            raise RuntimeError('Fresh stopped robot required')
        deadline=time.monotonic()+1
        while not observer.waypoint_cache.feedback()['samples']:
            if time.monotonic()>deadline:raise RuntimeError('C push missing')
            time.sleep(.005)
        client.attach_process_feedback(observer.cache,observer.waypoint_cache)
        report['prepare']=client.prepare()
        if not record_ok(report['prepare']) or not report['prepare']['prepared']:
            raise RuntimeError('Prepare refused: '+str(report['prepare']))
        fixed=tuple(report['prepare']['anchor_rad'])
        if args.stage in ('B','CLUTCH') and not (report['prepare']['joint_ranges_rad'][args.axis-1][0]+math.radians(measured_envelope if args.high_flow else 2.1)<fixed[args.axis-1]
            <report['prepare']['joint_ranges_rad'][args.axis-1][1]-math.radians(measured_envelope if args.high_flow else 2.1)):
            raise RuntimeError('J6 trajectory margin unavailable')
        worker=J6FollowWorker(client,fixed,j6_range_rad=report['prepare']['joint_ranges_rad'][5],
            joint_ranges_rad=report['prepare']['joint_ranges_rad'],waypoint_feedback=True,
            command_mode='passthrough',state_observer=observer,diagnostics=True,
            priming_reanchor_required=True,fixed_tick_batch=args.fixed_tick_batch,batch_point_limit=args.batch_points,
            wall_clock_batch=args.wall_clock_batch,refined_stream=args.refined_stream,motion_profile=args.motion_profile)
        if args.high_flow:
            if not args.fixed_tick_batch:raise RuntimeError('Program sine requires fixed-tick execution')
            from .aubo_program_sine import ProgramSineTrajectory
            worker._trajectory=ProgramSineTrajectory(fixed,worker.joint_ranges_rad,
                axis=args.axis-1,center=fixed,amplitude_deg=sine_amplitude,period_s=sine_period,profile=args.motion_profile)
        worker.enable_diagnostic_stream(32768);worker.start()
        deadline=time.monotonic()+((240 if args.high_flow else 30) if args.stage=='CLUTCH' else 11)
        clutch_phase='cold';clutch_index=0;leg_started=released=None
        report['clutch_cycles']=[]
        while time.monotonic()<deadline:
            status,latest=sample();now=time.monotonic()
            if signalled:raise RuntimeError('Operator signal')
            if status['fatal'] or status['stop_result_unknown'] or status['first_stop_trigger']:
                raise RuntimeError('Worker interrupted: '+str(status['first_stop_trigger'] or status['fault']))
            if status['state']=='active' and active_begin is None:
                active_begin=status['active_since_monotonic_s']
                report['active_begin_s']=active_begin
                print('Stage '+args.stage+' ACTIVE',flush=True)
            target=tuple(status.get('priming_anchor_rad',fixed))
            if status['state']=='priming' and status['priming_ready']:
                # This harness has no hand/IK state. Acknowledge zero anchor;
                # the real VR runtime separately anchors its latest hand pose.
                worker.acknowledge_priming_reanchor(1,now)
            elapsed=0 if active_begin is None else now-active_begin
            if args.stage=='CLUTCH':
                if latest is not None and max(abs(a-b) for a,b in zip(latest['q_rad'],fixed))>math.radians(measured_envelope if args.high_flow else 1.4):
                    raise RuntimeError('Clutch measured pose exceeded finite envelope')
                worker.note_input(now)
                if status['state'] in ('idle','arming','priming'):
                    if clutch_phase!='cold':raise RuntimeError('Unexpected cold mode transition during hot clutch')
                    worker.offer(Candidate(fixed,now,1))
                elif status['state']=='active':
                    if clutch_phase in ('cold','resuming'):
                        leg_started=status['active_since_monotonic_s'];clutch_phase='moving'
                    if clutch_index<5:
                        target=list(fixed);target[args.axis-1]+=math.radians(8. if args.high_flow else .8)*(-1)**clutch_index*math.sin(2*math.pi*(now-leg_started)/(4.8 if args.high_flow else 3))
                        if not status.get('underrun_recovering') and ((args.high_flow and
                                worker._trajectory.program_time>=getattr(worker._trajectory,'bridge_s',float('inf'))+1.25*sine_period and
                                getattr(worker._trajectory,'recovery_bridge',None) is None and
                                max(map(abs,worker._trajectory.v))>=math.radians(sine_peak*.98)) or
                                (not args.high_flow and now-leg_started>=1.5)):
                            if max(map(abs,worker._trajectory.v))<math.radians(.05):
                                raise RuntimeError('Release occurred at rest; moving brake acceptance not demonstrated')
                            row=dict(cycle=clutch_index+1,active_begin_s=leg_started,released_s=now,
                                q_cmd_rad=worker._trajectory.q,v_cmd_rad_s=worker._trajectory.v,
                                a_cmd_rad_s2=worker._trajectory.a)
                            report['clutch_cycles'].append(row)
                            print('CLUTCH release '+str(clutch_index+1),flush=True)
                            worker.revoke('grip_released');released=now;clutch_phase='released'
                        elif not worker.offer(Candidate(tuple(target),now,clutch_index+1)):
                            raise RuntimeError('Clutch active intent refused')
                    else:
                        # Return through the same bounded ACTIVE planner after the fifth resume.
                        if args.high_flow:worker._trajectory.program_enabled=False
                        target=fixed
                        quiet=(max(abs(a-b) for a,b in zip(worker._trajectory.q,fixed))<math.radians(.02) and
                            max(map(abs,worker._trajectory.v))<math.radians(.02) and
                            max(map(abs,worker._trajectory.a))<math.radians(.1) and latest is not None and
                            max(abs(a-b) for a,b in zip(latest['q_rad'],fixed))<math.radians(.05))
                        if quiet:
                            worker.revoke('grip_released');released=now;clutch_phase='final_holding'
                        elif now-leg_started>(15 if args.high_flow else 3):raise RuntimeError('Clutch final return failed to settle')
                        elif not worker.offer(Candidate(fixed,now,6)):raise RuntimeError('Final return refused')
                elif status['state']=='holding' and status['hot_ready']:
                    if clutch_phase=='final_holding' and now-released>=.5:
                        active_end=now;break
                    if clutch_phase=='released' and now-released>=.75:
                        row=report['clutch_cycles'][-1]
                        row.update(holding_ready_s=now,q_hold_rad=status['hold_q_rad'])
                        if not worker.resume_clutch(clutch_index+2,now):
                            raise RuntimeError('Hot resume refused after admission')
                        row['resumed_s']=time.monotonic();clutch_index+=1;clutch_phase='resuming'
                        if args.high_flow and clutch_index<5:
                            # Production resume preserves q/v/a. Re-enable only
                            # this test source after its stopped brake, never
                            # reset the production reference on Grip resume.
                            with worker._lock:
                                worker._trajectory=ProgramSineTrajectory(worker._trajectory.q,worker.joint_ranges_rad,
                                    axis=args.axis-1,center=fixed,
                                    amplitude_deg=(-sine_amplitude if worker._trajectory.q[args.axis-1]>fixed[args.axis-1] else sine_amplitude),
                                    period_s=sine_period,profile=args.motion_profile)
                        print('CLUTCH resume '+str(clutch_index),flush=True)
                time.sleep(.001)
                continue
            if args.stage=='B' and active_begin is not None and elapsed<5:
                q=list(target);q[5]+=math.radians(2)*math.sin(2*math.pi*elapsed/5)**3;target=tuple(q)
            if latest is not None:
                deviation=max(abs(a-b) for a,b in zip(latest['q_rad'],fixed))
                if deviation>math.radians(.1 if args.stage=='A' else 2.2):
                    raise RuntimeError('Measured pose exceeded finite test envelope')
            if active_begin is not None and elapsed>=5:
                if args.stage=='A':active_end=now;break
                quiet=(worker._trajectory is not None and
                    max(abs(a-b) for a,b in zip(worker._trajectory.q,target))<=math.radians(.02) and
                    max(map(abs,worker._trajectory.v))<=math.radians(.02) and latest is not None and
                    max(abs(a-b) for a,b in zip(latest['q_rad'],target))<=math.radians(.05))
                if quiet:active_end=now;break
                if elapsed>=7:raise RuntimeError('Bounded trajectory return did not settle within 2s')
            worker.note_input(now)
            if status['feedback_ready'] and not worker.offer(Candidate(target,now,1)):
                raise RuntimeError('Finite test intent refused')
            time.sleep(.001)
        if active_end is None:raise RuntimeError('Finite acceptance deadline')
        if args.stage=='CLUTCH':
            if len(report['clutch_cycles'])!=5 or not all('resumed_s' in c for c in report['clutch_cycles']):
                raise RuntimeError('Five complete clutch cycles missing')
            if sum(e['event']=='passthrough_enter_returned' for e in report['events'])!=1:
                raise RuntimeError('Expected exactly one cold entry in five-cycle run')
        if args.high_flow and args.stage=='CLUTCH':
            actual_sends=[e['attempt'] for e in report['events'] if e['event']=='send_returned' and e['attempt'].get('phase')=='active']
            peaks=[max(map(abs,(a.get('trajectory') or {}).get('v_cmd_rad_s',[0.]))) for a in actual_sends]
            report['reference_peak_deg_s']=math.degrees(max(peaks,default=0.))
            report['reference_excursion_deg']=math.degrees(max((abs(a['requested_q_rad'][args.axis-1]-fixed[args.axis-1]) for a in actual_sends),default=0.))
            if report['reference_peak_deg_s']<sine_peak*.9 or report['reference_excursion_deg']<sine_amplitude*.83:
                raise RuntimeError('Configured high-flow reference speed/excursion not demonstrated')
        report['completed_active_window']=True
        report['active_end_s']=active_end
        worker.revoke('staged_acceptance_complete')
        stop_deadline=time.monotonic()+5
        while time.monotonic()<stop_deadline:
            status,_=sample()
            if status['stop_confirmed'] or status['stop_result_unknown']:break
            time.sleep(.001)
        report['after_stop']=worker.status()
        if not status['stop_confirmed'] or status['stop_result_unknown']:
            raise RuntimeError('Stop not confirmed')
    except Exception as exc:
        report['exception']=dict(type=type(exc).__name__,message=str(exc))
        if worker:worker.revoke('acceptance_exception',fatal=True)
    finally:
        if worker:
            try:worker.close()
            except Exception as exc:report['close_error']=str(exc)
            report['events'].extend(worker.drain_diagnostic_stream(32768))
            report['final_status']=worker.status()
        if ack_experiment:
            try:
                report['tcp_ack_end']=ack_experiment.stop()
                if report['tcp_ack_end']['alive'] or report['tcp_ack_end']['error']:
                    raise RuntimeError('ACK helper did not stop cleanly')
            except Exception as exc:report['close_error']='ACK helper: '+str(exc)
        busy=worker is not None and worker._thread.is_alive()
        unknown=worker is not None and worker.status()['stop_result_unknown']
        if la:
            if not busy and not unknown and not (ack_experiment and ack_experiment.status()['alive']):
                try:report['logout_a']=client.logout()
                except Exception as exc:report['logout_a_error']=str(exc)
            else:client.abandon_cleanup()
        if lb:
            try:
                if worker and worker.state_query._thread.is_alive():observer.abandon_cleanup()
                else:report['logout_bc']=observer.logout()
            except Exception as exc:report['logout_bc_error']=str(exc)
        elif observer:observer.abandon_cleanup()
        network('after')
        if active_begin is not None and active_end is not None:
            active_rows=[x for x in report['monitoring'] if active_begin<=x['time_s']<=active_end]
            stats={}
            for field in ('state_received_s','joint_received_s'):
                rows=[x for x in report['monitoring'] if x[field] is not None]
                age=max((x['time_s']-x[field] for x in active_rows),default=float('inf'))
                upper=max((min(b['time_s'],active_end)-a[field] for a,b in zip(rows,rows[1:])
                    if b['time_s']>=active_begin and a['time_s']<=active_end),default=float('inf'))
                stats[field]=dict(sampled_max_ms=age*1000,continuous_upper_bound_ms=upper*1000)
            report['active_age']=stats
            report['active_s']=active_end-active_begin
            report['passed']=bool(report.get('completed_active_window') and
                report.get('after_stop',{}).get('stop_confirmed') and not report['errors'] and
                not report.get('final_status',{}).get('fatal',True) and
                not any(key in report for key in ('exception','close_error','logout_a_error','logout_bc_error')) and
                all(v['continuous_upper_bound_ms']<=500 for k,v in stats.items() if not (args.refined_stream and k=='state_received_s')) and
                report.get('logout_a',{}).get('return_code')==0 and report.get('logout_bc',{}).get('return_code')==0)
        report['finished']=datetime.now(timezone(timedelta(hours=8))).isoformat()
        with (output/'report.json').open('x') as f:json.dump(json_safe(report),f,ensure_ascii=False,indent=2,allow_nan=False)
        print(json.dumps({key:report.get(key) for key in ('stage','passed','active_s','active_age','exception','close_error')},
            ensure_ascii=False,indent=2),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
