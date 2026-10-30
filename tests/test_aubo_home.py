"""Isolated home return with synthetic measurements and fake SDK only."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_aubo_follow import Clock
from test_aubo_six_axis import SixClient
from test_aubo_recording import frame, accepted, sample
from teleop.aubo.follow import J6FollowWorker
from teleop.aubo.home import HOME_SPEED_RAD_S, HOME_ACCEL_RAD_S2, bounded_home_step, load_home
from teleop.aubo.quest_follow import J6QuestRuntime
from teleop.aubo.quest_preview import make_shadow
from teleop.aubo.export_lerobot import segments
from teleop.aubo.recording import RawRecorder


class HomeMathTests(unittest.TestCase):
    def test_reference_requires_real_schema_finite_positions_and_current_ranges(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'synthetic-home.json'
            value=dict(schema='aubo_reference_home_v1',joint_source='sdk_waypoint_push',
                       joint_names=[f'joint_{n}' for n in range(1,7)],q_rad=[.1]*6,
                       synthetic=True,hardware_connected=False)
            path.write_text(json.dumps(value))
            self.assertEqual(load_home(path,[[-3.,3.]]*6)['q_rad'],(.1,)*6)
            for key,bad in [('schema','wrong'),('q_rad',[float('nan')]*6),('q_rad',[4.]*6)]:
                path.write_text(json.dumps(dict(value,**{key:bad})))
                with self.assertRaises(ValueError):load_home(path,[[-3.,3.]]*6)

    def test_steps_start_gently_bound_velocity_and_never_overshoot(self):
        target=(.1,-.2,.05,-.01,0.,.001)
        previous=velocity=(0.,)*6
        for _ in range(400):
            current,new_velocity=bounded_home_step(previous,target,velocity,.04)
            for a,b,goal in zip(previous,current,target):
                self.assertLessEqual(abs(b-a),HOME_SPEED_RAD_S*.04+1e-12)
                self.assertLessEqual(min(a,goal)-1e-12,b)
                self.assertLessEqual(b,max(a,goal)+1e-12)
            previous,velocity=current,new_velocity
        self.assertLess(max(abs(a-b) for a,b in zip(previous,target)),1e-7)
        first,_=bounded_home_step((0.,)*6,(1.,)*6,(0.,)*6,.04)
        self.assertAlmostEqual(first[0],HOME_ACCEL_RAD_S2*.04**2)
        with self.assertRaises(ValueError):bounded_home_step(previous,target,velocity,.3)

    def test_long_returns_reach_twenty_deg_s_with_bounded_acceleration_and_braking(self):
        target=(2.,-2.,1.,-1.,0.,.01)
        for dt in (.005,.04,.05):
            with self.subTest(dt=dt):
                previous=velocity=(0.,)*6
                peak_speed=0.
                for _ in range(math.ceil(20./dt)):
                    current,new_velocity=bounded_home_step(previous,target,velocity,dt)
                    for a,b,goal,old_v,new_v in zip(previous,current,target,velocity,new_velocity):
                        self.assertLessEqual(abs(b-a),math.radians(20.)*dt+1e-12)
                        self.assertLessEqual(abs(new_v-old_v),HOME_ACCEL_RAD_S2*dt+1e-12)
                        self.assertLessEqual(min(a,goal)-1e-12,b)
                        self.assertLessEqual(b,max(a,goal)+1e-12)
                    peak_speed=max(peak_speed,max(abs(v) for v in new_velocity))
                    previous,velocity=current,new_velocity
                self.assertAlmostEqual(math.degrees(peak_speed),20.)
                self.assertLess(max(abs(a-b) for a,b in zip(previous,target)),1e-7)
                self.assertEqual(velocity,(0.,)*6)

    def test_export_excludes_home_commands_even_if_sample_source_is_missing(self):
        command=accepted(10.)
        command['attempt']['control_source']='home_return'
        self.assertEqual(segments([command,sample(10.04)],25),[])
        self.assertEqual(segments([accepted(10.),dict(sample(10.04),control_source='home_return')],25),[])


class HomeRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.client=SixClient(self.clock)
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.,3.),
            joint_ranges_rad=[[-3.,3.]]*6,clock=self.clock,follow_hz=25)
        self.shadow=make_shadow([0.]*6,[[-3.,3.]]*6)
        self.shadow.clock=self.clock;self.shadow.session.clock=self.clock
        self.shadow.session.cache.clock=self.clock
        self.reference=dict(path='isolated-synthetic-reference',q_rad=(.03,-.02,.01,0.,0.,.02))
        self.runtime=J6QuestRuntime(self.shadow,self.worker,home_reference=self.reference,
                                  control_dt=.005,clock=self.clock)
        self.seq=0
        self.patch=patch('vr_teleop_kit.core.quest_teleop.time.monotonic',self.clock)
        self.patch.start();self.addCleanup(self.patch.stop)
        self.stop_patch=patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051))
        self.stop_patch.start();self.addCleanup(self.stop_patch.stop)

    def tick(self,grip=False,dt=.041,*,home=True,left_present=True,epoch='test'):
        self.clock.advance(dt)
        self.shadow.feedback(self.client.q,received_monotonic_s=self.clock())
        self.runtime.submit_xr_frame(frame(self.seq,self.clock(),right=(1,) if grip else (),
            left=(5,) if home else (),left_present=left_present,epoch=epoch))
        self.seq+=1
        candidate=self.runtime.step();self.worker._cycle()
        return candidate

    def prepare_home(self):
        self.tick(False,home=False)
        self.runtime.request_operator_command('return_home');self.tick(False,home=False)
        self.assertEqual(self.runtime.home['state'],'ready')
        self.tick(False)

    def test_web_request_waits_for_left_y_and_right_grip_cannot_start_home(self):
        self.tick(False,home=False);self.tick(True,home=False)
        self.runtime.request_operator_command('return_home');self.tick(True,home=False)
        self.tick(True,home=False);self.tick(True,home=False)
        before=len(self.client.sent)
        self.tick(True,home=False);self.tick(True,home=False)
        self.assertEqual(self.runtime.home['state'],'ready')
        self.assertEqual(len(self.client.sent),before)
        self.tick(False);self.tick(False);self.tick(False)
        self.assertEqual(self.runtime.home['state'],'moving')
        self.assertEqual(self.worker.status()['last_attempt']['control_source'],'home_return')

    def test_left_y_alone_starts_home_without_web_request_or_right_grip(self):
        self.tick(False,home=False)
        for _ in range(5):self.tick(False)
        self.assertEqual(self.runtime.home['state'],'moving')
        self.assertTrue(self.client.sent)
        self.assertFalse(self.runtime._last_grip_state)
        self.assertFalse(self.shadow.session.engaged)

    def test_left_y_takes_over_active_teleop_after_confirmed_stop(self):
        self.tick(False,home=False);self.tick(True,home=False)
        self.assertEqual(self.worker.status()['state'],'active')
        for _ in range(5):self.tick(True)
        self.assertEqual(self.runtime.home['state'],'moving')
        self.assertEqual(self.worker.status()['last_attempt']['control_source'],'home_return')

    def test_reaches_saved_joints_with_bounded_targets_then_confirms_stop(self):
        self.prepare_home()
        for _ in range(150):
            self.tick(False)
            if self.runtime.home['state']=='complete':break
        self.assertEqual(self.runtime.home['state'],'complete')
        self.assertTrue(self.worker.status()['stop_confirmed'])
        self.assertFalse(self.client.permitted)
        self.assertLess(max(abs(a-b) for a,b in zip(self.client.q,self.reference['q_rad'])),math.radians(.15))
        previous=(0.,)*6
        for target in self.client.sent:
            self.assertLessEqual(max(abs(a-b) for a,b in zip(previous,target)),HOME_SPEED_RAD_S*.04+1e-12)
            previous=target
        before=len(self.client.sent)
        self.tick(False);self.tick(False)
        self.assertEqual(len(self.client.sent),before)

    def test_left_y_release_revokes_before_control_tick_and_cancels_home(self):
        self.prepare_home();self.tick(True);self.tick(True)
        self.clock.advance(.005)
        self.runtime.submit_xr_frame(frame(self.seq,self.clock(),right=(),left=()))
        self.seq+=1
        self.assertFalse(self.client.permitted)
        self.tick(False,home=False)
        self.assertEqual(self.runtime.home['state'],'cancelled')
        self.assertFalse(self.client.permitted)

    def test_right_grip_release_does_not_interrupt_held_left_y_return(self):
        self.prepare_home();self.tick(True);self.tick(True)
        self.tick(False);self.tick(False)
        self.assertEqual(self.runtime.home['state'],'moving')
        self.assertTrue(self.client.permitted)

    def test_missing_left_controller_revokes_immediately(self):
        self.prepare_home();self.tick(False);self.tick(False)
        self.runtime.submit_xr_frame(frame(self.seq+1,self.clock(),left_present=False))
        self.assertFalse(self.client.permitted)
        self.runtime.step()
        self.assertEqual(self.runtime.home['state'],'cancelled')

    def test_stale_hold_cancels_and_held_reconnect_cannot_restart(self):
        self.prepare_home();self.tick(False);self.tick(False)
        self.clock.advance(.201)
        self.shadow.feedback(self.client.q,received_monotonic_s=self.clock())
        self.runtime.step();self.worker._cycle()
        self.assertEqual(self.runtime.home['state'],'cancelled')
        self.assertFalse(self.client.permitted)
        self.tick(False);self.tick(False)
        self.assertEqual(self.runtime.home['state'],'cancelled')
        self.tick(False,home=False);self.tick(False);self.tick(False);self.tick(False)
        self.assertEqual(self.runtime.home['state'],'moving')

    def test_epoch_change_stops_and_requires_new_left_y_press(self):
        self.prepare_home();self.tick(False);self.tick(False)
        self.tick(False,epoch='new')
        self.assertEqual(self.runtime.home['state'],'cancelled')
        self.assertFalse(self.client.permitted)

    def test_slow_successful_sdk_call_never_accumulates_target_jump(self):
        self.prepare_home();self.tick(True)
        self.client.send_hook=lambda:self.clock.advance(.23)
        self.tick(True)
        before=self.client.sent[-1]
        self.client.send_hook=None
        self.tick(True)
        self.assertLessEqual(max(abs(a-b) for a,b in zip(before,self.client.sent[-1])),HOME_SPEED_RAD_S*.04+1e-12)

    def test_fault_cancels_home_without_clearing_latch(self):
        self.prepare_home();self.tick(True);self.tick(True)
        self.worker.revoke('synthetic_hardware_fault',fatal=True)
        self.tick(True)
        self.assertEqual(self.runtime.home['state'],'failed')
        self.assertTrue(self.worker.status()['fatal'])
        self.assertFalse(self.client.permitted)

    def test_missing_reference_cannot_start_home_or_submit_targets(self):
        self.runtime=J6QuestRuntime(self.shadow,self.worker,control_dt=.005,clock=self.clock)
        self.tick(False,home=False)
        self.runtime.request_operator_command('return_home');self.tick(False,home=False)
        self.assertEqual(self.runtime.home['state'],'disabled')
        self.assertIn('不可用',self.runtime.last_operator_message)
        self.assertEqual(self.client.sent,[])

    def test_tracking_failure_stops_and_never_reports_completion(self):
        self.reference=dict(path='synthetic-tracking',q_rad=(.5,)*6)
        self.runtime=J6QuestRuntime(self.shadow,self.worker,home_reference=self.reference,
                                  control_dt=.005,clock=self.clock)
        self.prepare_home();self.tick(True);self.tick(True)
        # The feedback remains fresh but reports a large deviation from the
        # most recent accepted target. This is distinct from SDK acceptance.
        self.client.q=[.2]*6
        self.worker.cache.update(self.client.q,received_monotonic_s=self.clock(),source='synthetic')
        self.tick(True)
        self.assertEqual(self.runtime.home['state'],'failed')
        self.assertEqual(self.runtime.home['error'],'home_tracking_error')
        self.assertFalse(self.client.permitted)
        self.assertIsNone(self.runtime.home['completed_monotonic_s'])

    def test_transition_from_home_to_teleop_requires_stop(self):
        self.prepare_home();self.tick(True)
        target=__import__('teleop.aubo.offline_session',fromlist=['Candidate']).Candidate(
            (0.,)*6,self.clock(),self.worker._active_session)
        self.worker.offer(target)
        self.worker._cycle()
        self.assertEqual(self.worker.status()['state'],'stop_requested')
        self.assertEqual(self.worker.status()['fault'],'control_source_changed')

    def test_recording_pauses_for_home_and_keeps_lifecycle_events(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'synthetic'
            recorder=RawRecorder(path,task='Isolated synthetic home test',clock=self.clock,
                                 metadata=dict(synthetic=True,hardware_connected=False))
            self.runtime.recorder=self.worker.recorder=recorder
            try:
                recorder.begin();self.prepare_home()
                for _ in range(150):
                    self.tick(True)
                    if self.runtime.home['state']=='complete':break
                recorder.end()
            finally:
                recorder.close();self.runtime.recorder=self.worker.recorder=None
            rows=[json.loads(line) for line in (path/'streams.jsonl').read_text().splitlines()]
            self.assertTrue(any(r['kind']=='home_return' and r['state']=='complete' for r in rows))
            self.assertFalse(any(r['kind']=='sdk_command' and r['phase']=='returned' for r in rows))
            self.assertFalse(any(r['kind']=='resume' for r in rows))
            self.assertEqual(segments(rows,25),[])

    def test_home_completion_requires_new_grip_and_resumes_same_episode(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder=RawRecorder(Path(directory)/'synthetic',task='Synthetic home resume',clock=self.clock)
            self.runtime.recorder=self.worker.recorder=recorder
            try:
                recorder.begin()
                self.tick(False,home=False);self.tick(True,home=False);self.tick(True,home=False)
                self.assertEqual(recorder.status()['state'],'recording')
                for _ in range(150):
                    self.tick(True)
                    if self.runtime.home['state']=='complete':break
                self.assertEqual(self.runtime.home['state'],'complete')
                before=len(self.client.sent)
                self.tick(True);self.tick(True,home=False);self.tick(True,home=False)
                self.assertEqual(len(self.client.sent),before)
                self.assertEqual(recorder.status()['state'],'paused')
                self.tick(False,home=False);self.tick(True,home=False);self.tick(True,home=False)
                self.assertEqual(recorder.status()['state'],'recording')
                self.assertEqual(recorder.status()['active_episode'],0)
            finally:
                recorder.close();self.runtime.recorder=self.worker.recorder=None


if __name__=='__main__':unittest.main(verbosity=2)
