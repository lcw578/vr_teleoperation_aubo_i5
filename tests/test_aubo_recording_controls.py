"""Formal recording controls exercised without robot, serial or camera I/O."""
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from test_aubo_follow import Clock
from test_aubo_priming import PrimingClient
from test_aubo_recording import frame
from teleop.aubo.follow import J6FollowWorker
from teleop.aubo.home import HOME_ACCEL_RAD_S2, HOME_SPEED_RAD_S
from teleop.aubo.quest_follow import J6QuestRuntime
from teleop.aubo.quest_preview import make_shadow
from teleop.aubo.recording import RawRecorder


class HomeClient(PrimingClient):
    def query_diagnosis(self):
        from test_aubo_follow import result
        return dict(result(self.clock),data=dict(buffer_data_raw=0))

    def send_batch(self,points,token):
        for q in points:
            answer=self.send_joints(q,token)
        return answer


class FormalRecordingControlsTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock()
        self.client=HomeClient(self.clock)
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.,3.),
            joint_ranges_rad=[[-3.,3.]]*6,clock=self.clock,command_mode='passthrough',diagnostics=True)
        self.shadow=make_shadow([0.]*6,[[-3.,3.]]*6,input_timeout_s=1.)
        self.shadow.clock=self.shadow.session.clock=self.shadow.session.cache.clock=self.clock
        self.reference=dict(path='synthetic-home',q_rad=(.03,-.02,.01,0.,0.,.02))
        self.runtime=J6QuestRuntime(self.shadow,self.worker,home_reference=self.reference,
            home_trigger='oneshot',control_dt=.005,clock=self.clock)
        self.seq=0
        clock_patch=patch('vr_teleop_kit.core.quest_teleop.time.monotonic',self.clock)
        clock_patch.start();self.addCleanup(clock_patch.stop)
        stop_patch=patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051))
        stop_patch.start();self.addCleanup(stop_patch.stop)
        self.addCleanup(self.runtime.close)

    def tick(self,*,grip=False,home=False,precision=False,mark=False):
        self.clock.advance(.008)
        self.runtime.submit_feedback(self.client.q,received_monotonic_s=self.clock())
        right=tuple(i for i,on in ((1,grip),(3,mark),(4,precision)) if on)
        self.runtime.submit_xr_frame(frame(self.seq,self.clock(),right=right,left=(5,) if home else ()))
        self.seq+=1
        candidate=self.runtime.step()
        self.worker._cycle()
        return candidate

    def complete_home(self):
        for _ in range(900):
            self.tick()
            if self.runtime.home['state']=='complete':break
            self.assertFalse(self.worker.status()['fatal'],self.worker.status()['fault'])
            self.assertNotIn(self.runtime.home['state'],('cancelled','failed'))
        self.assertEqual(self.runtime.home['state'],'complete')
        self.assertTrue(self.worker.status()['stop_confirmed'])
        self.assertLess(max(abs(a-b) for a,b in zip(self.client.q,self.reference['q_rad'])),math.radians(.15))

    def test_single_y_completes_passthrough_home_without_grip_or_holding_y(self):
        self.tick();self.tick(home=True)
        self.complete_home()
        self.assertFalse(self.runtime._last_grip_state)
        self.assertFalse(self.shadow.session.engaged)
        before=len(self.client.sent)
        for _ in range(6):self.tick()
        self.assertEqual(len(self.client.sent),before)
        self.assertLessEqual(self.worker._trajectory.velocity_rad_s,HOME_SPEED_RAD_S)
        self.assertLessEqual(self.worker._trajectory.acceleration_rad_s2,HOME_ACCEL_RAD_S2)

    def test_fixed_tick_batches_also_complete_home_and_keep_limits(self):
        # The fake transport's watermark reports empty slots; it has no SDK.
        self.worker.fixed_tick_batch=True
        self.tick();self.tick(home=True)
        self.complete_home()
        events=self.worker.diagnostic_report()['events']
        self.assertTrue(any(e['event']=='priming_completed' for e in events))
        self.assertTrue(any(e['event']=='send_returned' and 'trajectory' in e.get('attempt',{}) for e in events))

    def test_second_y_cancels_during_priming_before_any_nonanchor_target(self):
        self.tick();self.tick(home=True)
        for _ in range(20):
            self.tick()
            if self.worker.status()['state']=='priming':break
        self.assertEqual(self.worker.status()['state'],'priming')
        self.tick(home=True)
        self.assertEqual(self.runtime.home['state'],'cancelled')
        self.assertEqual(self.runtime.home['error'],'home_abort_pressed')
        for _ in range(5):self.tick()
        self.assertTrue(all(q==(0.,)*6 for q in self.client.sent))

    def test_y_takes_over_active_teleop_and_returns_after_stop(self):
        self.tick()
        for _ in range(35):self.tick(grip=True)
        self.assertEqual(self.worker.status()['state'],'active')
        self.tick(grip=True,home=True)
        self.complete_home()

    def test_right_a_halves_translation_and_rotation_only_while_held(self):
        self.tick()
        for _ in range(35):self.tick(grip=True)
        mapper=self.shadow.core._arms['right']['mapper']
        config=self.shadow.core.config
        self.assertEqual(mapper.scale,config.scale_translation)
        for _ in range(3):self.tick(grip=True,precision=True)
        self.assertEqual(mapper.scale,config.scale_translation*.5)
        self.assertEqual(mapper.scale_rotation,config.scale_rotation*.5)
        self.assertTrue(self.runtime.status()['precision_mode'])
        self.tick(grip=True)
        self.assertEqual(mapper.scale,config.scale_translation)
        self.assertEqual(mapper.scale_rotation,config.scale_rotation)
        self.assertFalse(self.runtime.status()['precision_mode'])

    def test_short_marker_presses_survive_latest_slot_and_have_persistent_hud_ack(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder=RawRecorder(Path(directory)/'raw',task='Synthetic mark',clock=self.clock)
            self.runtime.recorder=recorder
            try:
                recorder.begin()
                self.tick()
                # Press and release both arrive before the owner processes a tick.
                for mark in (True,True,False,True,False):
                    self.clock.advance(.001)
                    self.runtime.submit_xr_frame(frame(self.seq,self.clock(),right=(3,) if mark else ()))
                    self.seq+=1
                self.runtime.step()
                status=self.runtime.status()
                self.assertEqual(status['mark_count'],2)
                self.assertTrue(status['last_mark']['recorded'])
                self.assertEqual(status['last_mark']['count'],2)
                self.assertEqual(status['last_mark']['episode_id'],0)
                self.tick() # Other operator messages must not clear the mark.
                self.assertEqual(self.runtime.status()['last_mark']['count'],2)
                self.assertFalse(self.shadow.core._arms['right']['ramp_active'])
            finally:
                recorder.close();self.runtime.recorder=None
            rows=[json.loads(line) for line in (Path(directory)/'raw/streams.jsonl').read_text().splitlines()]
            self.assertEqual([r['mark_id'] for r in rows if r.get('command')=='mark'],[1,2])

    def test_marker_without_recorder_reports_unsaved(self):
        self.tick();self.tick(mark=True)
        self.assertFalse(self.runtime.status()['last_mark']['recorded'])

    def test_home_feedback_fault_stops_automatic_motion(self):
        self.tick();self.tick(home=True)
        for _ in range(60):
            self.tick()
            if self.runtime.home['state']=='moving':break
        self.assertEqual(self.runtime.home['state'],'moving')
        self.worker.revoke('synthetic_feedback_failure',fatal=True)
        self.tick()
        self.assertEqual(self.runtime.home['state'],'failed')
        self.assertFalse(self.client.permitted)

    def test_held_y_at_connection_never_starts_home_without_new_press(self):
        for _ in range(10):self.tick(home=True)
        self.assertEqual(self.runtime.home['state'],'idle')
        self.assertFalse(self.client.sent)
        self.tick();self.tick(home=True)
        self.complete_home()

    def test_teleop_profile_restored_after_home_and_cold_reengagement(self):
        self.tick();self.tick(home=True);self.complete_home()
        self.assertLess(self.worker._trajectory.acceleration_rad_s2,self.worker.motion_profile.radians[1])
        self.tick()
        for _ in range(150):
            self.tick(grip=True)
            if self.worker.status()['state']=='active':break
        self.assertEqual(self.worker.status()['state'],'active')
        self.assertEqual(self.worker._trajectory.acceleration_rad_s2,self.worker.motion_profile.radians[1])

    def test_home_pause_keeps_automatic_motion_label_and_raw_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder=RawRecorder(Path(directory)/'raw',task='Synthetic home',clock=self.clock)
            self.runtime.recorder=self.worker.recorder=recorder
            try:
                recorder.begin();self.tick();self.tick(home=True)
                for _ in range(80):
                    self.tick()
                    if self.runtime.home['state']=='moving':break
                self.assertEqual(self.runtime.home['state'],'moving')
                for _ in range(12):self.tick()
                self.assertEqual(recorder.status()['pause_reason'],'home_return')
            finally:
                recorder.close();self.runtime.recorder=self.worker.recorder=None
            rows=[json.loads(line) for line in (Path(directory)/'raw/streams.jsonl').read_text().splitlines()]
            self.assertTrue(any(r.get('kind')=='sample' and r.get('control_source')=='home_return'
                and r.get('pause_type')=='automatic_motion' for r in rows))


if __name__=='__main__':
    unittest.main()
