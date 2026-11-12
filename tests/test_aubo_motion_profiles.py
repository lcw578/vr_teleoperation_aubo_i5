"""Host/native profile agreement and high-flow brake continuity, offline only."""
import math,time,unittest
from unittest.mock import Mock
from teleop.aubo.motion_profile import PROFILES,motion_profile
from teleop.aubo.trajectory import JointTrajectory
from scripts.aubo_program_sine import ProgramSineTrajectory
from test_aubo_refined_stream import RefinedNativeTests

class ProfileTrajectoryTests(unittest.TestCase):
    def test_profiles_are_per_instance_and_brakes_obey_whole_curve_bounds(self):
        for name,p in PROFILES.items():
            t=JointTrajectory([0.]*6,[[-3,3]]*6,profile=name)
            for i in range(600):t.advance([.8]*6,.005)
            qva=(t.q,t.v,t.a);goal,d=t.brake()
            self.assertEqual(qva,(t.q,t.v,t.a))
            for controls in t.controls:
                for limit in p.radians:
                    controls=t.derivative(controls,d)
                    self.assertLessEqual(max(abs(x) for _,x in t.extrema(controls)),limit+1e-9)
            for i in range(600):t.advance(goal,.005)
            self.assertEqual(t.v,(0.,)*6);self.assertEqual(t.a,(0.,)*6)
        t=JointTrajectory([0.]*6,[[-3,3]]*6)
        self.assertEqual(t.velocity_rad_s,math.radians(10))
        with self.assertRaises(ValueError):motion_profile('turbo')

    def test_practical_brake_tracking_envelope_scales_but_hold_tolerance_does_not(self):
        from test_aubo_fixed_tick_batch import BatchClient
        from test_aubo_follow import Clock
        from teleop.aubo.follow import J6FollowWorker
        clock=Clock();client=BatchClient(clock)
        client.configure_refined_stream=Mock()
        client.configure_motion_limits=Mock(return_value=dict(max_step_deg=.225))
        w=J6FollowWorker(client,[0.]*6,j6_range_rad=(-3,3),joint_ranges_rad=[[-3,3]]*6,
            clock=clock,command_mode='passthrough',waypoint_feedback=True,state_observer=Mock(),
            fixed_tick_batch=True,wall_clock_batch=True,refined_stream=True,batch_point_limit=8,
            motion_profile='practical')
        self.assertAlmostEqual(math.degrees(w.braking_tracking_limit_rad),13.5)
        self.assertAlmostEqual(math.degrees(w.holding_tolerance_rad),.1)
        self.assertEqual(w.buffer_points,18)
        client.configure_motion_limits.assert_called_once_with(45.,180.,1800.,18,18)

    def test_practical_program_reaches_24deg_s_and_step_exceeds_old_limit_without_jump(self):
        period=2*math.pi*8/24
        t=ProgramSineTrajectory([0.]*6,[[-3,3]]*6,axis=0,center=[0.]*6,
            amplitude_deg=8,period_s=period,profile='balanced')
        peak=step=0.;previous=t.q
        for _ in range(3000):
            t.advance([0.]*6,.005)
            peak=max(peak,math.degrees(abs(t.v[0])))
            step=max(step,math.degrees(abs(t.q[0]-previous[0])));previous=t.q
            self.assertLessEqual(max(map(abs,t.a)),math.radians(75)+1e-9)
        self.assertGreater(peak,23.9);self.assertGreater(step,.1);self.assertLessEqual(step,.125+1e-9)
        while abs(t.v[0])<math.radians(23.99):t.advance([0.]*6,.005)
        before=t.q,t.v,t.a;goal,d=t.brake();self.assertEqual(before,(t.q,t.v,t.a))
        for _ in range(600):t.advance(goal,.005)
        self.assertEqual(t.v,(0.,)*6)

    def test_repeated_high_speed_program_entry_uses_near_turning_point(self):
        q=[0.]*6;center=[0.]*6;period=2*math.pi*8/24
        for _ in range(5):
            amplitude=-8 if q[0]>center[0] else 8
            t=ProgramSineTrajectory(q,[[-3,3]]*6,axis=0,center=center,
                amplitude_deg=amplitude,period_s=period,profile='balanced')
            for i in range(2000):
                t.advance(center,.005)
                if t.program_time>t.bridge_s+1.25*period and abs(t.v[0])>=math.radians(23.52):break
            self.assertGreater(abs(t.v[0]),math.radians(23.52))
            goal,_=t.brake()
            for i in range(600):t.advance(goal,.005)
            q=t.q
            self.assertLess(abs(math.degrees(q[0])),13)

    def test_practical_high_flow_sine_and_peak_speed_brake(self):
        t=ProgramSineTrajectory([0.]*6,[[-3,3]]*6,axis=0,center=[0.]*6,
            amplitude_deg=12,period_s=2*math.pi*12/43.2,profile='practical')
        previous=t.q
        for _ in range(3000):
            t.advance([0.]*6,.005)
            self.assertLessEqual(math.degrees(abs(t.q[0]-previous[0])),.225+1e-8)
            self.assertLessEqual(math.degrees(abs(t.a[0])),180+1e-8)
            previous=t.q
            if abs(t.v[0])>math.radians(43.19):break
        self.assertGreater(math.degrees(abs(t.v[0])),43.19)
        before=t.q,t.v,t.a;goal,d=t.brake();self.assertEqual(before,(t.q,t.v,t.a))
        self.assertLess(d,1.)
        for _ in range(600):t.advance(goal,.005)
        self.assertEqual(t.v,(0.,)*6)

class ProfileNativeTests(RefinedNativeTests):
    def configured(self,name='practical',cushion=16,underrun=0):
        self.a.stop();self.publish();self.a.configure_refined_stream()
        p=motion_profile(name)
        ack=self.a.configure_motion_limits(p.velocity_deg_s,p.acceleration_deg_s2,p.jerk_deg_s3,cushion,underrun)
        self.assertAlmostEqual(ack['max_step_deg'],p.step_deg)
        self.token=self.a.revision();self.assertTrue(self.a.arm(self.token))
        # Native retains a 500ms cold-entry cooldown after exit.
        time.sleep(.51);self.publish();self.a.enter_passthrough(self.token)
        for _ in range(12):self.publish();self.a.send_priming(self.token)
        self.publish();self.assertTrue(self.a.finish_priming(self.token))
        return p

    def test_dynamic_cross_and_within_batch_step_guards_for_all_profiles(self):
        for name in PROFILES:
            p=self.configured(name);self.a.clock_batch_plan(self.token,8)
            step=math.radians(p.step_deg*.99)
            self.a.send_batch([[step*i]*6 for i in range(1,9)],self.token,True)
            self.publish();self.a.clock_batch_plan(self.token,8)
            self.a.send_batch([[step*i]*6 for i in range(9,17)],self.token,True)
            time.sleep(.012);self.publish();self.a.clock_batch_plan(self.token,8)
            with self.assertRaisesRegex(RuntimeError,'step'):
                self.a.send_batch([[step*16+math.radians(p.step_deg*1.01)]*6]*2,self.token,True)
            self.tearDown();self.setUp()

    def fill(self):
        for n in (8,8,2):
            self.publish();self.a.clock_batch_plan(self.token,8)
            self.a.send_batch([[0.]*6]*n,self.token,True)

    def test_aligned_ninety_ms_pad_and_injected_long_gap_require_restart(self):
        self.configured(cushion=18,underrun=18);self.fill()
        time.sleep(.075);self.publish();p=self.a.clock_batch_plan(self.token,8)
        self.assertFalse(p['recovery_required']);self.assertEqual(p['nominal_cushion_points'],18)
        self.assertEqual(p['underrun_gap_s'],.09)
        self.a.send_batch([[0.]*6]*p['point_count'],self.token,True)
        time.sleep(.105);self.publish();p=self.a.clock_batch_plan(self.token,8)
        self.assertTrue(p['recovery_required']);self.assertEqual(p['point_count'],0)
        with self.assertRaisesRegex(RuntimeError,'recovery required'):
            self.a.send_batch([[0.]*6]*2,self.token,True)
        self.a.restart_clock_from_rest(self.token)
        p=self.a.clock_batch_plan(self.token,8)
        self.assertEqual(p['point_count'],2);self.assertFalse(p['recovery_required'])

    def test_cumulative_short_gaps_cannot_bypass_net_debt_recovery(self):
        self.configured(cushion=18,underrun=18);self.fill()
        for _ in range(5):
            time.sleep(.065);self.publish();p=self.a.clock_batch_plan(self.token,8)
            if p['recovery_required']:break
            self.a.send_batch([[0.]*6]*p['point_count'],self.token,True)
        self.assertTrue(p['recovery_required']);self.assertGreater(p['net_missing_points'],0)

    def test_native_rechecks_expiry_after_plan_before_sdk_send(self):
        from test_aubo_hot_feedback_native import controls
        self.configured(cushion=18,underrun=18);self.fill()
        time.sleep(.012);self.publish();p=self.a.clock_batch_plan(self.token,8)
        self.assertFalse(p['recovery_required'])
        time.sleep(.10);self.publish();before=controls.batch_counts()
        with self.assertRaisesRegex(RuntimeError,'recovery required'):
            self.a.send_batch([[0.]*6]*2,self.token,True)
        self.assertEqual(before,controls.batch_counts())

    def test_misaligned_threshold_rejected_before_arm(self):
        self.a.stop();self.a.configure_refined_stream()
        with self.assertRaises(ValueError):self.a.configure_motion_limits(45,180,1800,10,18)

    def test_eighteen_point_partial_prefill_is_not_a_recovery_loop(self):
        from test_aubo_hot_feedback_native import controls
        self.configured(cushion=18,underrun=18)
        for n in (8,8):
            self.publish();p=self.a.clock_batch_plan(self.token,8)
            self.assertFalse(p['warmup_complete']);self.assertFalse(p['recovery_required'])
            controls.send_delay_s(.055);self.a.send_batch([[0.]*6]*n,self.token,True)
        controls.send_delay_s(0);self.publish();p=self.a.clock_batch_plan(self.token,8)
        self.assertFalse(p['recovery_required']);self.assertFalse(p['warmup_complete'])
        self.a.send_batch([[0.]*6]*2,self.token,True)
        self.publish();p=self.a.clock_batch_plan(self.token,8)
        self.assertTrue(p['recovery_required']);self.a.restart_clock_from_rest(self.token)
        for _ in range(12):
            self.publish();p=self.a.clock_batch_plan(self.token,8)
            self.assertFalse(p['recovery_required'])
            if p['warmup_complete']:break
            self.a.send_batch([[0.]*6]*p['point_count'],self.token,True)
        self.assertTrue(p['warmup_complete'])

    def test_hot_transitions_preserve_epoch_and_cannot_mint_another_cushion(self):
        self.configured();p=self.a.clock_batch_plan(self.token,8)
        self.a.send_batch([[0.]*6]*8,self.token,True)
        self.a.clock_batch_plan(self.token,8);self.a.send_batch([[0.]*6]*8,self.token,True)
        epoch=p['epoch_monotonic_s']
        for _ in range(20):
            self.publish();holding=self.a.clock_batch_plan(self.token,8,True)
            self.assertGreaterEqual(holding['epoch_monotonic_s'],epoch)
            epoch=holding['epoch_monotonic_s']
            moving=self.a.clock_batch_plan(self.token,8,False)
            self.assertEqual(moving['epoch_monotonic_s'],epoch)
            self.assertEqual(moving['cushion_points'],16)
            self.assertFalse(moving['recovery_required'])

    def test_reconfiguration_while_armed_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'disarmed'):self.a.configure_motion_limits(25,75,600,16)
