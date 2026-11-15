"""Unloaded placement with fake SDK, clock and measured feedback only."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_aubo_home as home_tests
from test_aubo_recording import frame, accepted, sample
from teleop.aubo.operator_controls import OperatorButtons
from teleop.aubo.placement import load_placement
from teleop.aubo.recording import RawRecorder
from teleop.aubo.export_lerobot import segments


class PlacementRuntimeTests(unittest.TestCase):
    def setUp(self):
        home_tests.HomeRuntimeTests.setUp(self)
        self.runtime.placement.reference = dict(path='synthetic-basket', q_rad=(.08, -.05, .04, .01, 0., .03),
            home_q_rad=tuple(self.reference['q_rad']))
        self.runtime.placement.status.update(enabled=True, state='idle')
        self.runtime.buttons.placement_enabled = True

    def tick(self, *, left=(), grip=False, epoch='test', dt=.041):
        self.clock.advance(dt)
        self.shadow.feedback(self.client.q, received_monotonic_s=self.clock())
        self.runtime.submit_xr_frame(frame(self.seq, self.clock(), left=left,
            right=(1,) if grip else (), epoch=epoch))
        self.seq += 1
        result = self.runtime.step()
        self.worker._cycle()
        return result

    def start(self):
        self.tick()
        self.tick(left=(4,))
        for _ in range(5): self.tick()
        self.assertTrue(self.runtime.placement.busy())

    def complete(self):
        for _ in range(500):
            self.tick()
            if not self.runtime.placement.busy(): break
        self.assertEqual(self.runtime.placement.status['state'], 'complete')

    def test_left_x_goes_via_home_then_basket_and_stays_stopped(self):
        self.start()
        transfer_seen = False
        for _ in range(500):
            previous_segment = self.runtime.placement.status['segment']
            self.tick()
            if previous_segment == 'transfer' and self.runtime.placement.status['segment'] == 'basket':
                transfer_seen = True
                self.assertTrue(self.worker.status()['stop_confirmed'])
                self.assertLess(max(abs(q-g) for q,g in zip(self.client.q,self.reference['q_rad'])), .003)
            if not self.runtime.placement.busy(): break
        self.assertTrue(transfer_seen)
        self.assertEqual(self.runtime.placement.status['state'], 'complete')
        self.assertTrue(self.worker.status()['stop_confirmed'])
        self.assertFalse(self.client.permitted)
        self.assertLess(max(abs(q-g) for q,g in zip(self.client.q,self.runtime.placement.reference['q_rad'])), .003)
        before = len(self.client.sent)
        for _ in range(5): self.tick()
        self.assertEqual(len(self.client.sent), before)

    def test_grip_is_not_needed_and_commands_are_bounded(self):
        self.start(); self.complete()
        self.assertFalse(self.shadow.session.engaged)
        previous=(0.,)*6
        for q in self.client.sent:
            self.assertLessEqual(max(abs(a-b) for a,b in zip(q,previous)), home_tests.HOME_SPEED_RAD_S*.12+1e-9)
            previous=q
        self.assertEqual(self.worker.status()['last_attempt']['control_source'], 'placement')

    def test_second_x_revokes_before_control_tick_and_cancels_remaining_route(self):
        self.start()
        self.runtime.submit_xr_frame(frame(self.seq,self.clock(),left=(4,)))
        self.seq+=1
        self.assertFalse(self.client.permitted)
        self.runtime.step(); self.worker._cycle()
        self.assertEqual(self.runtime.placement.status['state'], 'cancelled')
        before=len(self.client.sent)
        for _ in range(20):self.tick()
        self.assertEqual(len(self.client.sent),before)

    def test_left_y_is_rejected_during_placement_without_interrupting_it(self):
        self.start(); self.tick(left=(5,)); self.tick()
        self.assertTrue(self.runtime.placement.busy())
        self.assertFalse(self.runtime._home_busy())
        self.complete()

    def test_x_y_simultaneous_never_starts_motion(self):
        self.tick();self.tick(left=(4,5))
        for _ in range(5):self.tick()
        self.assertFalse(self.runtime.placement.busy())
        self.assertFalse(self.runtime._home_busy())
        self.assertEqual(self.client.sent,[])

    def test_x_with_grip_held_does_not_start(self):
        self.tick(); self.tick(left=(4,),grip=True)
        self.assertFalse(self.runtime.placement.busy())

    def test_grip_press_during_automatic_motion_cancels_without_teleop_takeover(self):
        self.start(); self.tick(grip=True)
        self.assertEqual(self.runtime.placement.status['state'],'cancelled')
        self.assertFalse(self.client.permitted)

    def test_input_loss_cancels_and_reconnect_does_not_resume(self):
        self.start(); self.clock.advance(1.01)
        self.runtime.step(); self.worker._cycle()
        self.assertEqual(self.runtime.placement.status['state'],'cancelled')
        before=len(self.client.sent)
        for _ in range(10):self.tick()
        self.assertEqual(len(self.client.sent),before)

    def test_epoch_change_cancels(self):
        self.start(); self.tick(epoch='new')
        self.assertEqual(self.runtime.placement.status['state'],'cancelled')
        self.assertFalse(self.client.permitted)

    def test_invalid_frame_immediately_revokes(self):
        self.start()
        self.runtime.submit_xr_frame({'type':'xr_frame'})
        self.assertFalse(self.client.permitted)
        self.runtime.step();self.worker._cycle()
        self.assertEqual(self.runtime.placement.status['state'],'cancelled')

    def test_paused_recording_keeps_placement_observations_and_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder=RawRecorder(Path(directory)/'raw',task='synthetic unloaded placement',clock=self.clock)
            self.runtime.recorder=self.worker.recorder=recorder
            recorder.begin();recorder.pause_episode('grip_released')
            self.start();self.complete()
            recorder.end();recorder.close()
            rows=[json.loads(line) for line in (Path(directory)/'raw/streams.jsonl').read_text().splitlines()]
            self.assertTrue(any(r['kind']=='placement_observation' and r['measured_q_rad'] for r in rows))
            self.assertTrue(any(r['kind']=='sdk_command' and r['attempt']['control_source']=='placement' for r in rows))
            self.assertTrue(any(r['kind']=='sample' and r['recording_paused'] for r in rows))
            self.assertTrue(any(r['kind']=='planner_target' and r['control_source']=='placement' for r in rows))
            self.assertEqual(segments(rows,25),[])

    def test_fatal_latch_cannot_be_reset_by_x(self):
        self.start(); self.worker._fatal=True
        self.tick()
        self.assertEqual(self.runtime.placement.status['state'],'failed')
        self.tick(left=(4,))
        self.assertTrue(self.worker.status()['fatal'])
        self.assertFalse(self.runtime.placement.busy())


class PlacementReferenceTests(unittest.TestCase):
    def test_load_binds_exact_home_and_rejects_wrong_tool_and_bad_joints(self):
        with tempfile.TemporaryDirectory() as directory:
            home=Path(directory)/'home.json'; home.write_text('synthetic measured home')
            ref=dict(path=str(home),q_rad=(0.,)*6)
            basket=Path(directory)/'basket.json'
            value=dict(schema='aubo_placement_reference_v1',role='basket_above',joint_source='sdk_waypoint_push',
                joint_names=[f'joint_{n}' for n in range(1,7)],tool='bare_flange',gripper_installed=False,
                q_rad=[.1]*6,home_reference=dict(q_rad=[0.]*6,sha256=hashlib.sha256(home.read_bytes()).hexdigest()))
            basket.write_text(json.dumps(value))
            self.assertEqual(load_placement(basket,ref,[[-3,3]]*6)['q_rad'],(.1,)*6)
            for changes in (dict(tool='new_gripper'),dict(gripper_installed=True),dict(q_rad=[float('nan')]*6),dict(q_rad=[4.]*6),dict(schema='bad')):
                basket.write_text(json.dumps(dict(value,**changes)))
                with self.assertRaises(ValueError):load_placement(basket,ref,[[-3,3]]*6)
            basket.write_text(json.dumps(value));home.write_text('modified home')
            with self.assertRaises(ValueError):load_placement(basket,ref,[[-3,3]]*6)

    def test_default_x_is_unassigned_and_reappearing_held_x_is_not_a_press(self):
        for enabled in (False,True):
            buttons=OperatorButtons(placement_enabled=enabled)
            buttons.feed(frame(0,10.,left=(4,)),10.)
            self.assertEqual(buttons.drain(),[])
            buttons.feed(frame(1,10.01),10.01)
            buttons.feed(frame(2,10.02,left=(4,)),10.02)
            self.assertEqual([e['command'] for e in buttons.drain()],['placement'] if enabled else [])

    def test_export_excludes_placement_commands(self):
        command=accepted(10.); command['attempt']['control_source']='placement'
        self.assertEqual(segments([command,sample(10.04)],25),[])


if __name__=='__main__':unittest.main()
