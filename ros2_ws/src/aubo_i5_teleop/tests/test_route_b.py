"""Offline regression of frame semantics, actual-state handshake and recording."""
import json
import tempfile
import sqlite3
import math
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import mujoco
import numpy as np

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / "scripts"))
from clutch_pose_mapper import ClutchPoseMapper, quat_mul, quat_conj, rotvec_to_quat, quat_to_rotvec, quat_wxyz_to_xyzw
from teleop_geometry import quest_pose, QUEST_LOCAL_TO_TOOL, tool_pose
from gripper_profile import resolve
from lara_style_tracker import LaraTracker, GROUP, VMAX
from session_manager import SessionManager
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


def qerror(a, b):
    return np.linalg.norm(quat_to_rotvec(quat_mul(a, quat_conj(b))))


def tracker():
    # Execute real tracker callbacks and real MuJoCo FK/Jacobian without DDS.
    harness = SimpleNamespace(q={}, js_time=None, js_fresh=False, control_time=None,
        session=None, engaged=False, fault=None, following_since=None, target=None,
        cmd=None, target_time=0., feedback_timeout=.2, control_timeout=.2,
        following_limit=VMAX*.2, following_duration=.2, _n=0,
        pub=Mock(), pub_stamped=Mock(), pub_status=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**9,
            to_msg=lambda: JointState().header.stamp)), get_logger=lambda: Mock())
    for name in ['_js','_control','_tgt','_feedback_fresh','_measured','_fault','_tick','_publish_status']:
        setattr(harness, name, getattr(LaraTracker, name).__get__(harness))
    harness._stamp_ns = LaraTracker._stamp_ns
    return harness


def feedback(q=None):
    msg = JointState(); msg.name = GROUP
    msg.position = list(resolve()['q_rest'] if q is None else q)
    return msg


def control(epoch, engaged=True, q=None):
    msg = feedback(q); msg.header.stamp.sec = epoch
    msg.header.frame_id = 'engaged' if engaged else 'released'
    return msg


class Frames(unittest.TestCase):
    def test_quest_boundary_and_invalid_quaternion(self):
        p, q, raw = quest_pose([1.,2.,3.], [0.,0.,0.,1.])
        np.testing.assert_allclose(p, [-1.,3.,2.])
        np.testing.assert_allclose(raw, [1.,0.,0.,0.])
        with self.assertRaises(ValueError): quest_pose([0,0,0],[0,0,0,0])

    def test_local_axes_at_multiple_anchor_orientations(self):
        rng = np.random.default_rng(24)
        for _ in range(30):
            controller0 = rotvec_to_quat(rng.normal(size=3))
            tool0 = rotvec_to_quat(rng.normal(size=3))
            # Apply actual LOCAL controller increments, not fixed world increments.
            m = ClutchPoseMapper(controller_to_tool=QUEST_LOCAL_TO_TOOL,
                                rot_reach_limit=2., pos_reach_limit=1.)
            _, c0, _ = quest_pose([0,0,0], quat_wxyz_to_xyzw(controller0))
            m.engage([0,0,0], c0, [0,0,0], tool0)
            expected = tool0.copy(); current = controller0.copy()
            for axis in np.eye(3):
                inc = rotvec_to_quat(axis*.1)
                current = quat_mul(current,inc)
                _, c, _ = quest_pose([0,0,0],quat_wxyz_to_xyzw(current))
                expected = quat_mul(expected,rotvec_to_quat(QUEST_LOCAL_TO_TOOL@ (axis*.1)))
                p, out = m.target([.02,0,0], c, [0,0,0], expected)
                self.assertLess(qerror(out,expected),1e-7)
                np.testing.assert_allclose(p,[.02,0,0],atol=1e-10)
            _, out = m.target([.02,0,0], -c, [0,0,0], expected)
            self.assertLess(qerror(out,expected),1e-7)

    def test_gripper_open_close_does_not_rotate_tool(self):
        for name in ['rg','ag95']:
            profile=resolve(name); model=mujoco.MjModel.from_xml_path(profile['mjcf']); data=mujoco.MjData(model)
            mujoco.mj_resetDataKeyframe(model,data,0)
            bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,profile['ee_body'])
            jid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,profile['grasp_joint'])
            poses=[]
            for value in [profile['open'],profile['close']]:
                data.qpos[model.jnt_qposadr[jid]]=value; mujoco.mj_forward(model,data)
                poses.append(tool_pose(model,data,bid,profile))
            for first,second in zip(*poses): np.testing.assert_allclose(first,second,atol=1e-10)


class Handshake(unittest.TestCase):
    def test_mapper_and_tracker_share_actual_anchor_snapshot(self):
        from clutch_mapper_node import ClutchMapperNode
        h = SimpleNamespace(_q={}, _ee_p=None, _ee_q_wxyz=None, _fk_arrival=None,
            pub_measured=Mock(), pub_control=Mock(), mapper=ClutchPoseMapper(),
            scale=.5, _yaw_comp=False, _ctrl_p=np.zeros(3),
            _ctrl_q_wxyz=np.array([1.,0.,0.,0.]),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:control(2).header.stamp)),
            get_logger=lambda:Mock())
        h._publish_control = ClutchMapperNode._publish_control.__get__(h)
        measured=np.array(feedback().position)-.05
        message=feedback(measured); message.header.frame_id='original_source'
        ClutchMapperNode._on_js(h,message)
        ClutchMapperNode._engage(h)
        self.assertEqual(message.header.frame_id,'original_source')
        actual_control=h.pub_control.publish.call_args[0][0]
        n=tracker(); n._js(feedback(measured+.01)); n._control(actual_control)
        np.testing.assert_allclose(n.cmd,measured)
        position,orientation=h.mapper.target(h._ctrl_p,h._ctrl_q_wxyz,h._ee_p,h._ee_q_wxyz)
        np.testing.assert_allclose(position,h._ee_p)
        self.assertLess(qerror(orientation,h._ee_q_wxyz),1e-7)

    def test_reengage_resets_old_cmd_and_rejects_old_targets(self):
        n=tracker(); n._js(feedback()); n._control(control(1))
        n.cmd += .1
        n._control(control(1,False))
        np.testing.assert_allclose(n.cmd,feedback().position)
        measured=np.array(feedback().position)-.05
        n._js(feedback(measured)); n._control(control(2,q=measured))
        np.testing.assert_allclose(n.cmd,measured)
        target=PoseStamped(); target.header.frame_id='world'; target.header.stamp.sec=1
        target.pose.orientation.w=1.; n._tgt(target)
        self.assertIsNone(n.target)

    def test_missing_and_stale_feedback_stop_commands(self):
        n=tracker(); partial=feedback(); partial.name=GROUP[:-1]; partial.position=partial.position[:-1]
        n._js(partial); self.assertFalse(n._feedback_fresh())
        n._js(feedback()); n._control(control(1)); n.js_time -= 1.
        n._tick(); self.assertEqual(n.fault,'feedback_stale'); n.pub.publish.assert_not_called()
        n._js(feedback()); n._control(control(1)); self.assertFalse(n.engaged)
        n._control(control(2)); self.assertTrue(n.engaged)

    def test_release_stops_old_target_immediately(self):
        n=tracker(); n._js(feedback()); n._control(control(1)); n.target=(np.zeros(3),np.eye(3))
        n._control(control(1,False)); n._tick()
        self.assertIsNone(n.target); np.testing.assert_allclose(n.pub.publish.call_args[0][0].data,feedback().position)

    def test_control_loss_and_persistent_following_error(self):
        n=tracker(); n._js(feedback()); n._control(control(1)); n.control_time-=1
        n._tick(); self.assertEqual(n.fault,'control_stale')
        n._control(control(2)); n.cmd+=2.; n.following_since=time.monotonic()-1
        n._tick(); self.assertEqual(n.fault,'following_error'); self.assertFalse(n.engaged)

    def test_actual_ik_step_is_finite_and_rate_bounded(self):
        n=tracker(); n._js(feedback()); n._control(control(1))
        profile=resolve(); model=mujoco.MjModel.from_xml_path(profile['mjcf']); data=mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model,data,0); mujoco.mj_forward(model,data)
        bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,profile['ee_body'])
        p,R=tool_pose(model,data,bid,profile); n.target=(p+np.array([.005,0,0]),R); n.target_time=time.monotonic()
        before=n.cmd.copy(); n._tick()
        self.assertTrue(np.all(np.isfinite(n.cmd)))
        self.assertTrue(np.all(np.abs(n.cmd-before)<=VMAX*.01+1e-12))

    def test_mujoco_execution_follows_small_translation(self):
        profile=resolve(); model=mujoco.MjModel.from_xml_path(profile['mjcf']); data=mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model,data,0); mujoco.mj_forward(model,data)
        bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,profile['ee_body'])
        adr=[model.jnt_qposadr[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,j)] for j in GROUP]
        p,R=tool_pose(model,data,bid,profile); desired=p+np.array([.01,0,0])
        n=tracker(); n._js(feedback(data.qpos[adr])); n._control(control(1,q=data.qpos[adr]))
        for _ in range(300):
            n._js(feedback(data.qpos[adr])); n.control_time=time.monotonic()
            n.target=(desired,R); n.target_time=time.monotonic(); n._tick()
            data.ctrl[:6]=n.cmd; data.ctrl[6]=profile['open']
            for step in range(5): mujoco.mj_step(model,data)
        actual,_=tool_pose(model,data,bid,profile)
        self.assertIsNone(n.fault); self.assertLess(np.linalg.norm(actual-desired),.003)

    def test_gripper_trigger_independent_of_arm_grip(self):
        from gripper_fsm_node import GripperFSM, GRIP
        from sensor_msgs.msg import Joy
        h=SimpleNamespace(state='OPEN',target_v=GRIP['open'],_prev_btn=0,_toggles=0,get_logger=lambda:Mock())
        message=Joy(); message.buttons=[0,1,0,0]; message.axes=[1.,0.]
        GripperFSM._on_joy(h,message); self.assertEqual(h.state,'CLOSED')
        GripperFSM._on_joy(h,message); self.assertEqual(h._toggles,1)
        message.axes=[0.,0.]; GripperFSM._on_joy(h,message)
        message.axes=[1.,0.]; GripperFSM._on_joy(h,message); self.assertEqual(h.state,'OPEN')

    def test_mock_producer_drives_gripper_analog_contract(self):
        import os
        os.environ.setdefault('PYNPUT_BACKEND','dummy')
        from mock_vr_node import MockVR
        from gripper_fsm_node import GripperFSM, GRIP
        h=SimpleNamespace(_last=time.monotonic(),_t0=time.monotonic(),_script=None,
            _held=set(),_clutch_level=0,_pulse_scale=False,_pulse_grip=True,
            _pulse_pause=False,_pulse_stop=False,p=np.zeros(3),q=np.array([1.,0.,0.,0.]),
            pub_pose=Mock(),pub_joy=Mock(),_n=0,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:control(2).header.stamp)))
        gripper=SimpleNamespace(state='OPEN',target_v=GRIP['open'],_prev_btn=0,
            _toggles=0,get_logger=lambda:Mock())
        MockVR._tick(h)
        joy=h.pub_joy.publish.call_args[0][0]
        self.assertEqual(joy.buttons[0],0)
        self.assertEqual(joy.header.stamp.sec,2)
        GripperFSM._on_joy(gripper,joy)
        self.assertEqual(gripper.state,'CLOSED')
        MockVR._tick(h)
        GripperFSM._on_joy(gripper,h.pub_joy.publish.call_args[0][0])
        self.assertEqual(gripper._toggles,1)


class Recording(unittest.TestCase):
    def test_source_time_preserved_and_headerless_is_explicit(self):
        import io
        clock=SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=300))
        harness=SimpleNamespace(get_clock=lambda:clock, writer=Mock(), sequences={}, timing=io.StringIO())
        from collections import Counter
        harness.sequences=Counter()
        msg=feedback(); msg.header.stamp.nanosec=100
        SessionManager._write(harness,'/joint_states',msg,200,250)
        row=json.loads(harness.timing.getvalue())
        self.assertEqual(row['source_ros_ns'],100); self.assertEqual(row['receive_ros_ns'],200)
        self.assertEqual(row['storage_ros_ns'],300); self.assertEqual(msg.header.stamp.nanosec,100)

    def test_real_rosbag_lifecycle_and_timing_index(self):
        from collections import Counter
        with tempfile.TemporaryDirectory(prefix='route_b_bag_') as directory:
            clock=SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000+time.monotonic_ns()%1000000))
            n=SimpleNamespace(state='IDLE',writer=None,timing=None,bags_dir=Path(directory),
                sequences=Counter(),pub_events=Mock(),pub_state=Mock(),get_clock=lambda:clock,
                get_parameter=lambda key:SimpleNamespace(value=False))
            for method in ['start','pause','stop','_event','_write','_publish_state']:
                setattr(n,method,getattr(SessionManager,method).__get__(n))
            self.assertTrue(n.start()[0]); msg=feedback(); msg.header.stamp.nanosec=12345
            n._write('/joint_states',msg); self.assertTrue(n.pause()[0]); self.assertEqual(n.state,'PAUSED')
            self.assertTrue(n.pause()[0]); self.assertTrue(n.stop()[0])
            rows=[json.loads(row) for row in (n.bag_uri/'timing.jsonl').read_text().splitlines()]
            with sqlite3.connect(next(n.bag_uri.glob('*.db3'))) as connection:
                self.assertEqual(len(rows),connection.execute('select count(*) from messages').fetchone()[0])
            self.assertEqual(next(row for row in rows if row['topic']=='/joint_states')['source_ros_ns'],12345)


if __name__=='__main__': unittest.main()
