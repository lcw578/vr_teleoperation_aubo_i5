"""Workspace fit, sampling and stale spatial-reference refusal; no hardware."""
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
os.environ.setdefault('MUJOCO_GL','egl')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from teleop.aubo.workspace import WorkspaceSampler,workspace_basis,fit_alignment,load_alignment
from teleop.aubo.quest_preview import make_shadow
from teleop.aubo.runtime import OfflineRuntime


class Clock:
    def __init__(self):self.now=100.
    def __call__(self):return self.now
    def tick(self):self.now+=.02;return self.now


def frame(seq,pos,grip,ref='spatial',epoch='ws1',quat=None):
    return dict(type='xr_frame',epoch=epoch,reference_id=ref,seq=seq,t_client=seq*20.,
        controllers={'right':dict(position=list(pos),orientation=quat or [0.,0.,0.,1.],
            buttons=[dict(p=False,v=0.),dict(p=grip,v=float(grip))])})


class FitTests(unittest.TestCase):
    def setUp(self):
        # Forward +X, right -Y, up +Z.
        self.B=np.column_stack(([0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]))
    def test_known_yaw_fits_all_six_directions_and_preserves_up(self):
        for yaw in (0.,.6,-1.2,math.pi/2):
            r=np.array([math.cos(yaw),0.,math.sin(yaw)])
            f=np.cross([0,1,0],r)
            R,e=fit_alignment(self.B,dict(right=r,forward=f,up=[0,1,0]))
            for v,desired in ((r,self.B[:,0]),(f,self.B[:,1]),(np.array([0,1,0]),self.B[:,2])):
                np.testing.assert_allclose(R@v,desired,atol=1e-12)
                np.testing.assert_allclose(R@(-v),-desired,atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(R),1.)
    def test_photo_forward_is_horizontal_flange_axis_and_stays_fixed(self):
        T=np.eye(4);T[:3,:3]=[[0.,-.6,.8],[1.,0.,0.],[0.,.8,.6]]
        B=workspace_basis(T)
        np.testing.assert_allclose(B,self.B,atol=1e-12)
        T[:3,:3]=np.eye(3)
        np.testing.assert_allclose(B,self.B,atol=1e-12)
        with self.assertRaisesRegex(ValueError,'vertical'):workspace_basis(T)
    def test_invalid_or_inconsistent_sweeps_are_refused(self):
        for sweeps in (dict(right=[1,0,0],forward=[1,0,0],up=[0,1,0]),
                       dict(right=[1,0,0],forward=[0,0,1],up=[0,1,0]),
                       dict(right=[1,1,0],forward=[0,0,-1],up=[0,1,0]),
                       dict(right=[1,0,0],forward=[0,0,-1],up=[0,-1,0]),
                       dict(right=[float('nan'),0,0],forward=[0,0,-1],up=[0,1,0])):
            with self.assertRaises(ValueError):fit_alignment(self.B,sweeps)
    def test_override_maps_author_translation_and_world_rotation_consistently(self):
        from vr_teleop_kit.core.pose_mapping import ClutchPoseMapper,rotvec_to_quat
        R,_=fit_alignment(self.B,dict(right=[1,0,0],forward=[0,0,-1],up=[0,1,0]))
        s=make_shadow([.18,.26,2.37,2.04,1.51,.8],[[-3.05,3.05]]*6,quest_to_base=R)
        try:
            np.testing.assert_allclose(s.core._arms['right']['mapper'].R,R)
            self.assertFalse(s.core.config.head_yaw_alignment)
            m=ClutchPoseMapper(R=R,scale=1.,rot_reach_limit=None,pos_reach_limit=None)
            m.engage(np.zeros(3),np.array([1.,0,0,0]),np.zeros(3),np.array([1.,0,0,0]))
            pos,quat=m.target([.1,0,0],rotvec_to_quat([.1,0,0]))
            np.testing.assert_allclose(pos,[0,-.1,0],atol=1e-12)
            np.testing.assert_allclose(quat,rotvec_to_quat([0,-.1,0]),atol=1e-12)
        finally:s.close()
    def test_simulation_default_matrix_unchanged(self):
        from teleop.mujoco_session import QUEST_TO_BASE
        s=make_shadow([.18,.26,2.37,2.04,1.51,.8],[[-3.05,3.05]]*6)
        try:np.testing.assert_array_equal(s.core.config.r_calib,QUEST_TO_BASE)
        finally:s.close()
    def test_nonrotation_override_refused(self):
        for R in (np.zeros((3,3)),np.diag([-1,1,1]),np.eye(4)):
            with self.assertRaises(ValueError):make_shadow([0.]*6,[[-3.05,3.05]]*6,quest_to_base=R)


class SamplerTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/'alignment.json';self.clock=Clock()
        self.s=WorkspaceSampler(np.column_stack(([0,-1,0],[1,0,0],[0,0,1])),self.path,
            provenance={'synthetic':True},clock=self.clock);self.seq=0
    def send(self,pos,grip,**kw):
        self.seq+=1;self.clock.tick()
        return self.s.submit_xr_frame(frame(self.seq,pos,grip,**kw),received_monotonic_s=self.clock())
    def sweep(self,delta):
        self.send([0,0,0],False)
        for _ in range(20):self.send([0,0,0],True)
        for t in np.linspace(0,1,30):self.send(np.array(delta)*t,True)
        for _ in range(20):self.send(delta,True)
        self.send(delta,False)
    def test_explicit_start_required_no_grip_can_move_robot(self):
        self.sweep([.15,0,0]);self.assertEqual(self.s.status()['state'],'waiting_start')
        self.assertFalse(self.path.exists());self.assertFalse(self.s.status()['motion_available'])
    def test_full_three_sweeps_save_fixed_alignment_without_raw_pose_log(self):
        self.s.start();self.sweep([.15,0,0]);self.sweep([0,0,-.15]);self.sweep([0,.15,0])
        self.assertEqual(self.s.status()['state'],'saved')
        saved=load_alignment(self.path)
        self.assertEqual(saved['xr_reference_id'],'spatial')
        self.assertNotIn('samples',saved)
        self.assertFalse(saved['head_yaw_alignment'])
        self.assertLess(self.path.stat().st_size,3000)
    def test_reference_change_requires_starting_all_sweeps_again(self):
        self.s.start();self.sweep([.15,0,0]);self.send([0,0,0],False,ref='new-space')
        self.assertEqual(self.s.status()['state'],'waiting_start');self.assertFalse(self.path.exists())
    def test_short_motion_cannot_be_saved(self):
        self.s.start();self.sweep([.02,0,0]);self.assertEqual(self.s.stage,0)
        self.assertIn('8 to 40',self.s.error);self.assertFalse(self.path.exists())
    def test_large_rotation_is_rejected(self):
        self.s.start();self.send([0,0,0],False)
        for _ in range(30):self.send([0,0,0],True)
        for _ in range(30):self.send([.15,0,0],True,quat=[0,0,.5,math.sqrt(.75)])
        self.send([.15,0,0],False)
        self.assertIn('rotated',self.s.error);self.assertFalse(self.path.exists())
    def test_gap_revokes_current_capture(self):
        self.s.start();self.send([0,0,0],False);self.send([0,0,0],True)
        self.clock.now+=.3;self.send([.15,0,0],True)
        self.assertEqual(self.s.state,'ready');self.assertTrue(self.s.need_release)
        self.assertFalse(self.path.exists())
    def test_missing_spatial_id_cannot_calibrate(self):
        self.s.start();self.send([0,0,0],False);self.send([0,0,0],True)
        self.assertFalse(self.send([0,0,0],True,ref=None));self.assertEqual(self.s.state,'ready')
    def test_reconnect_keeps_completed_sweeps_but_discards_current_sweep(self):
        self.s.start();self.sweep([.15,0,0]);self.send([0,0,0],True)
        self.send([0,0,-.1],True,epoch='ws2')
        self.assertEqual(self.s.stage,1);self.assertTrue(self.s.need_release)
    def test_file_tampering_is_rejected(self):
        self.s.start();self.sweep([.15,0,0]);self.sweep([0,0,-.15]);self.sweep([0,.15,0])
        data=json.loads(self.path.read_text());data['quest_to_base'][0][0]=float('nan')
        self.path.write_text(json.dumps(data))
        with self.assertRaises(ValueError):load_alignment(self.path)


class ReferenceGateTests(unittest.TestCase):
    def setUp(self):
        self.s=make_shadow([.18,.26,2.37,2.04,1.51,.8],[[-3.05,3.05]]*6)
        self.addCleanup(self.s.close)
        self.r=OfflineRuntime(self.s,control_dt=.005,dispatch_fake=False,expected_reference_id='spatial')
    def test_changed_or_missing_spatial_reference_rejected(self):
        for ref in ('other',None):
            self.assertFalse(self.r.submit_xr_frame(frame(0,[0,0,0],True,ref=ref)))
            self.assertEqual(self.r.status()['xr_input']['last_error'],'workspace_alignment_reference_mismatch')
    def test_transport_reconnect_with_same_spatial_reference_is_allowed(self):
        self.assertTrue(self.r.submit_xr_frame(frame(0,[0,0,0],False)))
        self.assertTrue(self.r.submit_xr_frame(frame(0,[0,0,0],False,epoch='ws2')))
        self.assertEqual(self.r._last_epoch,'ws2')
    def test_head_rotation_never_enters_alignment(self):
        f=frame(0,[0,0,0],False);f['viewer']={'orientation':[0,1,0,0]}
        self.assertTrue(self.r.submit_xr_frame(f))
        from teleop.mujoco_session import QUEST_TO_BASE
        np.testing.assert_array_equal(self.s.core._arms['right']['mapper'].R,QUEST_TO_BASE)


if __name__=='__main__':unittest.main(verbosity=2)
