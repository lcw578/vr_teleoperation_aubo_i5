"""Upstream parity, physical tracking and input recovery for the AUBO port."""
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("MUJOCO_GL", "egl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from teleop.mujoco_session import MuJoCoFollower, Recorder, SPEC, RG_READY, load_model, parse_args, recording_path
from vr_teleop_kit.core.pose_mapping import ClutchPoseMapper, quat_mul, quat_conj, rotvec_to_quat


EXPECTED = json.loads((ROOT/"tests/fixtures/vrkit_8d37f3_expected.json").read_text())


class SimulationTests(unittest.TestCase):
    backend = "physics"
    def setUp(self):
        self.p = MuJoCoFollower(backend=self.backend)
        self.seq = 0
        self.clock = patch("vr_teleop_kit.core.quest_teleop.time.monotonic", side_effect=lambda: 1000.+self.p.data.time)
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def frame(self, pos=(0, 0, 0), quat=(1, 0, 0, 0), grip=False, trigger=0., precision=False, rest=False, epoch="test", right=True):
        w,x,y,z = quat
        buttons = [dict(p=False, v=0.) for _ in range(6)]
        buttons[0] = dict(p=bool(trigger), v=trigger)
        buttons[1] = dict(p=grip, v=float(grip))
        buttons[3]["p"] = rest
        buttons[4]["p"] = precision
        msg = dict(type="xr_frame", seq=self.seq, epoch=epoch, t_client=self.p.data.time*1000,
                   controllers={"right":dict(position=list(pos), orientation=[x,y,z,w], buttons=buttons)} if right else {},
                   viewer=dict(orientation=[0., 1., 0., 0.]))
        self.seq += 1
        self.p.teleop.accept_xr_frame(msg)
        return msg

    def cycle(self, **kwargs):
        self.frame(**kwargs); self.p.tick()

    def test_physics_hold_and_model_sites(self):
        import numpy as np
        import mujoco
        q = self.p.data.qpos.copy()
        for _ in range(100): self.p.tick()
        np.testing.assert_allclose(self.p.data.qpos[:6], q[:6], atol=1e-9)
        solver = self.p.arm["solver"]
        solver.fk(q); anchor = solver.j4_anchor_xpos().copy()
        for axis in (3,4,5):
            other=q.copy(); other[axis] -= .2
            solver.fk(other)
            np.testing.assert_allclose(solver.j4_anchor_xpos(), anchor, atol=1e-12)
        np.testing.assert_allclose(self.p.model.body("foreArm_Link").pos, [.408,0,0], atol=1e-6)
        self.assertEqual(self.p.model.nu, 7)
        self.assertEqual(self.p.model.ncam, 3)
        self.assertEqual(mujoco.mj_id2name(self.p.model,mujoco.mjtObj.mjOBJ_SITE,solver.site_id),"tool0")

    def test_reanchor_uses_measured_state(self):
        import numpy as np
        self.p.arm["qpos"][:6] += .2
        measured=self.p.data.qpos.copy()
        self.cycle(grip=True)
        np.testing.assert_allclose(self.p.arm["mapper"]._ee_engage_pos,
                                   self.p.arm["solver"].fk(measured)[0],atol=1e-12)
        np.testing.assert_allclose(self.p.command, measured[:6], atol=1e-9)

    def test_translation_physical_follow_and_stop(self):
        import numpy as np
        self.cycle(grip=True)
        initial=self.p.arm["solver"].fk(self.p.data.qpos)[0].copy()
        peak=0.
        previous_v=self.p.velocity.copy()
        for i in range(600):
            self.cycle(pos=(0,.03*min(i/300,1),0),grip=True)
            peak=max(peak,float(np.max(np.abs(self.p.command-self.p.data.qpos[:6]))))
            self.assertTrue(np.all(np.abs(self.p.velocity)<=self.p.vmax+1e-10))
            self.assertTrue(np.all(np.abs(self.p.velocity-previous_v)<=self.p.amax*self.p.dt+1e-8))
            previous_v=self.p.velocity.copy()
        end=self.p.arm["solver"].fk(self.p.data.qpos)[0]
        self.assertGreater(end[2]-initial[2],.015)
        self.assertLess(peak,.03)
        self.assertIsNone(self.p.fault)
        for _ in range(100): self.cycle(pos=(0,.03,0),grip=False)
        self.assertFalse(self.p.arm["engaged"])
        np.testing.assert_allclose(self.p.velocity,0.,atol=1e-12)

    def test_trigger_independent_binary_and_gap_no_false_edge(self):
        self.cycle(trigger=0.)
        self.cycle(trigger=1.)
        self.assertEqual(self.p.arm["trigger"],1.)
        self.assertFalse(self.p.arm["engaged"])
        for _ in range(50): self.cycle(trigger=1.)
        self.assertEqual(self.p.arm["trigger"],1.)
        for _ in range(50): self.p.tick()
        self.cycle(trigger=1.)
        self.assertEqual(self.p.arm["trigger"],1.)
        self.cycle(trigger=0.); self.cycle(trigger=1.)
        self.assertEqual(self.p.arm["trigger"],0.)

    def test_missing_right_and_recovery_no_catchup(self):
        import numpy as np
        self.cycle(grip=True)
        for _ in range(60): self.cycle(right=False)
        self.assertEqual(self.p.arm["paused"],"input_timeout")
        before=self.p.command.copy()
        self.cycle(pos=(1.,1.,1.),grip=True)
        np.testing.assert_allclose(self.p.arm["mapper"]._ctrl_engage_pos,[1.,1.,1.],atol=1e-10)
        np.testing.assert_allclose(self.p.command,before,atol=1e-9)
        self.assertIsNone(self.p.arm["paused"])

    def test_reference_reset_requires_release(self):
        self.cycle(grip=True)
        self.seq=0
        self.cycle(grip=True, epoch="new")
        self.assertEqual(self.p.arm["paused"],"release_grip_to_reanchor")
        self.assertFalse(self.p.arm["engaged"])
        self.cycle(grip=False, epoch="new")
        self.cycle(grip=True, epoch="new")
        self.assertTrue(self.p.arm["engaged"])

    def test_old_queue_nan_and_out_of_order_rejected(self):
        msg=self.frame()
        self.assertFalse(self.p.teleop.accept_xr_frame(msg))
        msg=dict(msg,seq=self.seq,t_client=-1000.)
        self.assertFalse(self.p.teleop.accept_xr_frame(msg))
        self.seq += 1
        self.frame(pos=(float("nan"),0,0))
        self.assertGreater(self.p.teleop.input_stats["invalid"],0)
        self.assertGreater(self.p.teleop.input_stats["out_of_order"],0)

    def test_hmd_ignored_and_mode_switch_gate(self):
        self.cycle(grip=True)
        self.p.teleop._apply_config_update(dict(rotation_mode="tool_local"))
        self.assertEqual(self.p.teleop.config.rotation_mode,"world")
        self.cycle(grip=False)
        self.p.teleop._apply_config_update(dict(rotation_mode="tool_local"))
        self.assertEqual(self.p.arm["mapper"].rotation_mode,"tool_local")
        self.assertFalse(self.p.teleop.config.head_yaw_alignment)
        self.assertEqual(self.p.arm["mapper"].R.tolist(),[[-1.,0.,0.],[0.,0.,1.],[0.,1.,0.]])

    def test_mode_change_waits_for_physical_stop(self):
        self.cycle(grip=True)
        for i in range(100): self.cycle(pos=(0,.08*i/100,0),grip=True)
        self.cycle(pos=(0,.08,0),grip=False)
        self.assertFalse(self.p.teleop.mode_switch_allowed)
        self.p.teleop._apply_config_update(dict(rotation_mode="tool_local"))
        self.assertEqual(self.p.teleop.config.rotation_mode,"world")
        for _ in range(200): self.cycle(pos=(0,.08,0),grip=False)
        self.assertTrue(self.p.teleop.mode_switch_allowed)
        self.p.teleop._apply_config_update(dict(rotation_mode="tool_local"))
        self.assertEqual(self.p.teleop.config.rotation_mode,"tool_local")

    def test_settings_cannot_raise_velocity_ceiling(self):
        import numpy as np
        self.p.teleop._apply_config_update(dict(max_dq_per_joint_scalar_pos=.5,max_dq_per_joint_scalar_rot=.5,pose_filter_alpha=float("nan")))
        np.testing.assert_allclose(self.p.teleop.config.max_dq_per_joint,self.p.vmax*self.p.dt)
        self.assertEqual(self.p.teleop.config.pose_filter_alpha,.8)

    def test_kinematic_target_is_not_physical_teleport(self):
        import numpy as np
        self.cycle(grip=True)
        measured=self.p.data.qpos.copy()
        self.frame(pos=(.2,0,0),grip=True)
        self.p.teleop.observe("right",self.p.data.qpos,self.p.data.time)
        self.p.teleop.get_action()
        np.testing.assert_array_equal(self.p.data.qpos,measured)
        self.assertGreater(np.linalg.norm(self.p.arm["qpos"][:6]-measured[:6]),0)

    def test_following_error_latches(self):
        # Artificially jam the plant; keep commanding a feasible moving reference.
        import numpy as np
        fixed=self.p.data.qpos.copy()
        self.cycle(grip=True)
        for i in range(800):
            self.p.data.qpos[:6]=fixed[:6]
            self.p.data.qvel[:6]=0.
            self.cycle(pos=(0,.2*min(i/100,1),0),grip=True)
            if self.p.fault: break
        self.assertEqual(self.p.fault,"following_error")
        self.cycle(grip=True)
        self.assertFalse(self.p.arm["engaged"])

    def test_historical_horizontal_demand_with_original_bias(self):
        # Historical physics characterization, not axial retraction: this arm
        # points vertically, whereas the original demand was horizontal.
        import mujoco
        import numpy as np
        self.p.arm["solver"].q_rest_123 = np.array([-1.137355, .041596, 1.593368])
        self.p.data.qpos[:6] = [0.,0.,0.,-.5,.7,0.]
        self.p.command = self.p.data.qpos[:6].copy()
        self.p.data.ctrl[self.p.actuators] = self.p.command
        mujoco.mj_forward(self.p.model,self.p.data)
        self.p.arm["qpos"] = self.p.data.qpos.copy()
        self.cycle(grip=True)
        self.p.arm["solver"].fk(self.p.data.qpos)
        initial=self.p.arm["solver"].j4_anchor_xpos().copy()
        direction=initial.copy(); direction[2]=0.; direction /= np.linalg.norm(direction)
        F=np.asarray(self.p.teleop.config.r_calib)
        for i in range(600):
            self.cycle(pos=F.T@(-direction*.04*min(i/300,1)),grip=True)
        self.p.arm["solver"].fk(self.p.data.qpos)
        final=self.p.arm["solver"].j4_anchor_xpos()
        self.assertGreater(np.dot(initial-final,direction),.01)
        self.assertIsNone(self.p.fault)

    def test_rotation_modes_physical_bounds_and_reverse_at_stop(self):
        import numpy as np
        import mujoco
        for mode in ("world","tool_local"):
            self.p=MuJoCoFollower(rotation_mode=mode, backend="physics")
            self.p.data.qpos[5] = self.p.limit
            self.p.command[5] = self.p.limit
            self.p.data.ctrl[self.p.actuators] = self.p.command
            self.p.arm["qpos"] = self.p.data.qpos.copy()
            mujoco.mj_forward(self.p.model, self.p.data)
            self.seq=0
            self.cycle(grip=True)
            max_accel=0.
            previous_v=self.p.velocity.copy()
            for i in range(800):
                # Push then reverse; wrist3 starts exactly at the working upper bound.
                angle=.7*np.sin(2*np.pi*i/800)
                self.cycle(quat=rotvec_to_quat(np.array([.1*angle,0.,angle])),grip=True)
                max_accel=max(max_accel,float(np.max(np.abs(self.p.velocity-previous_v))/self.p.dt))
                previous_v=self.p.velocity.copy()
                self.assertTrue(np.all(np.abs(self.p.command)<=self.p.limit+1e-10))
                self.assertTrue(np.all(np.abs(self.p.velocity)<=self.p.vmax+1e-10))
            self.assertLessEqual(max_accel,self.p.amax+1e-7)
            self.assertIsNone(self.p.fault)

    def test_author_pivot_arc_observed_on_aubo(self):
        # Characterization of the transplanted policy, not a task-quality claim.
        import numpy as np
        import mujoco
        self.p=MuJoCoFollower(rotation_mode="tool_local", backend="physics")
        self.p.data.qpos[5]=1.8  # Interior wrist3 pose; isolate pivot behavior from the joint stop.
        self.p.command[5]=1.8
        self.p.data.ctrl[self.p.actuators]=self.p.command
        self.p.arm["qpos"]=self.p.data.qpos.copy()
        mujoco.mj_forward(self.p.model,self.p.data)
        initial=self.p.arm["solver"].fk(self.p.data.qpos)[0].copy()
        self.cycle(grip=True)
        for i in range(600):
            self.cycle(quat=rotvec_to_quat(np.array([0.,0.,-.2*min(i/300,1)])),grip=True)
        actual=self.p.arm["solver"].fk(self.p.data.qpos)[0]
        self.assertGreater(np.linalg.norm(actual-initial),.025)
        self.assertLess(np.linalg.norm(actual-self.p.arm["target"][0]),.001)
        self.assertIsNone(self.p.fault)


class UpstreamParityTests(unittest.TestCase):
    def test_world_mapper_matches_pinned_original(self):
        import numpy as np
        kwargs=dict(R=np.array([[-1.,0,0],[0,0,1.],[0,1.,0]]),scale=1.5,scale_rotation=1.5)
        a=ClutchPoseMapper(**kwargs)
        p=np.array([.1,.2,.4]); q=rotvec_to_quat(np.array([.1,.4,-.1])); pivot=np.array([.2,.1,.3])
        a.engage(np.zeros(3),np.array([1.,0,0,0]),p,q,pivot)
        rng=np.random.default_rng(52)
        for right in EXPECTED["world_mapper"]:
            hand=rng.normal(0,.3,3); rot=rotvec_to_quat(rng.normal(0,.4,3))
            left=a.target(hand,rot,p,q)
            for x,y in zip(left,right): np.testing.assert_allclose(x,y,atol=1e-14)

    def test_aubo_solver_matches_original_equations(self):
        import mujoco
        import numpy as np
        from vr_teleop_kit.ik.decoupled_ik import DecoupledIKSolver
        model,data,profile,_=load_model()
        caps=np.asarray(SPEC["velocity_limits_rad_s"])*.005
        # Fixtures were generated at the original source rest pose. Preserve
        # that provenance independently of the user-selected startup preset.
        kwargs=dict(q_rest=[-1.137355, .041596, 1.593368, -1.589821, -1.137355, 2.94],max_dq_per_joint=caps)
        a=DecoupledIKSolver(model=model,**kwargs)
        a.joint_limits[:]=[-2.94,2.94]
        rng=np.random.default_rng(15)
        for expected in EXPECTED["aubo_ik"]:
            seed=data.qpos.copy(); seed[:6]=rng.uniform(-2.8,2.8,6)
            target=rng.uniform(-.8,.8,3); quat=rotvec_to_quat(rng.normal(0,.7,3))
            np.testing.assert_allclose(a.solve(target,quat,seed),expected,atol=1e-14)

    def test_local_axis_rotation_and_wrist_arc(self):
        import numpy as np
        for start in (np.array([1.,0,0,0]),rotvec_to_quat(np.array([0.,1.,0.]))):
            tool=rotvec_to_quat(np.array([.2,-.8,.1]))
            mapper=ClutchPoseMapper(rotation_mode="tool_local",scale=1.,scale_rotation=1.,rot_reach_limit=0,pos_reach_limit=0)
            mapper.engage(np.zeros(3),start,np.array([0.,0.,.2]),tool,np.zeros(3))
            hand=quat_mul(start,rotvec_to_quat(np.array([0.,0.,-.1])))
            p,q=mapper.target(np.zeros(3),hand)
            expected=quat_mul(tool,rotvec_to_quat(np.array([0.,0.,.1])))
            np.testing.assert_allclose(q,expected,atol=1e-12)
            self.assertAlmostEqual(np.linalg.norm(p),.2)

    def test_default_recording_off_and_opt_in_manifest(self):
        self.assertIsNone(parse_args([]).record)
        with tempfile.TemporaryDirectory() as base:
            rec=Recorder(Path(base)/"session",{"source":"test"})
            rec.write({"sim_time":.01,"q_commanded":[0]*6});rec.close()
            row=json.loads((rec.path/"signals.jsonl").read_text())
            self.assertEqual(row["sim_time"],.01)
            self.assertEqual(sorted(p.name for p in rec.path.iterdir()),["manifest.json","signals.jsonl"])

    def test_recording_paths_stay_in_designated_runtime_without_moving_existing_project(self):
        # 2026-11-01: bulk-output root is parameterized (AUBO_RUNTIME_DIR/AUBO_ALLOWED_ROOTS);
        # the invariant under test is "recordings stay inside the designated runtime tree,
        # existing project data is never moved" — not the literal /data mount point.
        runtime = Path(os.environ.get("AUBO_RUNTIME_DIR", "/data/VR_teleoperation_runtime"))
        self.assertTrue(recording_path("auto").is_relative_to(runtime))
        self.assertEqual(recording_path(str(runtime / "explicit")),
                         runtime / "explicit")
        with self.assertRaises(ValueError): recording_path("/tmp/record")


class KinematicTests(unittest.TestCase):
    backend = "kinematic"
    setUp = SimulationTests.setUp
    frame = SimulationTests.frame
    cycle = SimulationTests.cycle

    def test_current_bias_retracts_vertically_extended_arm(self):
        import numpy as np
        import mujoco
        self.p.data.qpos[:6] = [0., 0., 0., -.5, .7, 0.]
        self.p.command = self.p.data.qpos[:6].copy()
        self.p.arm["qpos"] = self.p.data.qpos.copy()
        mujoco.mj_forward(self.p.model, self.p.data)
        self.cycle(grip=True)
        solver = self.p.arm["solver"]
        initial_ee, _ = solver.fk(self.p.data.qpos)
        initial_anchor = solver.j4_anchor_xpos().copy()
        F = np.asarray(self.p.teleop.config.r_calib)
        for i in range(1200):
            # The arm extends vertically upward at q123=0. Demand actual
            # inward motion downward: 4cm hand motion, default 1.5 gain.
            self.cycle(grip=True, pos=F.T @ np.array([0., 0., -.04*min(i/300, 1)]))
            self.assertTrue(np.all(np.isfinite(self.p.command)))
            self.assertTrue(np.all(np.abs(self.p.command) <= self.p.limit))
        final_ee, _ = solver.fk(self.p.data.qpos)
        final_anchor = solver.j4_anchor_xpos().copy()
        self.assertGreater(initial_anchor[2]-final_anchor[2], .04)
        self.assertGreater(initial_ee[2]-final_ee[2], .04)
        # Characterizes the remaining task residual; does not assert perfect
        # six-dimensional tracking by the unchanged author decoupled solver.
        self.assertLess(np.linalg.norm(final_ee-self.p.arm["target"][0]), .02)

    def test_rg_ready_and_home_have_bidirectional_tool_roll_margin(self):
        import numpy as np
        import mujoco
        np.testing.assert_array_equal(self.p.profile["q_rest"], RG_READY)
        self.assertEqual(self.p.profile["q_rest"][5], 0.)
        self.assertGreater(self.p.limit, np.pi/2)
        solver = self.p.arm["solver"]
        tcp, _ = solver.fk(self.p.data.qpos)
        R = solver.data.site_xmat[solver.site_id].reshape(3,3)
        np.testing.assert_allclose(R[:,2], [0.,-1.,0.], atol=1e-6)
        np.testing.assert_allclose(R[:,0], [0.,0.,1.], atol=1e-6)
        cam = self.p.data.cam_xpos[self.p.model.camera("cam_wrist_d405").id]
        self.assertGreater(cam[2]-tcp[2], .10)
        self.assertEqual(self.p.data.ncon, 0)
        key = self.p.model.key("ready").id
        np.testing.assert_array_equal(self.p.model.key_qpos[key, :6], self.p.profile["q_rest"])
        for mode in ("world", "tool_local"):
            for sign in (-1, 1):
                self.p = MuJoCoFollower(rotation_mode=mode)
                self.seq = 0
                initial = self.p.command.copy()
                self.cycle(grip=True)
                for i in range(1000):
                    # Default gain 1.5: 60 degree hand twist requests 90 degree tool twist.
                    angle = sign*np.pi/3*min(i/600, 1)
                    self.cycle(grip=True, quat=rotvec_to_quat(np.array([0.,0.,-angle])))
                    self.assertEqual(self.p.data.ncon, 0)
                self.assertAlmostEqual(self.p.command[5]-initial[5], sign*np.pi/2, places=4)
                self.assertTrue(np.all(np.abs(self.p.command) <= self.p.limit))
                self.cycle(grip=False, rest=True)
                for _ in range(700): self.cycle(grip=False)
                np.testing.assert_allclose(self.p.command, self.p.profile["q_rest"], atol=1e-10)
        # The optional AG95 preset is unchanged.
        self.assertEqual(load_model("ag95")[2]["q_rest"][5], 1.570796)

    def test_default_backend_and_no_physics_or_acceleration(self):
        import numpy as np
        self.assertEqual(parse_args([]).backend, "kinematic")
        self.assertIsNone(parse_args([]).acceleration)
        self.assertIsNone(self.p.amax)
        with patch("mujoco.mj_step", side_effect=AssertionError("preview must never integrate physics")):
            for i in range(200):
                self.cycle(grip=True, pos=(0., .01*min(i/100, 1), 0.))
        self.assertGreater(np.max(np.abs(self.p.command-np.array(self.p.profile["q_rest"]))), .001)
        row = self.p.status()
        self.assertEqual(row["state_source"], "kinematic_preview")
        for key in ("q_measured", "q_velocity", "physics_qpos", "physics_qvel", "gripper_simulated_actuator_force"):
            self.assertIsNone(row[key], key)
        self.assertEqual(row["q_simulated"], row["q_commanded"])
        self.assertAlmostEqual(self.p.data.time, 1.)

    def test_both_modes_display_exact_author_ik_output(self):
        import numpy as np
        for mode in ("world", "tool_local"):
            self.p.teleop.config.rotation_mode = mode
            self.p.arm["mapper"].rotation_mode = mode
            self.cycle(grip=False)
            self.cycle(grip=True)
            with patch.object(self.p.arm["solver"], "solve", wraps=self.p.arm["solver"].solve) as solving:
                for i in range(60):
                    previous = self.p.command.copy()
                    self.cycle(grip=True, pos=(.005*i/60, .004*i/60, 0.),
                               quat=rotvec_to_quat(np.array([.02*i/60, -.03*i/60, .01*i/60])))
                    np.testing.assert_array_equal(self.p.data.qpos[:6], self.p.arm["qpos"][:6])
                    np.testing.assert_array_equal(self.p.command, self.p.data.qpos[:6])
                    self.assertTrue(np.all(np.abs(self.p.command-previous) <= self.p.vmax*self.p.dt + 1e-10))
                    self.assertTrue(np.all(np.abs(self.p.command) <= self.p.limit))
            self.assertEqual(solving.call_count, 60)

    def test_small_step_converges_without_added_command_reversals(self):
        import numpy as np
        self.cycle(grip=True)
        commands = []
        for _ in range(400):
            self.cycle(grip=True, pos=(0., .005, 0.))
            commands.append(self.p.command.copy())
        commands = np.asarray(commands)
        for joint in (2, 3):
            delta = np.diff(commands[:, joint])
            delta = delta[np.abs(delta) > 1e-7]
            self.assertFalse(np.any(delta[1:]*delta[:-1] < 0))
        self.assertLess(np.max(np.ptp(commands[-100:], axis=0)), 1e-9)

    def test_release_gap_and_reference_reset_hold_preview(self):
        import numpy as np
        self.cycle(grip=True)
        for _ in range(40): self.cycle(grip=True, pos=(0., .01, 0.))
        self.cycle(grip=False)
        stopped = self.p.data.qpos[:6].copy()
        for _ in range(30): self.cycle(grip=False, pos=(0., .3, 0.))
        np.testing.assert_array_equal(self.p.data.qpos[:6], stopped)
        self.cycle(grip=True, pos=(0., .3, 0.))
        np.testing.assert_allclose(self.p.arm["mapper"]._ee_engage_pos,
                                   self.p.arm["solver"].fk(stopped)[0], atol=1e-12)
        # The original solver's rest bias can move a non-rest pose slightly
        # even on the engage tick. Do not change that algorithm for this test.
        self.assertLess(np.max(np.abs(self.p.data.qpos[:6]-stopped)), .001)
        for _ in range(50): self.cycle(right=False)
        self.assertIsNotNone(self.p.arm["paused"])
        stopped = self.p.data.qpos[:6].copy()
        for _ in range(20): self.cycle(right=False)
        np.testing.assert_array_equal(self.p.data.qpos[:6], stopped)
        self.cycle(grip=True, epoch="new-space")
        self.assertTrue(self.p.arm["require_release"])
        np.testing.assert_allclose(self.p.data.qpos[:6], stopped, atol=1e-12)

    def test_binary_grippers_use_existing_mimic_geometry(self):
        import numpy as np
        for profile in ("rg", "ag95"):
            self.p = MuJoCoFollower(gripper=profile)
            self.seq = 0
            self.cycle(trigger=0.)
            self.cycle(trigger=1.)
            self.assertFalse(self.p.arm["engaged"])
            self.assertEqual(self.p.data.qpos[self.p.gripper_joint], self.p.profile["close"])
            if profile == "rg":
                self.assertAlmostEqual(self.p.data.qpos[self.p.model.joint("rg_right_finger_joint").qposadr[0]],
                                       -.020995*self.p.profile["close"], places=10)
                self.assertAlmostEqual(self.p.data.qpos[self.p.model.joint("rg_left_finger_joint").qposadr[0]],
                                       -.020999*self.p.profile["close"], places=10)
            else:
                np.testing.assert_allclose(self.p.data.qpos[6:12], self.p.profile["close"])
                # Validate loop closure independently of the URDF mimic assignment.
                for i in range(2):
                    a, b = self.p.model.eq_obj1id[i], self.p.model.eq_obj2id[i]
                    da = self.p.model.eq_data[i]
                    pa = self.p.data.xpos[a] + self.p.data.xmat[a].reshape(3,3) @ da[:3]
                    pb = self.p.data.xpos[b] + self.p.data.xmat[b].reshape(3,3) @ da[3:6]
                    np.testing.assert_allclose(pa, pb, atol=2e-6)
            self.cycle(trigger=0.)
            self.cycle(trigger=1.)
            self.assertEqual(self.p.data.qpos[self.p.gripper_joint], self.p.profile["open"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
