"""Real author core + AUBO geometry, with fake feedback/dispatch only."""
import json
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mujoco
import numpy as np
from teleop.aubo.model import AuboModel, compare_fk, rigid_transform
from teleop.aubo.offline_session import SessionPolicy
from teleop.aubo.shadow import AuboShadowCoordinator
from teleop.aubo.feedback import FeedbackCache
from teleop.aubo.readonly import ReadOnlyFollower


class Clock:
    now = 10.
    def __call__(self): return self.now


def pose_data(T):
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.ascontiguousarray(T[:3, :3]).ravel())
    return {"position_m": T[:3, 3].tolist(), "quaternion_wxyz": quat.tolist()}


class ModelTests(unittest.TestCase):
    def test_six_named_joints_leave_non_arm_qpos_as_display_only(self):
        geometry = AuboModel()
        q = [.1, -.2, .3, -.4, .5, -.6]
        full = geometry.qpos(q)
        np.testing.assert_array_equal(full[:6], q)
        np.testing.assert_array_equal(full[6:], geometry.template[6:])
        self.assertGreater(len(full), 6)
        self.assertFalse(geometry.metadata()["joint_order_direction_zero_verified_on_device"])
        full[0] = 9.
        self.assertNotEqual(geometry.template[0], 9.)

    def test_fk_with_explicit_nonidentity_base_and_tool_transforms(self):
        geometry = AuboModel()
        q = [.2, -.4, 1.1, 1., .7, -.3]
        A = np.eye(4); A[:3, :3] = [[0,-1,0],[1,0,0],[0,0,1]]; A[:3,3] = [.2,-.3,.4]
        B = np.eye(4); B[:3,3] = [.01,.02,.03]
        config = {"model_frame":"wrist3_Link", "sdk_base_from_model_base":A.tolist(),
                  "model_frame_from_sdk_pose_frame":B.tolist(), "provenance":"synthetic test; not device calibration"}
        expected = A @ geometry.pose(q, "wrist3_Link") @ B
        result = compare_fk(q, pose_data(expected), config)
        self.assertLess(result["position_residual_m"], 1e-12)
        self.assertLess(result["orientation_residual_rad"], 1e-7)
        json.dumps(result, allow_nan=False)
        perturbed = expected.copy(); perturbed[0,3] += .01
        result = compare_fk(q, pose_data(perturbed), config)
        self.assertAlmostEqual(result["position_residual_m"], .01)

    def test_fk_refuses_unverified_implicit_alignment_and_bad_rotations(self):
        for bad in (np.zeros((4,4)), np.diag([-1,1,1,1]), [[float("nan")]*4]*4):
            with self.assertRaises(ValueError): rigid_transform(bad, "test")
        with self.assertRaises(ValueError): compare_fk([0]*6, {}, {})
        with self.assertRaises(ValueError): AuboModel().qpos([0]*5)


class ShadowTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        # Explicit synthetic values, not commissioned hardware parameters.
        self.policy = SessionPolicy((-2.94,)*6, (2.94,)*6, (.5,)*6, .1, .1, .05)
        self.shadow = AuboShadowCoordinator(self.policy, control_dt=.005, clock=self.clock)
        self.clock_patch = patch("vr_teleop_kit.core.quest_teleop.time.monotonic", self.clock)
        self.clock_patch.start(); self.addCleanup(self.clock_patch.stop)
        self.q = self.shadow.geometry.template[:6].copy()
        self.seq = 0
        self.feedback()

    def feedback(self):
        self.shadow.feedback(self.q, received_monotonic_s=self.clock())

    def frame(self, grip=False, *, rest=False, position=(0,0,0), epoch="test"):
        buttons = [dict(p=False, v=0.) for _ in range(6)]
        buttons[1] = dict(p=grip, v=float(grip))
        buttons[3] = dict(p=rest, v=float(rest))
        frame = dict(type="xr_frame", epoch=epoch, seq=self.seq, t_client=self.clock()*1000,
            controllers={"right":dict(position=list(position), orientation=[0,0,0,1], buttons=buttons)})
        self.seq += 1
        return frame

    def cycle(self, grip=False, **kwargs):
        self.clock.now += .005
        self.feedback()
        self.assertTrue(self.shadow.accept_xr_frame(self.frame(grip, **kwargs)))
        return self.shadow.tick()

    def engage(self):
        self.cycle(False)
        candidate = self.cycle(True)
        self.assertIsNotNone(candidate)
        return candidate

    def test_first_grip_does_not_engage_and_no_auto_home_or_gripper_submission(self):
        self.assertIsNone(self.cycle(True, rest=True))
        self.assertFalse(self.shadow.session.engaged)
        self.assertEqual(self.shadow.session.backend.submitted, [])
        self.assertFalse(self.shadow.core._arms["right"]["ramp_active"])

    def test_actual_measured_anchor_overrides_old_internal_command(self):
        arm = self.shadow.core._arms["right"]
        arm["qpos"][:6] += .1
        self.engage()
        position, _ = arm["solver"].fk(self.shadow.geometry.qpos(self.q))
        np.testing.assert_allclose(arm["mapper"]._ee_engage_pos, position, atol=1e-12)
        np.testing.assert_array_equal(self.shadow.session.last_reference, self.q)
        self.assertEqual(self.shadow.session.backend.submitted, [])

    def test_fake_dispatch_never_synthesizes_feedback(self):
        self.engage()
        self.clock.now += .005
        self.assertTrue(self.shadow.dispatch_fake())
        np.testing.assert_array_equal(self.shadow.session.cache.latest().q_rad, self.q)
        self.assertEqual(len(self.shadow.session.backend.submitted), 1)

    def test_normal_grip_release_reengages_from_new_measured_pose(self):
        self.engage()
        self.assertIsNone(self.cycle(False))
        self.q[0] += .01
        self.assertIsNotNone(self.cycle(True))
        np.testing.assert_array_equal(self.shadow.session.last_reference, self.q)

    def test_missing_frame_identity_cannot_count_as_release(self):
        frame = self.frame(False); del frame["seq"]
        self.assertFalse(self.shadow.accept_xr_frame(frame))
        self.assertTrue(self.shadow.session.require_release)

    def test_non_object_frame_revokes_before_error_can_escape(self):
        self.engage()
        self.assertFalse(self.shadow.accept_xr_frame(None))
        self.assertFalse(self.shadow.session.engaged)
        self.assertIsNone(self.shadow.session.pending)

    def test_home_is_masked_and_position_candidate_respects_limits(self):
        self.engage()
        for _ in range(5):
            candidate = self.cycle(True, rest=True, position=(0,.01,0))
            self.assertIsNotNone(candidate)
            self.assertFalse(self.shadow.core._arms["right"]["ramp_active"])
            self.clock.now += .001
            self.assertTrue(self.shadow.dispatch_fake())

    def test_timeout_locks_recovery_until_new_release_and_press(self):
        self.engage(); self.clock.now += .11
        self.assertIsNone(self.shadow.tick())
        self.feedback()
        self.assertIsNone(self.cycle(True))
        self.assertFalse(self.shadow.session.engaged)
        self.cycle(False)
        self.assertIsNotNone(self.cycle(True))

    def test_fresh_input_after_unobserved_gap_cannot_hide_timeout(self):
        self.engage(); self.clock.now += .11; self.feedback()
        self.assertFalse(self.shadow.accept_xr_frame(self.frame(True)))
        self.assertEqual(self.shadow.session.fault, "input_timeout")
        self.assertFalse(self.shadow.dispatch_fake())
        self.assertIsNone(self.cycle(True))
        self.cycle(False); self.assertIsNotNone(self.cycle(True))

    def test_invalid_hand_immediately_revokes_pending_and_feedback_failure_blocks(self):
        self.engage()
        invalid = self.frame(True); invalid["controllers"] = {}
        self.assertFalse(self.shadow.accept_xr_frame(invalid))
        self.assertIsNone(self.shadow.session.pending)
        self.assertFalse(self.shadow.dispatch_fake())
        self.cycle(False); self.cycle(True)
        self.shadow.feedback([float("nan")]*6, received_monotonic_s=self.clock())
        self.assertFalse(self.shadow.session.engaged)
        self.assertIsNone(self.shadow.session.pending)

    def test_reference_change_cannot_count_as_recovery_release(self):
        self.engage()
        self.assertFalse(self.shadow.accept_xr_frame(self.frame(False, epoch="new")))
        self.assertTrue(self.shadow.session.require_release)
        self.assertIsNone(self.cycle(True, epoch="new"))
        self.cycle(False, epoch="new")
        self.assertIsNotNone(self.cycle(True, epoch="new"))

    def test_core_exception_revokes_before_propagating(self):
        self.engage()
        with patch.object(self.shadow.core, "get_action", side_effect=RuntimeError("injected IK failure")):
            with self.assertRaises(RuntimeError): self.shadow.tick()
        self.assertIsNone(self.shadow.session.pending)
        self.assertFalse(self.shadow.session.engaged)
        self.assertEqual(self.shadow.session.fault, "core_exception")

    def test_closed_session_cannot_rearm(self):
        self.engage(); self.shadow.close(); self.shadow.close()
        self.assertFalse(self.shadow.accept_xr_frame(self.frame(False)))
        self.assertIsNone(self.shadow.tick())
        self.assertFalse(self.shadow.dispatch_fake())
        with self.assertRaises(RuntimeError): self.feedback()

    def test_readonly_observation_transfer_keeps_source_and_blocks_expired_feedback(self):
        follower = ReadOnlyFollower(object(), FeedbackCache(self.clock))
        follower.connected = True  # Fake-only stand-in, no native client.
        follower.cache.update(self.q, source="joint_angles")
        self.assertTrue(self.shadow.observe_readonly(follower))
        self.assertEqual(self.shadow.session.cache.latest().source, "aubo_readonly:joint_angles")
        self.engage(); self.clock.now += .11
        self.assertFalse(self.shadow.observe_readonly(follower))
        self.assertFalse(self.shadow.session.engaged)
        self.assertIsNone(self.shadow.session.pending)


if __name__ == "__main__":
    unittest.main(verbosity=2)
