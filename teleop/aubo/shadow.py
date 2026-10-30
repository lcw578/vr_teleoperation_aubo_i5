"""Author Quest core + measured joint mapping + fake-only session coordination.

No native SDK import, network connection, automatic home or gripper writes.
Candidates are inspectable; dispatch_fake is explicitly restricted to the fake
backend. This is a tested integration rehearsal, not a real hardware follower.
"""
import math

import numpy as np

from .model import AuboModel
from .feedback import joints
from .offline_session import FakePositionBackend, OfflinePositionSession


class AuboShadowCoordinator:
    def __init__(self, policy, *, control_dt, clock, rotation_mode="world", geometry=None, rest_q=None, quest_to_base=None,
                 core_input_timeout_s=None):
        if not math.isfinite(control_dt) or control_dt <= 0:
            raise ValueError("Explicit positive control period required")
        self.geometry = geometry if geometry is not None else AuboModel()
        from teleop.mujoco_session import QUEST_TO_BASE
        from vr_teleop_kit.core.quest_teleop import QuestTeleoperator, QuestTeleoperatorConfig
        mapping = np.asarray(QUEST_TO_BASE if quest_to_base is None else quest_to_base,float)
        if (mapping.shape!=(3,3) or not np.all(np.isfinite(mapping)) or
                not np.allclose(mapping.T@mapping,np.eye(3),atol=1e-6) or
                not np.isclose(np.linalg.det(mapping),1.,atol=1e-6)):
            raise ValueError("Quest to base mapping must be a proper rotation")
        caps = (np.asarray(policy.velocity_rad_s) * control_dt).tolist()
        cfg = QuestTeleoperatorConfig(
            id="aubo-shadow", active_hands=("right",), rotation_mode=rotation_mode,
            head_yaw_alignment=False, binary_gripper=True, r_calib=mapping.tolist(),
            publish_ik_state=False, force_haptic_enabled=False,
            rest_qpos_right=list(self.geometry.profile["q_rest"] if rest_q is None else rest_q),
            velocity_limits=list(policy.velocity_rad_s), control_dt=control_dt,
            max_dq_per_joint=caps, max_dq_per_joint_scalar_pos=caps[0],
            max_dq_per_joint_scalar_rot=caps[3],
            **(dict(xr_frame_stale_timeout_s=core_input_timeout_s) if core_input_timeout_s is not None else {}))
        self.core = QuestTeleoperator(cfg, models={"right": self.geometry.model})
        self.core.state_source = "offline_shadow"
        self.core._arms["right"]["solver"].joint_limits[:] = np.column_stack(
            (policy.joint_min_rad, policy.joint_max_rad))
        self.core.mode_switch_allowed = False
        self.session = OfflinePositionSession(FakePositionBackend(), policy, clock=clock)
        self.clock = clock
        self.input_time = None
        self.grip = False
        self.last_candidate = None
        self.closed = False
        self.last_input_event = None
        self.last_input_interruption = None
        self.core.reset_input("initial_release_required")

    def feedback(self, q_rad, *, received_monotonic_s, source="fake_only"):
        if self.closed:
            raise RuntimeError("Shadow session is closed; construct a new session")
        try:
            sample = self.session.cache.update(q_rad, source=source,
                received_monotonic_s=received_monotonic_s)
            if not sample.valid:
                self._revoke(sample.reason)
        except (TypeError, ValueError, OverflowError):
            self._revoke("invalid_feedback")

    def observe_readonly(self, follower):
        """Use cached read-only observations for candidate-only inspection.

        The caller performs queries separately; no blocking SDK call in tick.
        Fake dispatch remains fake, regardless of the observation source.
        """
        from .readonly import ReadOnlyFollower
        if type(follower) is not ReadOnlyFollower:
            raise TypeError("Only the motion-disabled read-only follower is accepted")
        try:
            observation = follower.get_observation(max_age_s=self.session.policy.feedback_max_age_s)
            self.feedback(observation["joint_positions_rad"],
                received_monotonic_s=observation["host_received_monotonic_s"],
                source="aubo_readonly:" + observation["source"])
        except (RuntimeError, ValueError, TypeError):
            self._revoke("readonly_feedback_unavailable")
            return False
        return self.session.cache.latest().valid

    def _revoke(self, reason):
        if self.session.engaged:
            arm = self.core._arms["right"]
            self.last_input_interruption = dict(reason=str(reason),
                host_monotonic_s=self.clock(), grip=self.grip,
                input_age_s=None if self.input_time is None else self.clock()-self.input_time,
                core_paused=arm["paused"], core_engaged=arm["engaged"],
                core_ramp_active=arm["ramp_active"], last_input_event=self.last_input_event)
        self.session.revoke(reason)
        self.core.reset_input(reason)
        self.input_time = None
        self.last_candidate = None

    def reject_input(self, reason="invalid_xr_frame"):
        """Revoke the current session from an input/transport owner.

        The offline runtime uses this instead of reaching into private state.
        It does not interpret the reason as a hardware stop confirmation.
        """
        if self.closed:
            return False
        self._revoke(str(reason))
        return False

    def acknowledge_clutch_stop(self):
        """Record a normal release after the hardware owner confirms rest.

        Only the hardware runtime may use this for a confirmed, fault-free
        clutch stop. The validated release is a lifecycle event, not a pose
        to replay after a potentially long stop. Reset all old hand anchors;
        the next fresh XR frame and measured joints must establish new ones.
        """
        if self.closed:
            return
        self._revoke("clutch_stop_confirmed")
        self.grip = False
        self.session.require_release = False
        self.session.fault = None
        with self.core._lock:
            self.core._arms["right"].update(require_release=False, last_grip=False)

    def watchdog_tick(self):
        """Run the session checks without advancing IK or sending a target."""
        if self.closed or not self.session.engaged:
            return self.session.fault is None
        if self.session.check():
            return True
        reason = self.session.fault or "watchdog_fault"
        self.core.reset_input(reason)
        self.input_time = None
        self.last_candidate = None
        return False

    def accept_xr_frame(self, frame, *, received_monotonic_s=None):
        # Validate before author input caching: an invalid hand must not leave
        # an older valid hand active until its timeout. No buttons default to
        # a valid release in this hardware integration rehearsal.
        if self.closed:
            return False
        self.last_input_event = dict(processed_monotonic_s=self.clock(), accepted=False,
                                     reason="input_validation_failed")
        receive_time = self.clock() if received_monotonic_s is None else received_monotonic_s
        try:
            receive_time = float(receive_time)
        except (TypeError, ValueError, OverflowError):
            return self.reject_input("invalid_input_reception_timestamp")
        if (not math.isfinite(receive_time) or receive_time > self.clock() or
                self.clock() - receive_time > self.session.policy.input_max_age_s):
            return self.reject_input("input_invalid_or_expired")
        if (self.session.engaged and self.input_time is not None and
                self.clock() - self.input_time > self.session.policy.input_max_age_s):
            self._revoke("input_timeout")
            return False
        try:
            normalized = normalize_xr_frame(frame)
            grip = normalized["controllers"]["right"]["buttons"][1]["p"]
            prior_epoch = self.core._input_epoch
            prior_stats = dict(self.core.input_stats)
            accepted = self.core.accept_xr_frame(normalized)
            now = self.clock()
            self.last_input_event = dict(processed_monotonic_s=now,
                received_monotonic_s=receive_time, host_wait_s=now-receive_time,
                seq=normalized["seq"], epoch=normalized["epoch"],
                t_client_ms=normalized["t_client"], grip=grip, accepted=accepted,
                reason=("accepted" if accepted else
                        "source_delay_exceeded" if self.core.input_stats["queued"] > prior_stats["queued"] else
                        "source_or_sequence_out_of_order" if self.core.input_stats["out_of_order"] > prior_stats["out_of_order"] else
                        "core_rejected"),
                relative_source_delay_s=(now-normalized["t_client"]/1000.-self.core._source_offset
                    if self.core._source_offset is not None else None))
        except (KeyError, TypeError, ValueError, OverflowError):
            self._revoke("invalid_xr_frame")
            return False
        if not accepted:
            return False
        if prior_epoch is not None and frame.get("epoch") != prior_epoch:
            self._revoke("reference_changed")
            # The current reset frame cannot count as the recovery release.
            return False
        self.grip, self.input_time = grip, receive_time
        return True

    def tick(self, *, joint_target=None):
        try:
            return self._tick(joint_target=joint_target)
        except Exception:
            self._revoke("core_exception")
            raise

    def _tick(self, *, joint_target=None):
        self.last_candidate = None
        if self.closed or self.input_time is None:
            return None
        try:
            sample = self.session.require_feedback()
        except RuntimeError as exc:
            self._revoke(str(exc))
            return None
        self.core.observe("right", self.geometry.qpos(sample.q_rad))
        was_engaged = self.session.engaged
        self.session.input(grip=self.grip, received_monotonic_s=self.input_time, tracking_valid=True)
        if was_engaged and not self.session.engaged and self.session.fault:
            self.core.reset_input(self.session.fault)
        # Evaluate released frames as well, so reset_input's release gate clears.
        action = self.core.get_action() if joint_target is None else None
        if not self.session.engaged:
            return None
        arm = self.core._arms["right"]
        if joint_target is None and (arm["paused"] or not arm["engaged"] or arm["ramp_active"]):
            self._revoke("core_input_unavailable")
            return None
        q = (joints(joint_target) if joint_target is not None else
             [action[f"right_joint_{i}.pos"] for i in range(1, 7)])
        if self.session.offer(q, generated_monotonic_s=self.clock(), session=self.session.session):
            self.last_candidate = self.session.pending
        elif self.session.fault:
            self.core.reset_input(self.session.fault)
            self.input_time = None
        return self.last_candidate

    def dispatch_fake(self):
        if self.closed:
            return False
        result = self.session.dispatch()
        if self.session.fault:
            self.core.reset_input(self.session.fault)
            self.input_time = None
            self.last_candidate = None
        return result

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.session.disconnect()
        self.core.reset_input("closed")
        self.input_time = None
        self.last_candidate = None


def normalize_xr_frame(frame):
    """Validate every transport event before latest-only caching; no core mutation."""
    if not isinstance(frame, dict):
        raise ValueError("XR frame must be an object")
    if (frame.get("type") != "xr_frame" or not isinstance(frame.get("epoch"), str) or not frame["epoch"] or
            type(frame.get("seq")) is not int or frame["seq"] < 0 or
            not math.isfinite(float(frame["t_client"]))):
        raise ValueError("epoch, sequence and source time required")
    hand = frame["controllers"]["right"]
    p, q = np.asarray(hand["position"], float), np.asarray(hand["orientation"], float)
    buttons = [dict(b) for b in hand["buttons"]]
    norm = np.linalg.norm(q)
    if (hand.get("tracking_valid", True) is not True or p.shape != (3,) or q.shape != (4,) or
            not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)) or
            not np.isfinite(norm) or norm < 1e-8 or len(buttons) < 2):
        raise ValueError("invalid right controller")
    for b in buttons:
        v = float(b["v"])
        if type(b.get("p")) is not bool or not math.isfinite(v) or not 0. <= v <= 1.:
            raise ValueError("invalid button")
    if len(buttons) > 3:
        buttons[3] = dict(buttons[3], p=False, v=0.)
    return dict(frame, t_client=float(frame["t_client"]), controllers={"right":
        dict(hand, position=p.tolist(), orientation=(q/norm).tolist(), buttons=buttons)})
