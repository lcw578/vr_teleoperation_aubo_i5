"""Named SDK joint mapping and explicit frame comparisons; no SDK connection."""
import json
from pathlib import Path

import mujoco
import numpy as np

from .feedback import joints

ARM_JOINTS = ("shoulder_joint", "upperArm_joint", "foreArm_joint",
              "wrist1_joint", "wrist2_joint", "wrist3_joint")


class AuboModel:
    def __init__(self, gripper="rg", *, tool_frame="gripper"):
        # Reuse the accepted model/geometry without starting a simulation or relay.
        from teleop.mujoco_session import ROOT, load_model
        if tool_frame not in ("gripper", "flange"):
            raise ValueError("Explicit tool frame must be gripper or flange")
        self.tool_frame = tool_frame
        if tool_frame == "flange":
            self.scene = ROOT / "assets/aubo_i5/scene_aubo_i5_rg.xml"
            spec = mujoco.MjSpec.from_file(str(self.scene))
            spec.delete(spec.body("rg_base"))
            spec.body("wrist3_Link").add_site(name="tool0", size=[.008, 0, 0], rgba=[0, 1, 0, 1])
            spec.body("foreArm_Link").add_site(name="j4_anchor", pos=spec.body("wrist1_Link").pos)
            spec.worldbody.add_light(pos=[0, 0, 2], dir=[0, 0, -1])
            self.model = spec.compile()
            self.data = mujoco.MjData(self.model)
            self.profile = dict(name="flange", q_rest=[0.] * 6, tip_offset=[0., 0., 0.],
                                tool_quat=[1., 0., 0., 0.])
        else:
            self.model, self.data, self.profile, self.scene = load_model(gripper)
        self.addresses = tuple(int(self.model.joint(name).qposadr[0]) for name in ARM_JOINTS)
        if self.addresses != tuple(range(6)):
            raise ValueError("Author IK requires the named arm joints in the first six slots")
        self.template = self.data.qpos.copy()

    def qpos(self, sdk_q_rad):
        q = self.template.copy()
        q[list(self.addresses)] = joints(sdk_q_rad)
        return q

    def pose(self, sdk_q_rad, frame):
        self.data.qpos[:] = self.qpos(sdk_q_rad)
        mujoco.mj_forward(self.model, self.data)
        if frame == "tool0":
            item = self.data.site("tool0")
        elif frame == "wrist3_Link":
            item = self.data.body("wrist3_Link")
        else:
            raise ValueError("Explicit model frame must be tool0 or wrist3_Link")
        transform = np.eye(4)
        transform[:3, :3] = item.xmat.reshape(3, 3)
        transform[:3, 3] = item.xpos
        return transform

    def metadata(self):
        return {"scene": str(self.scene), "gripper": self.profile["name"],
                "sdk_joint_order_assumption": list(ARM_JOINTS),
                "joint_order_direction_zero_verified_on_device": False,
                "tool_frame": self.tool_frame,
                "non_arm_qpos_source": "model_display_template_not_measurement"}


def rigid_transform(value, name):
    T = np.asarray(value, float)
    if (T.shape != (4, 4) or not np.all(np.isfinite(T)) or
            not np.allclose(T[3], [0, 0, 0, 1], atol=1e-10) or
            not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-8) or
            not np.isclose(np.linalg.det(T[:3, :3]), 1., atol=1e-8)):
        raise ValueError(f"{name} must be a finite rigid 4x4 transform")
    return T


def sdk_transform(data):
    pos = np.asarray(data["position_m"], float)
    quat = np.asarray(data["quaternion_wxyz"], float)
    if (pos.shape != (3,) or quat.shape != (4,) or not np.all(np.isfinite(pos)) or
            not np.all(np.isfinite(quat)) or not np.isclose(np.linalg.norm(quat), 1., atol=1e-3)):
        raise ValueError("SDK pose must have finite position and a unit wxyz quaternion")
    out = np.zeros(9)
    mujoco.mju_quat2Mat(out, quat / np.linalg.norm(quat))
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = out.reshape(3, 3), pos
    return T


def compare_fk(sdk_q_rad, sdk_pose, alignment, *, gripper="rg"):
    """T_sdk_base_sdk_pose = A @ T_model_base_frame @ B.

    A and B and their provenance must be supplied. This never guesses a TCP,
    changes device configuration, or declares hardware acceptance from a residual.
    """
    if not isinstance(alignment.get("provenance"), str) or not alignment["provenance"].strip():
        raise ValueError("Frame alignment requires a provenance description")
    A = rigid_transform(alignment["sdk_base_from_model_base"], "base alignment")
    B = rigid_transform(alignment["model_frame_from_sdk_pose_frame"], "pose alignment")
    model = AuboModel(gripper)
    predicted = A @ model.pose(sdk_q_rad, alignment["model_frame"]) @ B
    measured = sdk_transform(sdk_pose)
    relative = predicted[:3, :3].T @ measured[:3, :3]
    angle = np.arccos(np.clip((np.trace(relative) - 1.) / 2., -1., 1.))
    return {"model": model.metadata(), "alignment": alignment,
            "input_joint_positions_rad": list(joints(sdk_q_rad)),
            "predicted_sdk_pose_transform": predicted.tolist(),
            "reported_sdk_pose_transform": measured.tolist(),
            "position_residual_m": float(np.linalg.norm(predicted[:3, 3] - measured[:3, 3])),
            "orientation_residual_rad": float(angle),
            "hardware_acceptance": "not_determined_single_sample_no_threshold"}


def compare_report(report_path, alignment_path, *, gripper="rg"):
    report = json.loads(Path(report_path).read_text())
    records = [r for r in report["queries"] if r.get("operation") == "fk" and
               r.get("return_code") == 0 and "validation_error" not in r]
    if len(records) != 1:
        raise ValueError("Report requires exactly one successful FK with its input joints")
    fk = records[0]
    return compare_fk(fk["input_joint_positions_rad"], fk["data"],
                      json.loads(Path(alignment_path).read_text()), gripper=gripper)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Offline explicit-frame FK comparison; no SDK loading")
    parser.add_argument("--report", required=True, help="saved read-only report")
    parser.add_argument("--alignment", required=True, help="verified/explicit frame transform JSON")
    parser.add_argument("--gripper", choices=("rg", "ag95"), default="rg")
    args = parser.parse_args()
    print(json.dumps(compare_report(args.report, args.alignment, gripper=args.gripper),
                     ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
