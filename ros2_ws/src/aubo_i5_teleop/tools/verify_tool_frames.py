#!/usr/bin/env python3
"""Compare independent URDF FK with MuJoCo TCP pose for RG and AG95.

Run after building and sourcing the workspace. Does not create ROS nodes,
connect to hardware or send commands. Uses the default models for both profiles.
"""
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from gripper_profile import PROFILES
from teleop_geometry import tool_pose


def urdf_fk(joints, positions):
    transforms = {"world": np.eye(4)}
    pending = list(joints)
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in transforms:
                continue
            origin = joint.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
            local = np.eye(4)
            local[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
            local[:3, 3] = xyz
            if joint.get("type") in ("revolute", "continuous"):
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                local[:3, :3] = local[:3, :3] @ Rotation.from_rotvec(axis * positions[joint.get("name")]).as_matrix()
            elif joint.get("type") == "prismatic":
                local[:3, 3] += local[:3, :3] @ (np.fromstring(joint.find("axis").get("xyz"), sep=" ") * positions.get(joint.get("name"), 0.0))
            transforms[child] = transforms[parent] @ local
            pending.remove(joint)
            progress = True
        if not progress:
            raise RuntimeError("URDF has a disconnected chain")
    return transforms


def verify(name, count=30):
    profile = PROFILES[name]
    output = subprocess.check_output(["xacro", str(PACKAGE / "urdf/aubo_i5_teleop.urdf.xacro"), f"gripper:={name}"])
    joints = ET.fromstring(output).findall("joint")
    model = mujoco.MjModel.from_xml_path(profile["mjcf"])
    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, profile["ee_body"])
    rng = np.random.default_rng(22)
    worst_position = worst_rotation = 0.0
    for _ in range(count):
        positions = {j.get("name"): float(rng.uniform(-1, 1)) for j in joints if j.get("type") in ("revolute", "continuous")}
        expected = urdf_fk(joints, positions)[profile["tip_link"]]
        for name_q, value in positions.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name_q)
            if jid >= 0:
                data.qpos[model.jnt_qposadr[jid]] = value
        mujoco.mj_forward(model, data)
        p, rotation = tool_pose(model, data, body, profile)
        worst_position = max(worst_position, np.linalg.norm(p - expected[:3, 3]))
        worst_rotation = max(worst_rotation, np.linalg.norm(rotation - expected[:3, :3]))
    print(f"{name}: {count} poses; max position error={worst_position:.3g} m; rotation matrix error={worst_rotation:.3g}")
    if worst_position > 1e-6 or worst_rotation > 1e-6:
        raise AssertionError(f"{name} URDF / MuJoCo tool frame mismatch")


if __name__ == "__main__":
    for gripper in ("rg", "ag95"):
        verify(gripper)
