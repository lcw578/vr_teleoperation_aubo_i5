"""Explicit WebXR/ROS/tool conventions; no ROS dependency."""
import numpy as np
import mujoco
from clutch_pose_mapper import quat_xyzw_to_wxyz, quat_mul

QUEST_TO_BASE = np.array([[-1., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
# WebXR grip frame: +X right, +Y up, -Z forward. Tool: +X right,
# -Y up, +Z approach. Rx(pi) preserves handedness (det=+1).
QUEST_LOCAL_TO_TOOL = np.diag([1., -1., -1.])

def quest_pose(position, orientation_xyzw):
    p = np.asarray(position, float)
    q = quat_xyzw_to_wxyz(orientation_xyzw)
    if p.shape != (3,) or q.shape != (4,) or not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)):
        raise ValueError("invalid Quest pose")
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        raise ValueError("zero quaternion")
    q /= norm
    calibration = np.zeros(4)
    mujoco.mju_mat2Quat(calibration, QUEST_TO_BASE.ravel())
    return QUEST_TO_BASE @ p, quat_mul(calibration, q), q

def tool_pose(model, data, body_id, profile):
    R_body = data.xmat[body_id].reshape(3, 3).copy()
    p = data.xpos[body_id] + R_body @ np.asarray(profile["tip_offset"])
    R_offset = np.zeros(9)
    mujoco.mju_quat2Mat(R_offset, np.asarray(profile["tool_quat"], float))
    return p, R_body @ R_offset.reshape(3, 3)
