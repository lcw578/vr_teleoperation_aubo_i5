"""注入 z-8cm 目标 10 秒，逐段量臂的 z 向位移（lara_tracker 全链稳定性终测）。"""
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
import mujoco

m = mujoco.MjModel.from_xml_path("/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml")
d = mujoco.MjData(m)
G = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
     "wrist1_joint", "wrist2_joint", "wrist3_joint"]
qa = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in G]
bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ag95_base")

rclpy.init()
n = Node("inj_stab")
n.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
pub = n.create_publisher(PoseStamped, "/target_pose", 10)
qq = {}
n.create_subscription(JointState, "/joint_states",
                      lambda m: qq.update({nm: (m.position[i], m.velocity[i])
                                           for i, nm in enumerate(m.name) if nm in G}), 10)


def fk():
    if len(qq) < 6:
        return None
    for a, j in zip(qa, G):
        d.qpos[a] = qq[j][0]
    mujoco.mj_forward(m, d)
    return d.xpos[bid] + d.xmat[bid].reshape(3, 3) @ np.array([-0.0405, -0.0143, 0.1492])


m2 = PoseStamped()
m2.header.frame_id = "world"
m2.pose.position.x, m2.pose.position.y, m2.pose.position.z = (-0.0008, -0.7691, 0.0722)
m2.pose.orientation.x = 0.7942
m2.pose.orientation.y = -0.1690
m2.pose.orientation.z = -0.4436
m2.pose.orientation.w = 0.3794

t0 = time.time()
marks = []
while time.time() - t0 < 10:
    m2.header.stamp = n.get_clock().now().to_msg()
    pub.publish(m2)
    rclpy.spin_once(n, timeout_sec=0.005)
    el = time.time() - t0
    if any(abs(el - k) < 0.5 for k in [2, 5, 8]) and not any(abs(mk - el) < 0.6 for mk, _ in marks):
        p = fk()
        if p is not None:
            marks.append((el, p.copy()))
    time.sleep(0.005)
p_end = fk()
for el, p in marks:
    print("  t=%4.1f  z差 %+.1f mm" % (el, (p[2] - 0.1522) * 1000))
n.destroy_node()
rclpy.shutdown()
