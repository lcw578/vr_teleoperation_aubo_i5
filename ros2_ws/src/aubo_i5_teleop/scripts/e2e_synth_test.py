import rclpy, time, math, threading
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np, mujoco

m = mujoco.MjModel.from_xml_path("/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml")
d = mujoco.MjData(m)
G = ["shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"]
qa = [m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,j)] for j in G]
bid = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,"ag95_base")

rclpy.init()
n = Node("e2e")
n.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
pub_pose = n.create_publisher(PoseStamped, "/quest/pose", 10)
pub_joy = n.create_publisher(Joy, "/quest/joy", 10)
qq = {}
n.create_subscription(JointState, "/joint_states", lambda m: qq.update({nm: (m.position[i], m.velocity[i]) for i, nm in enumerate(m.name) if nm in G}), 10)

def fk():
    if len(qq) < 6: return None
    for a, j in zip(qa, G): d.qpos[a] = qq[j][0]
    mujoco.mj_forward(m, d)
    return d.xpos[bid] + d.xmat[bid].reshape(3,3) @ np.array([-0.0405,-0.0143,0.1492])

while n.get_clock().now().nanoseconds == 0: rclpy.spin_once(n, timeout_sec=0.05)
end = time.time()+2
while time.time()<end: rclpy.spin_once(n, timeout_sec=0.02)
p_home = fk().copy()
print("臂初始位置 (%.3f,%.3f,%.3f)" % tuple(p_home))

# 模拟操作者：①按下 Grip（joy buttons[0]=1）②手柄位姿向 -y 移动 10cm（Quest 系经 R_CALIB 后=臂系？不——
# adapter 的 on_frame 里做 R_CALIB 变换，所以这里发【Quest 系原始位姿】）
# Quest 系：手柄在 (0.3, 0.9, 0.2) 附近；-y 推 10cm = y 0.9→0.8
q_quest = (0.3794, 0.7942, -0.1690, -0.4436)  # wxyz，任意自然姿态

def publish_step(step):
    # 手柄位置：step 0=基准，1=向 -y 移 10cm 后
    if step == 0:
        p_q = np.array([0.3, 0.9, 0.2])
    else:
        p_q = np.array([0.3, 0.8, 0.2])
    ps = PoseStamped(); ps.header.frame_id = "quest_world"
    ps.header.stamp = n.get_clock().now().to_msg()
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = p_q
    ps.pose.orientation.x, ps.pose.orientation.y = q_quest[1], q_quest[2]
    ps.pose.orientation.z, ps.pose.orientation.w = q_quest[3], q_quest[0]
    pub_pose.publish(ps)
    j = Joy(); j.buttons = [1, 0, 0, 0]; j.axes = [0.0, 1.0]
    pub_joy.publish(j)

# 阶段 1：Grip 按住 + 手柄基准位（保持 3 秒，让映射器接合并锚定）
t0 = time.time()
while time.time()-t0 < 3.0:
    publish_step(0)
    rclpy.spin_once(n, timeout_sec=0.005); time.sleep(0.004)
p_a = fk().copy()
print("阶段1（接合+锚定）后臂位 (%.3f,%.3f,%.3f)" % tuple(p_a))

# 阶段 2：手柄向 Quest -y 推 10cm（保持 Grip）
t0 = time.time()
while time.time()-t0 < 5.0:
    publish_step(1)
    rclpy.spin_once(n, timeout_sec=0.005); time.sleep(0.004)
p_b = fk().copy()
print("阶段2（手柄 -y 推 10cm）后臂位 (%.3f,%.3f,%.3f)" % tuple(p_b))
print("臂位移：(%+.1f, %+.1f, %+.1f) mm，合计 %.1f mm" % (
    (p_b-p_a)[0]*1000, (p_b-p_a)[1]*1000, (p_b-p_a)[2]*1000, np.linalg.norm(p_b-p_a)*1000))

# 阶段 3：松开 Grip（臂应冻结）
t0 = time.time()
frozen = p_b.copy()
while time.time()-t0 < 2.0:
    publish_step(1)  # 手柄位置不变但 Grip 松开——joy buttons[0]=0
    # 重发 joy：grip=0
    j = Joy(); j.buttons = [0,0,0,0]; j.axes=[0.0,0.0]
    pub_joy.publish(j)
    rclpy.spin_once(n, timeout_sec=0.005); time.sleep(0.004)
p_c = fk().copy()
print("阶段3（松开 Grip）臂漂移 %.2f mm" % (np.linalg.norm(p_c-p_b)*1000))
print()
print("═══ 判定：若臂合计位移 >50mm 且方向为 -y（向任务区）→ 端到端通 ═══")
n.destroy_node(); rclpy.shutdown()
