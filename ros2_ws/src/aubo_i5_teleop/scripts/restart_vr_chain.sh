#!/bin/bash
# VR 输入链重启（自带环境，幂等）：杀旧 → 起四节点 → 快照
cd /home/lcw/VR_teleoperation/ros2_ws/src/aubo_i5_teleop
source /opt/ros/humble/setup.bash
source /home/lcw/VR_teleoperation/ros2_ws/install/setup.bash
export DISPLAY=:0
PY=/home/lcw/tomato_robot/.venv/bin/python

echo "[1/3] 杀旧实例..."
$PY - <<'PYEOF'
import os, signal, subprocess
out = subprocess.run(["pgrep", "-f", "quest_adapter_nod[e]|clutch_mapper_nod[e]|gripper_fsm_nod[e]|lara_style_tracke[r]"],
                     capture_output=True, text=True).stdout.split()
for p in out:
    try:
        os.kill(int(p), signal.SIGKILL); print("killed", p)
    except Exception as e:
        print("skip", p, e)
PYEOF
sleep 2

echo "[2/3] 起四节点..."
$PY scripts/lara_style_tracker.py > /tmp/lt5.log 2>&1 &
sleep 1
$PY scripts/quest_adapter_node.py > /tmp/qa6.log 2>&1 &
sleep 1
$PY scripts/clutch_mapper_node.py --input quest > /tmp/cm6.log 2>&1 &
sleep 1
$PY scripts/gripper_fsm_node.py > /tmp/gf6.log 2>&1 &
sleep 8

echo "[3/3] 快照..."
timeout 12 $PY <<'PYEOF'
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
rclpy.init(); n = Node("chain7")
got = {"pose": 0, "joy": 0, "tgt": 0}
last_joy = None; last_pose = None
n.create_subscription(PoseStamped, "/quest/pose", lambda m: (got.__setitem__("pose", got["pose"]+1), last.__setitem__("pose", (round(m.pose.position.x,2), round(m.pose.position.y,2), round(m.pose.position.z,2)))), 10)
n.create_subscription(Joy, "/quest/joy", lambda m: (got.__setitem__("joy", got["joy"]+1), last.__setitem__("joy", list(m.buttons))), 10)
n.create_subscription(PoseStamped, "/target_pose", lambda m: got.__setitem__("tgt", got["tgt"]+1), 10)
end = time.time()+8
while time.time()<end: rclpy.spin_once(n, timeout_sec=0.02)
print("8 秒：", got)
print("joy=%s pose=%s" % (last["joy"], last["pose"]))
n.destroy_node(); rclpy.shutdown()
PYEOF
echo "✅ 完成——保持 Grip 按住并挥手，上面 pose/joy 计数应持续增长"
