#!/bin/bash
# VR Mock 会话一键恢复：清 → 起栈 → 归位 → 起 stage3 → 起三节点 → 自检
cd /home/lcw/VR_teleoperation/ros2_ws/src/aubo_i5_teleop
source /opt/ros/humble/setup.bash
source /home/lcw/VR_teleoperation/ros2_ws/install/setup.bash
export DISPLAY=:0
PY=/home/lcw/tomato_robot/.venv/bin/python

echo "[1/5] 清杀全部..."
bash /tmp/clean_all.sh
sleep 2

echo "[2/5] 起栈（stage2a/2b + go_ready + stage3）..."
setsid nohup ros2 launch aubo_i5_teleop stage2a_mujoco.launch.py arm_control_mode:=position \
  mujoco_model:=/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml > /tmp/vr_2a.log 2>&1 &
for _ in $(seq 1 90); do
  ros2 control list_controllers 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | awk '{print $1, $NF}' | grep -qx "forward_command_controller_position active" && break
  sleep 1
done
echo "  stage2a 就绪"
setsid nohup ros2 launch aubo_i5_teleop stage2b_moveit.launch.py > /tmp/vr_2b.log 2>&1 &
for _ in $(seq 1 90); do ros2 node list 2>/dev/null | grep -q "/move_group" && break; sleep 1; done
timeout 120 $PY scripts/go_ready.py --mode position 2>&1 | grep -E "到位误差" 
echo "  归位完成"

echo "[3/5] 起 stage3..."
setsid nohup ros2 launch aubo_i5_teleop stage3_pose_tracking.launch.py output_mode:=position > /tmp/vr_3.log 2>&1 &
for _ in $(seq 1 60); do
  ros2 node list 2>/dev/null | grep -q "servo_pose_tracking" && break
  sleep 1
done
sleep 3
echo "  stage3 就绪"

echo "[4/5] 起映射器 + 夹爪 FSM + 键盘 Mock..."
setsid nohup $PY scripts/clutch_mapper_node.py > /tmp/teleop_mapper.log 2>&1 &
setsid nohup $PY scripts/gripper_fsm_node.py  > /tmp/teleop_gripper.log 2>&1 &
sleep 3
setsid nohup $PY scripts/mock_vr_node.py > /tmp/teleop_mock.log 2>&1 &
sleep 3

echo "[5/5] 自检..."
OK=1
for t in /clock /joint_states /mock_vr/pose /mock_vr/joy; do
  HZ=$(timeout 6 ros2 topic hz $t 2>/dev/null | head -1 | awk '{print $2}')
  echo "  $t: ${HZ:-无} Hz"
  [ -z "$HZ" ] && OK=0
done
[ $OK -eq 1 ] && echo "✅ 会话就绪——键盘操控可用" || echo "❌ 有话题无数据，看 /tmp/vr_*.log 与 /tmp/teleop_*.log"
