#!/usr/bin/env bash
# VR 会话一键恢复（Quest 头显版）：清 → 起栈 → 归位 → stage3 → quest适配器+映射器+夹爪FSM → 自检。
# relay 不在本脚本管理内（头显流的前提）：bash scripts/run_relay.sh 另起。
# 键盘 Mock 版见 run_teleop.sh；把 QUEST=0 换回 mock：
#   QUEST=0 bash scripts/vr_session.sh
set +u
QUEST=${QUEST:-1}
cd /home/lcw/VR_teleoperation/ros2_ws/src/aubo_i5_teleop
source /opt/ros/humble/setup.bash
source /home/lcw/VR_teleoperation/ros2_ws/install/setup.bash
export DISPLAY=:0
PY=/home/lcw/tomato_robot/.venv/bin/python

echo "[1/6] 清杀全部..."
# ⚠️ 杀名单含本进程祖宗时会自杀——模式全部用 [x] 括号规避自匹配
for pat in "quest_adapter_nod[e]" "mock_vr_nod[e]" "clutch_mapper_nod[e]" "gripper_fsm_nod[e]" \
           "stage3_pose_trackin[g]" "stage2b_movei[t]" "stage2a_mujoc[o]" "ros2_contro[l]_node" \
           "move_grou[p]" "robot_state_publishe[r]" "add_scene_floo[r]" "rviz[2]" \
           "servo_interfac[e]" "pose_tracking_nod[e]"; do
  for p in $(pgrep -f "$pat"); do kill -9 "$p" 2>/dev/null; done
done
sleep 2

echo "[2/6] 起栈（stage2a/2b + go_ready）..."
setsid nohup ros2 launch aubo_i5_teleop stage2a_mujoco.launch.py arm_control_mode:=position \
  mujoco_model:=/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml > /tmp/vr_2a.log 2>&1 &
for _ in $(seq 1 90); do
  ros2 control list_controllers 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | awk '{print $1, $NF}' \
    | grep -qx "forward_command_controller_position active" && break
  sleep 1
done
echo "  stage2a 就绪"
setsid nohup ros2 launch aubo_i5_teleop stage2b_moveit.launch.py > /tmp/vr_2b.log 2>&1 &
for _ in $(seq 1 90); do ros2 node list 2>/dev/null | grep -q "/move_group" && break; sleep 1; done
timeout 120 $PY scripts/go_ready.py --mode position 2>&1 | grep -E "到位误差"
echo "  归位完成"

echo "[3/6] 起 stage3（等 pose_tracking 与 servo_interface 都在）..."
setsid nohup ros2 launch aubo_i5_teleop stage3_pose_tracking.launch.py output_mode:=position > /tmp/vr_3.log 2>&1 &
for _ in $(seq 1 60); do
  ros2 node list 2>/dev/null | grep -q "servo_pose_tracking" \
    && ros2 node list 2>/dev/null | grep -q "servo_interface" && break
  sleep 1
done
sleep 3
echo "  stage3 就绪"

echo "[4/6] 起输入侧（$([ $QUEST -eq 1 ] && echo 'Quest 适配器' || echo '键盘 Mock') + 映射器 + 夹爪 FSM）..."
if [ $QUEST -eq 1 ]; then
  setsid nohup $PY scripts/quest_adapter_node.py > /tmp/quest_adapter.log 2>&1 &
fi
MAPPER_INPUT=$([ $QUEST -eq 1 ] && echo quest || echo mock_vr)
setsid nohup $PY scripts/clutch_mapper_node.py --input $MAPPER_INPUT > /tmp/teleop_mapper.log 2>&1 &
setsid nohup $PY scripts/gripper_fsm_node.py  > /tmp/teleop_gripper.log 2>&1 &
if [ $QUEST -eq 0 ]; then
  sleep 3
  setsid nohup $PY scripts/mock_vr_node.py > /tmp/teleop_mock.log 2>&1 &
fi
sleep 4

echo "[5/6] 自检..."
OK=1
if [ $QUEST -eq 1 ]; then POSE=/quest/pose; JOY=/quest/joy; else POSE=/mock_vr/pose; JOY=/mock_vr/joy; fi
for t in /clock /joint_states $POSE $JOY; do
  HZ=$(timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -m1 average | awk '{print $3}')
  echo "  $t: ${HZ:-无} Hz"
  [ -z "$HZ" ] && OK=0
done
pgrep -f "vr_teleop_kit.relay.serve[r]" >/dev/null && echo "  relay: 运行中" \
  || { echo "  relay: ⚠️ 未运行——先 bash scripts/run_relay.sh（Quest 模式必需）"; [ $QUEST -eq 1 ] && OK=0; }
[ $OK -eq 1 ] && echo "✅ VR 会话就绪——戴头显按 Grip 操控" || echo "❌ 有话题缺数据，查 /tmp/vr_*.log 与 /tmp/quest_adapter.log"
