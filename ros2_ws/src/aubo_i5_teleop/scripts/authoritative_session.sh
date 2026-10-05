#!/usr/bin/env bash
# 权威会话拉起脚本（幂等）：无论当前什么状态，跑完即为"仿真+MoveIt+stage3(参数显式)+归位"。
# 与 vr_session.sh 的区别：本脚本 stage3 显式带 butterworth_filter_coeff:=15.0，
# 并在最后验证参数与 /clock——一切 auto_tune / VR 操控以此为前置。
set +u
cd /home/lcw/VR_teleoperation/ros2_ws/src/aubo_i5_teleop
source /opt/ros/humble/setup.bash
source /home/lcw/VR_teleoperation/ros2_ws/install/setup.bash
export DISPLAY=:0
PY=/home/lcw/tomato_robot/.venv/bin/python
BFC=${BFC:-15.0}

echo "[1/5] 杀 stage3 链与输入节点（保留 stage2a/2b 若健康）..."
for pat in "stage3_pose_trackin[g]" "servo_interfac[e]" "pose_tracking_nod[e]" \
           "clutch_mapper_nod[e]" "quest_adapter_nod[e]" "mock_vr_nod[e]" "gripper_fsm_nod[e]" \
           "lara_style_tracke[r]"; do
  for p in $(pgrep -f "$pat"); do kill -9 "$p" 2>/dev/null; done
done
# /clock 检查：仿真死了就连 stage2 一起重启
CLOCK_OK=$(timeout 6 ros2 topic hz /clock 2>/dev/null | grep -m1 average | awk '{print $3}')
if [ -z "$CLOCK_OK" ]; then
  echo "  仿真不在——重启 stage2a/2b..."
  for pat in "stage2b_movei[t]" "stage2a_mujoc[o]" "ros2_contro[l]_node" "move_grou[p]" \
             "robot_state_publishe[r]" "add_scene_floo[r]" "rviz[2]"; do
    for p in $(pgrep -f "$pat"); do kill -9 "$p" 2>/dev/null; done
  done
  sleep 2
  setsid nohup ros2 launch aubo_i5_teleop stage2a_mujoco.launch.py arm_control_mode:=position \
    mujoco_model:=/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml \
    headless:=true > /tmp/b2a.log 2>&1 &
  for _ in $(seq 1 90); do
    ros2 control list_controllers 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' \
      | awk '{print $1, $NF}' | grep -qx "forward_command_controller_position active" && break
    sleep 1
  done
  setsid nohup ros2 launch aubo_i5_teleop stage2b_moveit.launch.py > /tmp/b2b.log 2>&1 &
  for _ in $(seq 1 90); do ros2 node list 2>/dev/null | grep -q "/move_group" && break; sleep 1; done
fi
echo "  /clock: $(timeout 6 ros2 topic hz /clock 2>/dev/null | grep -m1 average | awk '{print $3}') Hz"

if [ "${ROUTE_B:-1}" = "1" ]; then
  echo "[2/5] 路线 B：跳过 stage3（pose_tracking+servo_interface 已被 lara_style_tracker 替代）"
fi
if [ "${ROUTE_B:-1}" = "0" ]; then
  setsid nohup ros2 launch aubo_i5_teleop stage3_pose_tracking.launch.py \
    output_mode:=position butterworth_filter_coeff:=$BFC > /tmp/authoritative_3.log 2>&1 &
  for _ in $(seq 1 60); do
    ros2 node list 2>/dev/null | grep -q "servo_pose_tracking" \
      && ros2 node list 2>/dev/null | grep -q "servo_interface" && break
    sleep 1
  done
  sleep 3
  echo "[3/5] 参数验证（路线 A）..."
  B=$(timeout 10 ros2 param get /servo_pose_tracking butterworth_filter_coeff 2>/dev/null | tail -1 | awk '{print $4}')
  echo "  butterworth=$B（期望 $BFC）"
  if [ "$B" != "$BFC" ] || [ -z "$B" ]; then echo "❌ 参数不符"; exit 1; fi
fi

echo "[4/5] 归位..."
timeout 120 $PY scripts/go_ready.py --mode position 2>&1 | grep -E "到位误差"

echo "[4b/5] 起 VR 执行核心 + 输入链（lara_tracker + quest_adapter + clutch_mapper + gripper FSM）..."
nohup $PY scripts/lara_style_tracker.py > /tmp/lara_tracker.log 2>&1 &
sleep 2
nohup $PY scripts/quest_adapter_node.py > /tmp/quest_adapter.log 2>&1 &
# --rot-axis-map yxz：用户手势轴表稳定互换（2026-09-29/10-05 两次 [诊断] 实测：点头→世界y、拧钥匙→世界x），
# 映射层换轴对齐（用户 2026-10-05 真机确认"完全正确"）；回滚=去旗子重启 mapper
nohup $PY scripts/clutch_mapper_node.py --input quest --no-scale-toggle --rot-axis-map yxz > /tmp/teleop_mapper.log 2>&1 &
nohup $PY scripts/gripper_fsm_node.py > /tmp/teleop_gripper.log 2>&1 &
nohup $PY scripts/controller_axes_viz.py > /tmp/controller_viz.log 2>&1 &
sleep 4

echo "[5/5] 状态..."
timeout 6 ros2 topic echo --once /target_pose 2>/dev/null | grep -m1 frame_id || echo "  /target_pose 无发布（头显未推流/未按 Grip——正常）"
echo "✅ 权威会话就绪（butterworth=$B）——可跑 auto_tune_pid.py 或接 VR 输入"
