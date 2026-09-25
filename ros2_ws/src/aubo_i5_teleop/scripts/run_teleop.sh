#!/usr/bin/env bash
# 键盘遥操作一键启动：映射器 + 夹爪 FSM + 键盘 Mock（栈需已在跑）。
#
# 用法：
#   bash scripts/run_teleop.sh            # 前台跑键盘（Ctrl+C 退出全部）
#   bash scripts/run_bench.sh bringup     # ← 先起栈（另一个终端或先跑这个）
#
# 键位（按住即动，松开即停）：
#   WASD+QE 平移（世界系）  UO/IK/JL 旋转（工具自系）
#   空格=离合（按住才动！）  Shift=1:1↔1:5 微调  Z=夹爪开/闭
#
# ⚠️ 与 follow_bench.py 互斥（都发 /target_pose）。
set +u
WS=/home/lcw/VR_teleoperation/ros2_ws
PKG=$WS/src/aubo_i5_teleop
PY=/home/lcw/tomato_robot/.venv/bin/python
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
cd "$PKG" || exit 1

cleanup() {
  echo ""
  echo "收摊..."
  for pat in "mock_vr_nod[e]" "clutch_mapper_nod[e]" "gripper_fsm_nod[e]"; do
    for p in $(pgrep -f "$pat"); do kill -TERM "$p" 2>/dev/null; done
  done
  sleep 1
  for pat in "mock_vr_nod[e]" "clutch_mapper_nod[e]" "gripper_fsm_nod[e]"; do
    for p in $(pgrep -f "$pat"); do kill -9 "$p" 2>/dev/null; done
  done
  echo "done"
}
trap cleanup EXIT

echo "启动映射器 / 夹爪 FSM / 键盘 Mock（日志: /tmp/teleop_*.log）..."
"$PY" scripts/clutch_mapper_node.py > /tmp/teleop_mapper.log 2>&1 &
"$PY" scripts/gripper_fsm_node.py  > /tmp/teleop_gripper.log 2>&1 &
sleep 2
"$PY" scripts/mock_vr_node.py               # 键盘模式，前台；Ctrl+C 退出
