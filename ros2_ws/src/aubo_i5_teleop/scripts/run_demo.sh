#!/usr/bin/env bash
# 演示场景一键启动（DATA_PIPELINE_PLAN.md 阶段 A 验收）：起栈（demo 场景）→ 归位 → 键盘遥操。
#
# 与 run_bench.sh bringup 的区别只有两处：
#   1. mujoco_model 用 scene_ros2_demo.xml（含桌/方块/果篮，scene_ros2.xml 的并行变体）
#   2. add_scene_floor.py 加 --scene demo（桌+篮进 planning scene；base 模式不发布，
#      防止基准台架出现"幽灵桌/幽灵篮"）
#
# 用法：
#   bash scripts/run_demo.sh          # 前台键盘遥操（Ctrl+C 退出全部）
#   键位同 run_teleop.sh：WASD+QE 平移 / UO IK JL 旋转 / 空格=离合 / Z=夹爪
#   演示闭环：夹爪移到红方块上方 → 下降 → Z 夹住 → 提起 → 移到蓝篮上方 → Z 松开
set +u
WS=/home/lcw/VR_teleoperation/ros2_ws
PKG=$WS/src/aubo_i5_teleop
# 模型绝对路径（同 run_bench.sh 的教训：相对路径易数错层级）
DEMO_SCENE=/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2_demo.xml
PY=/home/lcw/tomato_robot/.venv/bin/python
LOG=/tmp/demo

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
cd "$PKG" || exit 1

# ⚠️ 2026-09-25 晚事故：CycloneDDS 自动选网卡在晚上漂移到 lo（lo 无 MULTICAST 标志
#    且并行参与者有端口竞态），servo_interface 建不出参与者 → stage3 死 → 键盘链断。
#    解法 = 显式钉住 wlp4s0（详见 config/cyclonedds_local.xml 文件头的事故链记录）。
export CYCLONEDDS_URI="file://$PKG/config/cyclonedds_local.xml"
# ⚠️ 同一事故的另一半根因（A/B 对照实测确认）：.bashrc:162 的 ROS_LOCALHOST_ONLY=1
#    会让 rmw_cyclonedds 强制注入自己的 lo 接口、压过上面的 wlp4s0 钉死
#    （实测 A=1 复现失败 / B=0 立即成功）。本机仿真走 wlp4s0 已足够，脚本内显式关闭。
export ROS_LOCALHOST_ONLY=0
# RMW 也显式钉死（ CycloneDDS）——不给"系统默认是什么"留任何悬念
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ⚠️ 2026-09-25 教训：ros2 CLI 守护进程会卡死（症状：ros2 node list 永久无响应），
#    wait_node 60 次循环什么都看不见 → 误报"等不到 /move_group"（实际已就绪）。
#    起栈前强制重置；等待函数一律 --no-daemon 绕开守护进程。
ros2 daemon stop >/dev/null 2>&1 || true

# ── 栈级清理（2026-09-25 新增）────────────────────────────────────
# 教训：run_bench.sh bringup 式的 setsid 启动在脚本退出后全部存活，而本脚本原先
# 只清理键盘三节点 → 每跑一次叠一整套栈。实测后果（3 套栈并存）：2 个
# /joint_states 发布者、命令同时灌进 2 个 controller_manager、servo_interface
# 建节点直接崩（rmw handle invalid）→ 键盘完全失灵。所以：
#   ① 起栈前先清残留；② 退出（含 Ctrl+C）时收摊整个栈。
_P_CLEANED=0
cleanup_stack() {
  [ "$_P_CLEANED" = 1 ] && return 0
  _P_CLEANED=1
  echo ""
  echo "收摊：关闭演示栈（含残留旧栈）..."
  local pats=("stage2a_mujoco.launch.py" "stage2b_moveit.launch.py" \
              "stage3_pose_tracking.launch.py" "add_scene_floor.py" \
              "go_ready.py" "servo_interface.py" "servo_pose_tracking" \
              "multi_camera_renderer.py" "session_manager.py" \
              "move_group" "ros2_control_node" "rviz2")
  for pat in "${pats[@]}"; do
    for p in $(pgrep -f "$pat"); do kill -TERM "$p" 2>/dev/null; done
  done
  sleep 2
  for pat in "${pats[@]}"; do
    for p in $(pgrep -f "$pat"); do kill -KILL "$p" 2>/dev/null; done
  done
  echo "收摊完成。"
}
trap cleanup_stack EXIT INT TERM

# 起栈前清掉上一次的残留（清完重置标志，让 EXIT 陷阱仍然生效）
cleanup_stack
_P_CLEANED=0

# ── 实例互斥：同时只允许一个 run_demo.sh（多实例会互相清对方的栈、覆盖彼此日志
#    ——2026-09-25 晚"配置改了却不生效"的混乱就是这么来的）──
for p in $(pgrep -f "run_demo.s[h]"); do
  [ "$p" != "$$" ] && { echo "❌ 另一个 run_demo.sh 正在运行 (pid $p)，先等它退出"; exit 1; }
done

# ── DDS 自检：10 秒内明确暴露配置问题，而不是让栈起一半神秘死亡。
#    必须用 C++ 节点测（Python 端对坏 CYCLONEDDS_URI 静默回退，是假阳性——已踩坑）──
echo "########## 0.5/3 DDS 自检（wlp4s0 钉死验证）##########"
# Wi-Fi 上 DDS 初始化耗时有波动（实测 1s～20s+），重试 2 次容忍之
DDS_OK=0
for attempt in 1 2; do
  timeout 20 ros2 run demo_nodes_cpp talker > /tmp/demo_ddstest.log 2>&1 || true
  if grep -q "Publishing" /tmp/demo_ddstest.log; then DDS_OK=1; break; fi
  echo "  第 $attempt 次尝试未通过（20s），重试..."
done
if [ "$DDS_OK" = 1 ]; then
  echo "DDS 自检 OK"
else
  echo "❌ DDS 自检失败（CYCLONEDDS_URI=$CYCLONEDDS_URI，RMW=$RMW_IMPLEMENTATION）"
  echo "   talker 实际输出（诊断用）："
  head -8 /tmp/demo_ddstest.log
  exit 1
fi
echo "DDS 自检 OK"

# 防 install 与源目录脱节（同 run_bench.sh 的约定）
echo "########## 0/3 重编译（防 install 与源目录脱节）##########"
( cd "$WS" && colcon build --packages-select aubo_i5_teleop \
    --cmake-args -DCMAKE_BUILD_TYPE=Release > /tmp/demo_prebuild.log 2>&1 ) \
  || { echo "❌ 编译失败，见 /tmp/demo_prebuild.log"; exit 1; }
source "$WS/install/setup.bash"

wait_controller() {           # $1 = 控制器名
  for _ in $(seq 1 60); do
    # timeout 8：ros2 control CLI 偶发挂死（守护进程/发现层），不给它挂住整个启动的机会
    if timeout 8 ros2 control list_controllers 2>/dev/null \
         | sed 's/\x1b\[[0-9;]*m//g' | awk '{print $1, $NF}' \
         | grep -qx "$1 active"; then
      return 0
    fi
    sleep 1
  done
  echo "❌ 等不到控制器 $1 变 active"; return 1
}

wait_node() {                 # $1 = 节点名
  for _ in $(seq 1 60); do
    timeout 5 ros2 node list --no-daemon 2>/dev/null | grep -q "$1" && return 0
    sleep 1
  done
  echo "❌ 等不到节点 $1"; return 1
}

wait_topic_pub() {            # $1 = 话题名
  for _ in $(seq 1 60); do
    timeout 5 ros2 topic info "$1" --no-daemon 2>/dev/null | grep -q "Publisher count: [1-9]" && return 0
    sleep 1
  done
  echo "❌ 等不到 $1 的发布者"; return 1
}

echo "########## 1/3 起 stage2a（位置模式，演示场景）##########"
setsid ros2 launch aubo_i5_teleop stage2a_mujoco.launch.py \
  arm_control_mode:=position \
  mujoco_model:="$DEMO_SCENE" > ${LOG}_2a.log 2>&1 &
wait_controller forward_command_controller_position || exit 1
wait_controller gripper_controller || exit 1
echo "stage2a 就绪"

echo "########## 1.5/3 起三路虚拟相机渲染节点（阶段 B）##########"
# 订阅 /joint_states 渲染 D405 腕部 + D435 右/左，发布 /camera/*/image_rect_color
# （venv python 自带 mujoco 3.12.0 与仿真同版；节点内部自设 MUJOCO_GL=egl）
"$PY" scripts/multi_camera_renderer.py > ${LOG}_cams.log 2>&1 &
sleep 2

echo "########## 1.6/3 起会话管理节点（录制状态机，IDLE 起步）##########"
# P 键（joy[3]）=开始/暂停/恢复录制段；Esc（joy[4]）=结束当前段。袋落 /data/rosbags/
"$PY" scripts/session_manager.py > ${LOG}_session.log 2>&1 &
sleep 1

echo "########## 2/3 起 stage2b + 归位 + 演示场景 planning scene ##########"
setsid ros2 launch aubo_i5_teleop stage2b_moveit.launch.py > ${LOG}_2b.log 2>&1 &
wait_node /move_group || exit 1
"$PY" scripts/add_scene_floor.py --scene demo > ${LOG}_scene.log 2>&1 &
sleep 1   # 让桌/篮先进 planning scene，再归位（ready 位姿不与任何演示物体接触）
timeout 60 "$PY" scripts/go_ready.py --mode position 2>&1 | grep -E "到位误差|末端位置"

echo "########## 3/3 起 stage3（position 输出）+ 键盘 ##########"
setsid ros2 launch aubo_i5_teleop stage3_pose_tracking.launch.py output_mode:=position \
  > ${LOG}_3.log 2>&1 &
wait_topic_pub /forward_command_controller_position/commands || exit 1
echo "栈就绪，进入键盘遥操（空格=离合按住才动，Z=夹爪）"
# 不用 exec：退出后本脚本的 EXIT 陷阱还要收摊整个栈
# （run_teleop.sh 自己的 trap 只收键盘三节点，管不到栈）
bash scripts/run_teleop.sh
