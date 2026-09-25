#!/usr/bin/env bash
# 遥操跟随台架的一键驱动：起栈 → 归位 → 跑三张表 → 收摊。
#
# 为什么要有这个脚本：手工分五个终端起栈容易漏参数，而"三处默认值必须一致"
# （stage2a 的 arm_control_mode / mujoco_model、stage3 的 output_mode）是我们踩过的坑。
# 这里把参数**显式**传一遍，不依赖默认值。
#
# ⚠️ 就绪判定的坑（2026-09-24 修）：`ros2 control list_controllers` 的输出带 ANSI 颜色码，
#    而且控制器**类型**那一列夹在名字和 active 之间——旧正则
#    `forward_command_controller_velocity[^a-z]*[[:space:]]+active` 永远匹配不上
#    （实测：去掉 ANSI 也不匹配，因为 [^a-z]* 跳不过含小写字母的类型名），
#    结果每次白等 90 次循环（约 2 分钟）然后打印一句假的"就绪"。现在先剥 ANSI，
#    再用 awk 取首尾字段判 active。
#
# 用法：
#   bash scripts/run_bench.sh                 # 三张表都跑
#   bash scripts/run_bench.sh bringup          # 只起栈+归位，保持运行（配合 verify_traversal.py）
#   超时 1800 s 的依据：实测每条穿越（含分步收拢与退让重试）74–80 s，survey 有 8 条
#   bash scripts/run_bench.sh survey          # 只跑安静度
#   bash scripts/run_bench.sh sine latency    # 指定子集
set +u
WS=/home/lcw/VR_teleoperation/ros2_ws
PKG=$WS/src/aubo_i5_teleop
POSES=$PKG/config/bench_poses.yaml
# ⚠️ 模型用**绝对路径**：脚本在 src 下跑，相对路径很容易数错层级
#    （曾写成 $PKG/../../assets/... → 解析到 ros2_ws/assets/，插件报 "model file does not exist" 后 abort）
# ⚠️ 2026-09-24 起台架必须用这个解释器跑：follow_bench.py 的末端位姿改成了
#    /joint_states + MuJoCo FK（TF2 Python listener 在 197 Hz 下积压 0.3–0.5 s，
#    污染 sine 相位与 latency D1/D2，见 follow_bench.py 文件头仪器约定 4）。
#    该 venv 同时有 rclpy 和 mujoco 3.12.0（与 mujoco_ros2_control 链的版本一致）；
#    系统 python3 的 mujoco 缺 typing_extensions 连 import 都过不了。
PY=/home/lcw/tomato_robot/.venv/bin/python
LOG=/tmp/bench
MODES=("$@")
[ ${#MODES[@]} -eq 0 ] && MODES=(survey sine latency)

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
cd "$PKG" || exit 1

# ⚠️ 每次起栈前强制重编译（约 10 s）：本包的 launch/脚本/config 是普通拷贝安装，
#    手改源文件后忘了 colcon build 就会带着旧参数跑（2026-09-25 审计发现 install 里
#    pose_tracking_settings.yaml 落后于源目录）。宁可多花 10 秒。
echo "########## 0/4 重编译（防 install 与源目录脱节）##########"
( cd "$WS" && colcon build --packages-select aubo_i5_teleop \
    --cmake-args -DCMAKE_BUILD_TYPE=Release > /tmp/bench_prebuild.log 2>&1 ) \
  || { echo "❌ 编译失败，见 /tmp/bench_prebuild.log"; exit 1; }
source "$WS/install/setup.bash"

wait_controller() {           # $1 = 控制器名
  for _ in $(seq 1 60); do
    if ros2 control list_controllers 2>/dev/null \
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
    timeout 3 ros2 node list 2>/dev/null | grep -q "$1" && return 0
    sleep 1
  done
  echo "❌ 等不到节点 $1"; return 1
}

wait_topic_pub() {            # $1 = 话题名
  for _ in $(seq 1 60); do
    timeout 3 ros2 topic info "$1" 2>/dev/null | grep -q "Publisher count: [1-9]" && return 0
    sleep 1
  done
  echo "❌ 等不到 $1 的发布者"; return 1
}

echo "########## 1/4 起 stage2a（位置模式）##########"
setsid ros2 launch aubo_i5_teleop stage2a_mujoco.launch.py \
  arm_control_mode:=position \
  mujoco_model:="/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml" > ${LOG}_2a.log 2>&1 &
wait_controller forward_command_controller_position || exit 1
wait_controller gripper_controller || exit 1
echo "stage2a 就绪（forward_command_controller_position active）"

echo "########## 2/4 起 stage2b + 归位 ##########"
setsid ros2 launch aubo_i5_teleop stage2b_moveit.launch.py > ${LOG}_2b.log 2>&1 &
wait_node /move_group || exit 1
timeout 60 "$PY" scripts/go_ready.py --mode position 2>&1 | grep -E "到位误差|末端位置"
echo "stage2b 就绪"

echo "########## 3/4 起 stage3（position 输出）##########"
setsid ros2 launch aubo_i5_teleop stage3_pose_tracking.launch.py output_mode:=position \
  > ${LOG}_3.log 2>&1 &
wait_topic_pub /forward_command_controller_position/commands || exit 1
echo "stage3 就绪"
timeout 10 ros2 param get /servo_pose_tracking moveit_servo.publish_period 2>&1 | sed 's/^/  /'
timeout 10 ros2 param get /servo_pose_tracking moveit_servo.x_proportional_gain 2>&1 | sed 's/^/  /'

echo "########## 4/4 跑台架 ##########"
for m in "${MODES[@]}"; do
  echo "───── 模式 $m ─────"
  case "$m" in
    # bringup：只起栈 + 归位，然后**停在这里**（不跑表、不收摊）。
    # 给"先单独验证一条穿越"用（scripts/verify_traversal.py），
    # 免得为了试一条路径把三张表（十几分钟）都跑一遍。
    bringup) echo "栈已就绪，保持运行。" ;;
    survey)  timeout 1800 "$PY" scripts/follow_bench.py --poses "$POSES" --survey \
               --csv /tmp/bench_survey.csv ;;
    sine)    timeout 1800 "$PY" scripts/follow_bench.py --poses "$POSES" --sine --limit 3 \
               --freqs 0.1 0.25 0.5 1.0 --csv /tmp/bench_sine.csv ;;
    latency) timeout 1800 "$PY" scripts/follow_bench.py --poses "$POSES" --latency --limit 3 \
               --reps 10 --csv /tmp/bench_latency.csv ;;
    *) echo "未知模式 $m" ;;
  esac
done

# bringup 模式下收摊要单独调用：bash scripts/run_bench.sh stop
if [ "${MODES[0]}" = "bringup" ] && [ ${#MODES[@]} -eq 1 ]; then
  echo "（bringup 模式：栈保持运行，不执行收摊）"
  exit 0
fi

echo "########## 收摊 ##########"
# ⚠️ 必须覆盖这些进程名：它们不是 launch 的名字，只杀 launch 会留下孤儿 —— 实测一度累积出
#    4 个 robot_state_publisher + 4 个 add_scene_floor 同时往同一棵 TF 树里发，把测量搞脏。
for pat in "stage3_pose_trackin[g]" "stage2b_movei[t]" "stage2a_mujoc[o]" \
           "ros2_contro[l]_node" "move_grou[p]" "rvi[z]2" \
           "robot_state_publisher/robot_state_pub" "add_scene_floo[r].py" \
           "servo_interfac[e].py" "pose_tracking_nod[e]"; do
  for p in $(pgrep -f "$pat"); do kill -TERM "$p" 2>/dev/null; done
done
sleep 3
for pat in "ros2_contro[l]_node" "servo_interfac[e].py" "pose_tracking_nod[e]" \
           "robot_state_publisher/robot_state_pub" "add_scene_floo[r].py" "rvi[z]2"; do
  for p in $(pgrep -f "$pat"); do kill -9 "$p" 2>/dev/null; done
done
echo "done"
