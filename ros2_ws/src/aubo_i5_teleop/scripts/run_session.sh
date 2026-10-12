#!/usr/bin/env bash
# Foreground, owned-process Route B simulation supervisor. Ctrl+C stops its children.
set -eo pipefail
TASK_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ ! -d "$TASK_ROOT/assets/aubo_i5" ]]; do
  [[ "$TASK_ROOT" != / ]] || { echo 'Cannot locate project assets'; exit 1; }
  TASK_ROOT=$(dirname "$TASK_ROOT")
done
TASK_WS="$TASK_ROOT/ros2_ws"
TASK_PKG="$TASK_WS/src/aubo_i5_teleop"
TASK_PY=${TELEOP_PYTHON:-/home/lcw/tomato_robot/.venv/bin/python}
source /opt/ros/humble/setup.bash
source "$TASK_WS/install/setup.bash"
export TELEOP_GRIPPER=${GRIPPER:-${TELEOP_GRIPPER:-rg}}
INPUT=${INPUT:-quest}
for argument in "$@"; do
  case "$argument" in
    --vr) INPUT=quest ;;
    --mock) INPUT=mock_vr ;;
    *) echo "Unknown argument: $argument (use --vr / --mock)"; exit 2 ;;
  esac
done
[[ "$INPUT" == quest || "$INPUT" == mock_vr ]] || { echo 'INPUT must be quest or mock_vr'; exit 2; }
export ROS_LOG_DIR=${ROS_LOG_DIR:-$TASK_ROOT/data/ros_logs}
CAMERAS=${CAMERAS:-true}
# Recording is opt-in. The logger node is not started unless RECORDING=true.
RECORDING=${RECORDING:-false}
MOVEIT=${MOVEIT:-true}
if [[ "$TELEOP_GRIPPER" == rg ]]; then
  SCENE=${SCENE:-$TASK_ROOT/assets/aubo_i5/scene_ros2_rg_cameras.xml}
else
  SCENE=${SCENE:-$TASK_ROOT/assets/aubo_i5/scene_ros2_demo.xml}
fi
# All model consumers use the exact same scene selected for the simulator.
export TELEOP_MODEL="$SCENE"
exec 9>"$TASK_ROOT/.teleop_session.lock"
flock -n 9 || { echo 'Another Route B session owns this workspace.'; exit 1; }
TASK_CHILDREN=()
cleanup() {
  trap - EXIT INT TERM
  for child in "${TASK_CHILDREN[@]}"; do
    kill -INT -- "-$child" 2>/dev/null || kill -INT "$child" 2>/dev/null || true
  done
  # Background shells may inherit ignored SIGINT. Bound shutdown and only
  # signal the process groups created by this supervisor.
  for ((attempt=0;attempt<50;attempt++)); do
    local alive=0
    for child in "${TASK_CHILDREN[@]}"; do kill -0 -- "-$child" 2>/dev/null && alive=1; done
    [[ "$alive" == 1 ]] || break
    sleep 0.1
  done
  for child in "${TASK_CHILDREN[@]}"; do
    kill -TERM -- "-$child" 2>/dev/null || kill -TERM "$child" 2>/dev/null || true
  done
  for ((attempt=0;attempt<50;attempt++)); do
    local alive=0
    for child in "${TASK_CHILDREN[@]}"; do kill -0 -- "-$child" 2>/dev/null && alive=1; done
    [[ "$alive" == 1 ]] || break
    sleep 0.1
  done
  for child in "${TASK_CHILDREN[@]}"; do
    kill -KILL -- "-$child" 2>/dev/null || kill -KILL "$child" 2>/dev/null || true
  done
  for child in "${TASK_CHILDREN[@]}"; do wait "$child" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start() {
  local label=$1; shift
  setsid "$@" >"/tmp/teleop_${label}.log" 2>&1 &
  TASK_CHILDREN+=("$!")
}
start sim ros2 launch aubo_i5_teleop stage2a_mujoco.launch.py \
  gripper:="$TELEOP_GRIPPER" mujoco_model:="$SCENE" arm_control_mode:=position headless:=true use_sim_time:=true
READY=0
for ((attempt=0;attempt<40;attempt++)); do
  kill -0 "${TASK_CHILDREN[0]}" || { echo 'Simulator exited; see /tmp/teleop_sim.log'; exit 1; }
  if timeout 3 ros2 control list_controllers 2>/dev/null | grep -Eq 'forward_command_controller_position.*active'; then READY=1; break; fi
  sleep 0.5
done
[[ "$READY" == 1 ]] || { echo 'Position controller did not become active.'; exit 1; }
if [[ "$MOVEIT" == true ]]; then
  start moveit ros2 launch aubo_i5_teleop stage2b_moveit.launch.py gripper:="$TELEOP_GRIPPER" use_sim_time:=true
fi
# No tracker/input publisher competes with this exclusive initialization step.
timeout 30 "$TASK_PY" "$TASK_PKG/scripts/go_ready.py" --mode position --ros-args -p use_sim_time:=true
QUEST=true
[[ "$INPUT" == mock_vr ]] && QUEST=false
start control ros2 launch aubo_i5_teleop route_b_control.launch.py \
  input:="$INPUT" quest:="$QUEST" scene:="$SCENE" cameras:="$CAMERAS" recording:="$RECORDING" \
  python_executable:="$TASK_PY" relay_uri:="${TELEOP_RELAY_URI:-wss://127.0.0.1:8443/ws}" use_sim_time:=true
echo "Route B ready: gripper=$TELEOP_GRIPPER, input=$INPUT, scene=$SCENE"
if [[ "$RECORDING" == true ]]; then
  echo 'Recording enabled. Start explicitly: ros2 service call /recording/start std_srvs/srv/Trigger "{}"'
else
  echo 'Recording disabled (default). Use RECORDING=true to enable it for this session.'
fi
if [[ "$INPUT" == mock_vr ]]; then
  "$TASK_PY" "$TASK_PKG/scripts/mock_vr_node.py" --ros-args -p use_sim_time:=true
else
  wait -n "${TASK_CHILDREN[@]}"
fi
