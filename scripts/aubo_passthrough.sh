#!/usr/bin/env bash
# Quest VR -> latest local IK -> stationary priming -> bounded actual-dt trajectory -> TCP2CAN.
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=plan
duration=0
relay_port=8443
gripper=1
record=1
task='Quest 3 TCP2CAN 六轴相对位姿遥操'
camera_config=''
home_reference=''
diagnostic_output=''
headless=0
while (($#)); do
  case "$1" in
    --execute|--check) [[ $mode == plan ]] || { echo 'Choose --execute or --check.' >&2; exit 2; }; mode=${1#--}; shift ;;
    --no-gripper) gripper=0; shift ;;
    --no-record) record=0; shift ;;
    --headless) headless=1; shift ;;
    --duration|--relay-port|--task|--record-cameras|--diagnostic-output|--home-reference)
      [[ $# -ge 2 && -n $2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      case "$1" in
        --duration) duration=$2 ;;
        --relay-port) relay_port=$2 ;;
        --task) task=$2 ;;
        --record-cameras) camera_config=$2 ;;
        --home-reference) home_reference=$2 ;;
        --diagnostic-output) diagnostic_output=$2 ;;
      esac
      shift 2 ;;
    --help|-h)
      echo 'Usage: bash scripts/aubo_passthrough.sh [--check|--execute] [--no-gripper] [--no-record] [--headless] [--record-cameras camera-config.json] [--home-reference home.json] [--diagnostic-output path.jsonl] [--duration seconds] [--relay-port port] [--task description]'
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
if [[ ! $duration =~ ^[0-9]+$ || ${#duration} -gt 7 || ! $relay_port =~ ^[0-9]+$ || ${#relay_port} -gt 5 ]]; then
  echo 'Duration must be a nonnegative integer; relay port must be 1..65535.' >&2; exit 2
fi
duration=$((10#$duration)); relay_port=$((10#$relay_port))
((relay_port >= 1 && relay_port <= 65535)) || { echo 'Relay port must be 1..65535.' >&2; exit 2; }
if [[ $mode == plan ]]; then
  echo 'TCP2CAN / A motion only, B state + independent joint telemetry / stationary priming / bounded velocity, acceleration and jerk / 5ms minimum start-to-start period.'
  echo "USB relay localhost:$relay_port; gripper=$gripper; recording=$record; duration=$duration (0=until exit)."
  echo 'Use --check for offline readiness or --execute to start VR. A valid home reference enables single-press left Y return in this six-axis baseline.'
  exit 0
fi
# Prefer /data. Reuse the existing adjacent vehicle runtime if /data is absent.
if [[ -z ${TELEOP_RUNTIME_ROOT:-} ]]; then
  if [[ -d /data/VR_teleoperation_runtime ]]; then
    export TELEOP_RUNTIME_ROOT=/data/VR_teleoperation_runtime
  else
    export TELEOP_RUNTIME_ROOT="$(dirname "$PROJECT_ROOT")/VR_teleoperation_runtime"
  fi
fi
export AUBO_RUNTIME_DIR=${AUBO_RUNTIME_DIR:-$TELEOP_RUNTIME_ROOT}
if [[ $AUBO_RUNTIME_DIR != /data/* ]]; then
  export AUBO_ALLOWED_ROOTS="${AUBO_ALLOWED_ROOTS:+$AUBO_ALLOWED_ROOTS:}$AUBO_RUNTIME_DIR"
fi
source "$PROJECT_ROOT/scripts/teleop_env.sh"
teleop_env
cd "$PROJECT_ROOT"
"$TELEOP_PYTHON" - <<'PY'
from teleop.aubo.follow import load_follow_native
m=load_follow_native()
assert m.supports_hot_clutch and m.supports_passthrough and m.supports_process_state_query and m.supports_process_waypoint_feedback
print('SDK and native binary hashes verified; A motion / B state and joint telemetry available. No SDK connection opened.')
PY
command -v adb >/dev/null || { echo 'adb is required for USB WebXR.' >&2; exit 2; }
if [[ $mode == check ]]; then
  echo 'Offline readiness passed. No robot connection or motion.'
  exit 0
fi
mapfile -t headsets < <(adb devices | awk '$2 == "device" {print $1}')
[[ ${#headsets[@]} == 1 ]] || { echo 'Connect exactly one USB headset and accept USB debugging.' >&2; exit 2; }
adb -s "${headsets[0]}" reverse "tcp:$relay_port" "tcp:$relay_port"
args=()
if [[ $gripper == 1 ]]; then args+=(--gripper); fi
if [[ $record == 1 ]]; then args+=(--record --task "$task"); fi
if [[ -n $camera_config ]]; then
  ((gripper == 1 && record == 1)) || { echo '--record-cameras requires recording and --gripper.' >&2; exit 2; }
  args+=(--record-fps 30 --record-cameras "$camera_config")
fi
if [[ -n $diagnostic_output ]]; then args+=(--diagnostic-output "$diagnostic_output"); fi
if [[ -n $home_reference ]]; then args+=(--home-reference "$home_reference"); fi
if [[ $headless == 1 ]]; then args+=(--headless); fi
printf 'Headset browser: http://localhost:%s/ -> Start Teleop. Right Grip controls motion; release brakes then holds in passthrough (20s idle exit).\n' "$relay_port"
printf 'TCP2CAN stationary priming + actual-dt trajectory; SDK password follows. Valid home reference enables single-press left Y return; right A hold halves motion scale.\n'
exec bash "$PROJECT_ROOT/scripts/aubo_quest_follow.sh" \
  --execute-six-axis --command-mode passthrough --waypoint-feedback --parallel-state-query --separate-waypoint-connection \
  --home-trigger oneshot \
  --usb --host 192.168.1.5 --port 8899 --user aubo --rotation world \
  --relay-port "$relay_port" --duration "$duration" "${args[@]}"
