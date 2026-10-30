#!/usr/bin/env bash
# Build simulation ROS packages with only the Humble underlay. No nodes run.
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
RUNTIME_ROOT=${TELEOP_RUNTIME_ROOT:-/data/VR_teleoperation_runtime}
if [[ ${TELEOP_CLEAN_BUILD:-0} != 1 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-lcw}" LOGNAME="${LOGNAME:-lcw}" \
    LANG=C.UTF-8 PATH=/usr/bin:/bin PYTHONNOUSERSITE=1 \
    TELEOP_CLEAN_BUILD=1 TELEOP_RUNTIME_ROOT="$RUNTIME_ROOT" \
    bash --noprofile --norc "$0" "$@"
fi
[[ -d "$RUNTIME_ROOT" && -w "$RUNTIME_ROOT" ]] || {
  echo "Writable project runtime required: $RUNTIME_ROOT" >&2; exit 1;
}
case "$(realpath "$RUNTIME_ROOT")" in /data/*) ;; *) exit 1;; esac
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$PROJECT_ROOT/ros2_ws"
exec colcon --log-base "$RUNTIME_ROOT/ros2/log" build \
  --build-base "$RUNTIME_ROOT/ros2/build" \
  --install-base "$RUNTIME_ROOT/ros2/install" \
  --executor sequential \
  --packages-select aubo_description aubo_moveit_config aubo_msgs \
    dh_ag95_description mujoco_3d_lidar mujoco_ros2_control \
    mujoco_ros2_control_msgs mujoco_ros2_control_plugins aubo_i5_teleop ros_joints_plan \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DMUJOCO_DEP_VERSION_lodepng=e584a8490db1b76b81bf298c11c09d3b55d9c270 \
    -DAMENT_CMAKE_SYMLINK_INSTALL=OFF -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
