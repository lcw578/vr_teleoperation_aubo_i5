#!/usr/bin/env bash
# Shared Python boundary for all VR entry points. Source, then call teleop_env.
teleop_env() {
  export TELEOP_RUNTIME_ROOT="${TELEOP_RUNTIME_ROOT:-/data/VR_teleoperation_runtime}"
  export TELEOP_PYTHON="${TELEOP_PYTHON:-$TELEOP_RUNTIME_ROOT/.venv/bin/python}"
  [[ -x "$TELEOP_PYTHON" ]] || {
    echo "VR Python unavailable: $TELEOP_PYTHON; see environment/README.md" >&2
    return 1
  }
  unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LIBRARY_PATH
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PKG_CONFIG_PATH
  unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_DOMAIN_ID ROS_LOCALHOST_ONLY
  unset RMW_IMPLEMENTATION CYCLONEDDS_URI GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
  export CONDA_SHLVL=0
  export PYTHONNOUSERSITE=1 PIP_USER=0
  export PATH="$(dirname "$TELEOP_PYTHON"):/usr/local/bin:/usr/bin:/bin:$HOME/.local/bin"
  export VIRTUAL_ENV="$(dirname "$(dirname "$TELEOP_PYTHON")")"
}

teleop_ros_env() {
  teleop_env || return
  local overlay="$TELEOP_RUNTIME_ROOT/ros2/install"
  [[ -f "$overlay/local_setup.bash" ]] || {
    echo "Build the isolated ROS overlay with scripts/build_ros_clean.sh" >&2
    return 1
  }
  # ROS setup scripts are not nounset-safe; restore the caller's option.
  local had_nounset=0
  [[ $- != *u* ]] || had_nounset=1
  set +u
  source /opt/ros/humble/setup.bash
  source "$overlay/local_setup.bash"
  [[ $had_nounset == 0 ]] || set -u
}
