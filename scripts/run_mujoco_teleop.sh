#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
TELEOP_RUNTIME=${TELEOP_PYTHON:-/home/lcw/tomato_robot/.venv/bin/python}
if [[ ! -x "$TELEOP_RUNTIME" ]]; then
  echo "Python runtime unavailable: $TELEOP_RUNTIME; set TELEOP_PYTHON" >&2
  exit 1
fi
# Headless smoke tests can use EGL; interactive desktop keeps GLFW.
for arg in "$@"; do
  if [[ "$arg" == "--headless" ]]; then export MUJOCO_GL=${MUJOCO_GL:-egl}; fi
done
exec "$TELEOP_RUNTIME" "$PROJECT_ROOT/teleop/mujoco_session.py" "$@"
