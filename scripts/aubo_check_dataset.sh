#!/usr/bin/env bash
# Offline LeRobot reader only; never imports the robot SDK.
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"
AUBO_LEROBOT_PYTHON=${AUBO_LEROBOT_PYTHON:-/data/lerobot/.venv/bin/python}
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
exec "$AUBO_LEROBOT_PYTHON" -m teleop.aubo.check_dataset "$@"
