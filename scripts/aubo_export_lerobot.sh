#!/usr/bin/env bash
# Offline only: the LeRobot Python environment stays outside the live SDK loop.
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"
AUBO_LEROBOT_PYTHON=${AUBO_LEROBOT_PYTHON:-/data/lerobot/.venv/bin/python}
if [[ ! -x "$AUBO_LEROBOT_PYTHON" ]]; then
  printf 'LeRobot Python unavailable: %s\n' "$AUBO_LEROBOT_PYTHON" >&2
  exit 1
fi
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
exec "$AUBO_LEROBOT_PYTHON" -m teleop.aubo.export_lerobot "$@"
