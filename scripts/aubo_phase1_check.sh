#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$PROJECT_ROOT/scripts/teleop_env.sh"
teleop_env
AUBO_PYTHON=$TELEOP_PYTHON
cd "$PROJECT_ROOT"
"$AUBO_PYTHON" tests/test_aubo_offline.py
"$AUBO_PYTHON" tests/test_aubo_model_shadow.py
"$AUBO_PYTHON" tests/test_aubo_runtime.py
"$AUBO_PYTHON" tests/test_aubo_stationary_stop.py
bash -n scripts/aubo_phase1_check.sh scripts/aubo_offline_check.sh
git diff --check
echo "Phase-one offline runtime checks passed; no SDK login, motion, stop, or gripper I/O was used."
