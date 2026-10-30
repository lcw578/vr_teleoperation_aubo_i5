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
"$AUBO_PYTHON" tests/test_aubo_quest_preview.py
"$AUBO_PYTHON" tests/test_aubo_follow.py
"$AUBO_PYTHON" tests/test_aubo_six_axis.py
"$AUBO_PYTHON" tests/test_aubo_state_query.py
"$AUBO_PYTHON" tests/test_aubo_follow_analysis.py
"$AUBO_PYTHON" tests/test_aubo_workspace.py
"$AUBO_PYTHON" tests/test_aubo_stationary_stop.py
"$AUBO_PYTHON" tests/test_aubo_j6_trial.py
"$AUBO_PYTHON" tests/test_mujoco_teleop.py
"$AUBO_PYTHON" tests/smoke_aubo_readonly_binding.py
"$AUBO_PYTHON" tests/smoke_aubo_follow_binding.py
"$AUBO_PYTHON" tests/smoke_aubo_parallel_state.py
"${AUBO_RUNTIME_DIR:-/data/VR_teleoperation_runtime}/build-follow/test_follow_profile"
"${AUBO_RUNTIME_DIR:-/data/VR_teleoperation_runtime}/build-follow/test_passthrough"
bash scripts/aubo_readonly_probe.sh
bash scripts/aubo_stationary_stop_probe.sh
bash scripts/aubo_j6_trial.sh
bash -n scripts/build_aubo_stop_probe.sh scripts/aubo_stationary_stop_probe.sh
bash -n scripts/build_aubo_j6_trial.sh scripts/aubo_j6_trial.sh
bash scripts/aubo_quest_preview.sh
bash -n scripts/aubo_quest_preview.sh
bash scripts/aubo_quest_follow.sh
bash -n scripts/aubo_quest_follow.sh scripts/build_aubo_follow.sh
