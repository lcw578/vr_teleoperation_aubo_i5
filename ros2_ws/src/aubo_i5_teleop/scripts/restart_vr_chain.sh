#!/usr/bin/env bash
# Restart entry for the complete owned Route B session; stop its supervisor first.
set -e
TASK_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
source "$TASK_ROOT/scripts/teleop_env.sh"
teleop_ros_env
export TELEOP_GRIPPER=${GRIPPER:-${TELEOP_GRIPPER:-rg}}
# Stop the running supervisor with Ctrl+C and use authoritative_session.sh for a
# complete restart. Starting a second tracker would create competing commands.
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_session.sh" "$@"
