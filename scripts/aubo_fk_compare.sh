#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$PROJECT_ROOT/scripts/teleop_env.sh"
teleop_env
AUBO_PYTHON=$TELEOP_PYTHON
cd "$PROJECT_ROOT"
exec "$AUBO_PYTHON" -m teleop.aubo.model "$@"
