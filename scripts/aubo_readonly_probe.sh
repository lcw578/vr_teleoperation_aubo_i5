#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$PROJECT_ROOT/scripts/teleop_env.sh"
teleop_env
TELEOP_RUNTIME=$TELEOP_PYTHON
cd "$PROJECT_ROOT"
exec "$TELEOP_RUNTIME" -m teleop.aubo.probe "$@"
