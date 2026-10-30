#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$PROJECT_ROOT/scripts/teleop_env.sh"
teleop_env
cd "$PROJECT_ROOT"
exec "$TELEOP_PYTHON" -m teleop.aubo.servoj_probe_cli "$@"
