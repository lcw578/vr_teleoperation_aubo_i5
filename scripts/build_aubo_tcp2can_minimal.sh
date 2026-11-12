#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
AUBO_TEST_RUNTIME=${AUBO_RUNTIME_DIR:-/data/VR_teleoperation_runtime}
AUBO_TEST_SDK=$(python3 - "$AUBO_TEST_RUNTIME" <<'PY'
import hashlib, json, pathlib, sys
manifest = json.loads((pathlib.Path(sys.argv[1]) / 'readonly-build.json').read_text())
assert hashlib.sha256(pathlib.Path(manifest['release_library']).read_bytes()).hexdigest() == manifest['release_sha256'], 'SDK library hash mismatch'
print(manifest['sdk_root'])
PY
)
mkdir -p "$AUBO_TEST_RUNTIME/tcp2can-minimal"
g++ -std=c++17 -O2 -Wall -Wextra -Werror \
  -I "$AUBO_TEST_SDK/include" \
  "$PROJECT_ROOT/teleop/aubo/native/tcp2can_minimal.cpp" \
  "$AUBO_TEST_SDK/lib/libaubo_sdk.so.2.5.0" \
  -Wl,-rpath,"$AUBO_TEST_SDK/lib" -pthread \
  -o "$AUBO_TEST_RUNTIME/tcp2can-minimal/tcp2can_minimal"
"$AUBO_TEST_RUNTIME/tcp2can-minimal/tcp2can_minimal"
