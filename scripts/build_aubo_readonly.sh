#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
AUBO_RUNTIME=${AUBO_RUNTIME_DIR:-/data/VR_teleoperation_runtime}
AUBO_SDK_PATH=${AUBO_SDK_ROOT:-/data/robot_SDK/新版SDK/aubo_sdk-2.5.0-linux64.bf7070c}
source "$PROJECT_ROOT/scripts/teleop_env.sh"
teleop_env
AUBO_PYTHON=$TELEOP_PYTHON
if [[ ! -d "$AUBO_RUNTIME" || ! -w "$AUBO_RUNTIME" ]]; then
  echo "Create a writable runtime directory first: $AUBO_RUNTIME" >&2
  exit 1
fi
case "$(realpath "$AUBO_RUNTIME")" in
  /data/*) ;;
  *) echo "Build outputs must be in an independent /data directory" >&2; exit 1;;
esac
cmake -S "$PROJECT_ROOT/teleop/aubo/native" -B "$AUBO_RUNTIME/build-readonly" \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$AUBO_PYTHON" \
  -DAUBO_SDK_ROOT="$AUBO_SDK_PATH"
cmake --build "$AUBO_RUNTIME/build-readonly" --parallel 2
"$AUBO_PYTHON" - "$AUBO_RUNTIME" "$AUBO_SDK_PATH" <<'PY'
import hashlib, json, pathlib, sys
runtime, sdk = map(pathlib.Path, sys.argv[1:])
binary = sdk / 'lib/libaubo_sdk.so.2.5.0'
manifest = {'python': sys.executable, 'sdk_root': str(sdk.resolve()),
            'release_library': str(binary.resolve()),
            'release_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
            'module_directory': str(runtime.resolve() / 'build-readonly/python'),
            'scope': 'compiled read-only binding; no controller connection or hardware verification'}
(runtime / 'readonly-build.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Read-only build complete. No SDK login or movement was executed.')
PY
