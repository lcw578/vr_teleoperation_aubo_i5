#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
AUBO_RUNTIME=${AUBO_RUNTIME_DIR:-/data/VR_teleoperation_runtime}
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
# Reuse the exact already-reviewed SDK; never silently change the read-only build.
AUBO_SDK_PATH=$("$AUBO_PYTHON" - "$AUBO_RUNTIME" <<'PY'
import hashlib,json,pathlib,sys
m=json.loads((pathlib.Path(sys.argv[1])/'readonly-build.json').read_text())
assert hashlib.sha256(pathlib.Path(m['release_library']).read_bytes()).hexdigest()==m['release_sha256'], 'SDK changed'
print(m['sdk_root'])
PY
)
cmake -S "$PROJECT_ROOT/teleop/aubo/native" -B "$AUBO_RUNTIME/build-stop-probe" \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$AUBO_PYTHON" \
  -DAUBO_SDK_ROOT="$AUBO_SDK_PATH" -DAUBO_BUILD_STOP_PROBE=ON
cmake --build "$AUBO_RUNTIME/build-stop-probe" --target _aubo_stop_probe --parallel 2
"$AUBO_PYTHON" - "$AUBO_RUNTIME" <<'PY'
import hashlib,json,pathlib,sys
root=pathlib.Path(sys.argv[1]).resolve()
m=json.loads((root/'readonly-build.json').read_text())
modules=list((root/'build-stop-probe/python').glob('_aubo_stop_probe*.so'))
assert len(modules)==1
manifest={k:m[k] for k in ('release_library','release_sha256')}
manifest.update(module_path=str(modules[0]),module_sha256=hashlib.sha256(modules[0].read_bytes()).hexdigest(),
    scope='stationary stop request diagnostic only; no motion/startup/mode/configuration API')
(root/'stop-probe-build.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('Stationary stop probe built. No SDK construction, connection or stop was executed.')
PY
