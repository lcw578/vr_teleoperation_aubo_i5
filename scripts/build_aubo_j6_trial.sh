#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
AUBO_RUNTIME=${AUBO_RUNTIME_DIR:-/data/VR_teleoperation_runtime}
source "$PROJECT_ROOT/scripts/teleop_env.sh"
teleop_env
AUBO_PYTHON=$TELEOP_PYTHON
if [[ ! -d "$AUBO_RUNTIME" || ! -w "$AUBO_RUNTIME" ]]; then
  echo "A writable /data runtime directory is required" >&2; exit 1
fi
case "$(realpath "$AUBO_RUNTIME")" in /data/*) ;; *) exit 1;; esac
AUBO_SDK_PATH=$("$AUBO_PYTHON" - "$AUBO_RUNTIME" <<'PY'
import hashlib,json,pathlib,sys
m=json.loads((pathlib.Path(sys.argv[1])/'readonly-build.json').read_text())
assert hashlib.sha256(pathlib.Path(m['release_library']).read_bytes()).hexdigest()==m['release_sha256']
print(m['sdk_root'])
PY
)
cmake -S "$PROJECT_ROOT/teleop/aubo/native" -B "$AUBO_RUNTIME/build-j6-trial" \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$AUBO_PYTHON" \
  -DAUBO_SDK_ROOT="$AUBO_SDK_PATH" -DAUBO_BUILD_J6_TRIAL=ON
cmake --build "$AUBO_RUNTIME/build-j6-trial" --target _aubo_j6_trial --parallel 2
"$AUBO_PYTHON" - "$AUBO_RUNTIME" <<'PY'
import hashlib,json,pathlib,sys
r=pathlib.Path(sys.argv[1]).resolve()
m=json.loads((r/'readonly-build.json').read_text())
modules=list((r/'build-j6-trial/python').glob('_aubo_j6_trial*.so'));assert len(modules)==1
out={k:m[k] for k in ('release_library','release_sha256')}
out.update(module_path=str(modules[0]),module_sha256=hashlib.sha256(modules[0].read_bytes()).hexdigest(),
  scope='supervised J6 single target, bounded 5/10/20Hz sequences, or fixed +10deg then anchor reversal; not a general follower')
(r/'j6-trial-build.json').write_text(json.dumps(out,indent=2)+'\n')
print('J6 trial built. No SDK connection or movement executed.')
PY
