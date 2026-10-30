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
cmake -S "$PROJECT_ROOT/teleop/aubo/native" -B "$AUBO_RUNTIME/build-follow" \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$AUBO_PYTHON" \
  -DAUBO_SDK_ROOT="$AUBO_SDK_PATH" -DAUBO_BUILD_FOLLOW=ON
cmake --build "$AUBO_RUNTIME/build-follow" --target _aubo_follow _aubo_follow_fake test_follow_profile test_passthrough --parallel 2
"$AUBO_RUNTIME/build-follow/test_follow_profile"
"$AUBO_RUNTIME/build-follow/test_passthrough"
"$AUBO_PYTHON" - "$AUBO_RUNTIME" <<'PY'
import hashlib,json,pathlib,sys
r=pathlib.Path(sys.argv[1]).resolve()
m=json.loads((r/'readonly-build.json').read_text())
modules=list((r/'build-follow/python').glob('_aubo_follow*.so'));assert len(modules)==1
out={k:m[k] for k in ('release_library','release_sha256')}
out.update(module_path=str(modules[0]),module_sha256=hashlib.sha256(modules[0].read_bytes()).hexdigest(),
  scope='supervised J6 or explicit six-axis: six-axis passthrough 200Hz or follow default 20Hz / explicit 25Hz, 50Hz or 200Hz requested; opt-in independent read-only state SDK connection for six-axis follow with redundant A/B waypoint push and incremental cache reads; B query period measured start-to-start, single inflight, no catch-up; 200ms state/joint reception age revokes motion without killing healthy queries, no reconnect; streaming diagnostics retain 4096 history events plus bounded queue; passthrough default with enter/send/leave lifecycle, legacy J6 10Hz, 500ms SDK budget, 650ms gap budget, optional waypoint push, controller-reported full range; six-axis direct IK without host step shaping or SDK profile writes/reset or unreliable SDK profile readback; explicit profile inspection remains separate; legacy J6 retains explicit 10deg/s and 5deg/s2 and step checks; serial motion/stop, command-feedback differences diagnostic only; no startup/home')
(r/'follow-build.json').write_text(json.dumps(out,indent=2)+'\n')
print('Supervised follower built. No SDK connection or movement executed.')
PY
