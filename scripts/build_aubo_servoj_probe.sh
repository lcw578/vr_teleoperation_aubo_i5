#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
AUBO_RUNTIME=${AUBO_RUNTIME_DIR:-/data/VR_teleoperation_runtime}
SDK_ROOT=/data/robot_SDK/arcs-servoj-0.24.1-rc.3/aubo_sdk-0.24.1-rc.3-Linux_x86_64+318754d
source "$PROJECT_ROOT/scripts/teleop_env.sh"
teleop_env
[[ -d "$AUBO_RUNTIME" && -w "$AUBO_RUNTIME" ]] || { echo "Writable /data runtime required" >&2; exit 1; }
case "$(realpath "$AUBO_RUNTIME")" in
  /data/*) ;;
  *) echo "Build outputs must be in an independent /data directory" >&2; exit 1;;
esac
[[ -f "$SDK_ROOT/lib/libaubo_sdk.so" ]] || { echo "ARCS ServoJoint SDK is not installed: $SDK_ROOT" >&2; exit 1; }
PYBIND11_DIR=$("$TELEOP_PYTHON" -m pybind11 --cmakedir)
cmake -S "$PROJECT_ROOT/teleop/aubo/servoj_probe" -B "$AUBO_RUNTIME/build-servoj-probe" \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$TELEOP_PYTHON" \
  -Dpybind11_DIR="$PYBIND11_DIR" \
  -DAUBO_ARCS_SDK_ROOT="$SDK_ROOT"
cmake --build "$AUBO_RUNTIME/build-servoj-probe" --target _aubo_servoj_probe --parallel 2
"$TELEOP_PYTHON" - "$AUBO_RUNTIME" "$SDK_ROOT" <<'PY'
import datetime, hashlib, json, pathlib, sys
runtime, sdk = pathlib.Path(sys.argv[1]).resolve(), pathlib.Path(sys.argv[2]).resolve()
matches = list((runtime/'build-servoj-probe/python').glob('_aubo_servoj_probe*.so'))
if len(matches) != 1: raise SystemExit('Probe binary missing or ambiguous')
module = matches[0]
library = sdk/'lib/libaubo_sdk.so'
manifest = dict(schema_version=1, python=sys.executable,
    created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    sdk_package_version='0.24.1-rc.3+318754d',
    sdk_root=str(sdk),sdk_library=str(library.resolve()),
    sdk_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
    module_path=str(module),module_sha256=hashlib.sha256(module.read_bytes()).hexdigest(),
    scope='standalone ARCS 0.24 read-only controller compatibility probe; no servo mode entry, target, stop or configuration writes')
(runtime/'servoj-probe-build.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('Standalone read-only ServoJoint probe built; no robot connection or motion executed.')
PY
