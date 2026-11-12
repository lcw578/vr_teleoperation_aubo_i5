#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
AUBO_ACK_RUNTIME=$(realpath -e "${AUBO_RUNTIME_DIR:-/data/VR_teleoperation_runtime}")
allowed=0
for root in /data ${AUBO_ALLOWED_ROOTS:-}; do
  [[ -d $root ]] || continue
  root=$(realpath -e "$root")
  case "$AUBO_ACK_RUNTIME" in "$root"/*) allowed=1 ;; esac
done
[[ $allowed == 1 && -w $AUBO_ACK_RUNTIME ]] || {
  printf 'Writable runtime under /data or AUBO_ALLOWED_ROOTS required.\n' >&2; exit 2;
}
mkdir -p "$AUBO_ACK_RUNTIME/quickack"
gcc -std=c11 -O2 -Wall -Wextra -Werror -fPIC -shared \
  "$PROJECT_ROOT/teleop/aubo/native/quickack.c" -o "$AUBO_ACK_RUNTIME/quickack/libaubo_quickack.so" \
  -ldl -pthread
python3 - "$PROJECT_ROOT" "$AUBO_ACK_RUNTIME" <<'PY'
import hashlib,json,pathlib,subprocess,sys
project,runtime=map(pathlib.Path,sys.argv[1:])
source=project/'teleop/aubo/native/quickack.c'
library=runtime/'quickack/libaubo_quickack.so'
report=dict(schema='aubo_quickack_experiment_v1',source=str(source),
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),library=str(library),
    library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
    compiler=subprocess.check_output(['gcc','--version'],text=True).splitlines()[0],
    activation='Explicit LD_PRELOAD plus AUBO_QUICKACK=1; not added to normal launchers',
    scope='Exact IPv4 TCP peer; QUICKACK only; no SDK load or connection during build')
(runtime/'quickack/build.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
PY
