#!/usr/bin/env bash
# Separate stationary Get experiment. Never starts the teleoperation controller.
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
execute=0
check=0
mode=baseline
label=A1
duration=90
while (($#)); do
  case "$1" in
    --execute) execute=1; shift ;;
    --check) check=1; shift ;;
    --mode|--label|--duration)
      [[ $# -ge 2 ]] || { printf 'Missing argument: %s\n' "$1" >&2; exit 2; }
      case "$1" in
        --mode) mode=$2 ;;
        --label) label=$2 ;;
        --duration) duration=$2 ;;
      esac
      shift 2 ;;
    --help|-h)
      printf 'Usage: bash scripts/aubo_ack_stationary.sh [--check|--execute] [--mode baseline|quickack|nodelay|combined] [--label A1|B|A2|NODELAY|COMBINED] [--duration 30..120]\n'
      exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done
[[ $execute != 1 || $check != 1 ]] || { printf 'Choose --check or --execute.\n' >&2; exit 2; }
cd "$PROJECT_ROOT"
source scripts/teleop_env.sh
teleop_env
export AUBO_RUNTIME_DIR=${AUBO_RUNTIME_DIR:-$TELEOP_RUNTIME_ROOT}
args=(--mode "$mode" --label "$label" --duration "$duration")
# Always validate modes and print the file-free plan before doing anything else.
"$TELEOP_PYTHON" -m teleop.aubo.ack_probe "${args[@]}"
if [[ $execute != 1 && $check != 1 ]]; then
  printf 'Plan/check only: no SDK loading, login, sudo or TCP option changes.\n'
  exit 0
fi
for utility in sudo tcpdump timeout flock; do
  command -v "$utility" >/dev/null || { printf 'Missing %s\n' "$utility" >&2; exit 2; }
done
[[ -d /sys/class/net/eno1 ]] || { printf 'Vehicle cabinet interface eno1 absent.\n' >&2; exit 2; }
[[ -f "$AUBO_RUNTIME_DIR/readonly-build.json" ]] || { printf 'Read-only SDK build manifest absent.\n' >&2; exit 2; }
if [[ $check == 1 ]]; then
  printf 'Preflight OK: read-only manifest, eno1 and capture tools present. No SDK loading, login, sudo or option changes.\n'
  exit 0
fi
# Serialize this stationary launcher; the lock does not create SDK connections.
exec 9> "$AUBO_RUNTIME_DIR/ack-stationary.lock"
flock -n 9 || { printf 'Another stationary ACK test is running.\n' >&2; exit 2; }
# Do not disturb existing processes. Refuse overlapping controller experiments.
"$TELEOP_PYTHON" - <<'PY'
import os,pathlib
blocked=[]
modules={b'teleop.aubo.quest_follow', b'teleop.aubo.ack_probe', b'teleop.aubo.j6_trial',
         b'teleop.aubo.placement_teach', b'teleop.aubo.stop_probe'}
for path in pathlib.Path('/proc').iterdir():
    if not path.name.isdigit() or int(path.name)==os.getpid(): continue
    try:
        if path.stat().st_uid != os.getuid(): continue
        args=(path/'cmdline').read_bytes().split(b'\0')
        if modules.intersection(args): blocked.append(path.name)
    except OSError: pass
if blocked: raise SystemExit('Existing robot session(s): '+','.join(blocked)+'. End them normally first.')
PY
printf 'Keep robot stationary. This test uses Get calls only; no headset or camera.\n'
printf 'Enter COMPUTER sudo password for capture; SDK password follows.\n'
sudo -v
exec "$TELEOP_PYTHON" -m teleop.aubo.ack_probe "${args[@]}" --connect \
  --host 192.168.1.5 --port 8899 --user aubo --capture-interface eno1
