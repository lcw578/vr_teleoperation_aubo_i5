#!/usr/bin/env bash
# Explicit supervised field run: unchanged Follow25 + recording + post-login PCAP.
set -euo pipefail
trap 'field_rc=$?; printf "Field launcher failed at line %s (exit %s).\n" "$LINENO" "$field_rc" >&2' ERR
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
duration=120
relay_port=8443
execute=0
check_only=0
quickack=0
ack_mode=
placement_reference=
gripper=1
while (($#)); do
  case "$1" in
    --execute) execute=1; shift ;;
    --no-gripper) gripper=0; shift ;;
    --check) check_only=1; shift ;;
    --quickack) quickack=1; shift ;;
    --ack-mode) [[ $# -ge 2 ]] || { printf 'Missing ACK mode.\n' >&2; exit 2; }; ack_mode=$2; shift 2 ;;
    --placement-reference) [[ $# -ge 2 && -n $2 ]] || { printf 'Missing placement reference.\n' >&2; exit 2; }; placement_reference=$2; shift 2 ;;
    --duration) [[ $# -ge 2 ]] || { printf 'Missing duration value.\n' >&2; exit 2; }; duration=$2; shift 2 ;;
    --relay-port) [[ $# -ge 2 ]] || { printf 'Missing relay port.\n' >&2; exit 2; }; relay_port=$2; shift 2 ;;
    --help|-h) printf 'Usage: bash scripts/aubo_home_field.sh [--check | --execute] [--no-gripper] [--quickack] [--ack-mode baseline|quickack|nodelay|combined] [--relay-port 1..65535] [--duration 30..1800] [--placement-reference basket.json]\n'; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done
if [[ $execute == 1 && $check_only == 1 ]]; then
  printf 'Choose either --check or --execute.\n' >&2; exit 2
fi
if [[ ! $duration =~ ^[0-9]+$ ]] || ((${#duration}>4)) || ((10#$duration<30 || 10#$duration>1800)); then
  printf 'Duration must be an integer from 30 to 1800 seconds.\n' >&2; exit 2
fi
duration=$((10#$duration))
if [[ ! $relay_port =~ ^[0-9]+$ ]] || ((${#relay_port}>5)) || ((10#$relay_port<1 || 10#$relay_port>65535)); then
  printf 'Relay port must be an integer from 1 to 65535.\n' >&2; exit 2
fi
relay_port=$((10#$relay_port))
if [[ -n $ack_mode && ! $ack_mode =~ ^(baseline|quickack|nodelay|combined)$ ]]; then
  printf 'ACK mode must be baseline, quickack, nodelay or combined.\n' >&2; exit 2
fi
if [[ $execute != 1 && $check_only != 1 ]]; then
  printf 'Plan only: no files, sudo, SDK connection or movement.\n'
  printf 'Explicit execution requires a local operator and physical stop access.\n'
  printf 'Follow25 / serial / USB / oneshot; recording + diagnostics; capture after SDK prepare.\n'
  printf 'QUICKACK experiment selected: %s (0=baseline, 1=explicit opt-in).\n' "$quickack"
  printf 'Placement reference (explicit opt-in): %s\n' "${placement_reference:-disabled}"
  printf 'TCP option experiment: %s\n' "${ack_mode:-disabled}"
  printf 'Vehicle gripper: %s (right index Trigger press toggles; --no-gripper disables).\n' "$gripper"
  printf 'WebXR relay URL: http://localhost:%s/\n' "$relay_port"
  printf 'Set TELEOP_RUNTIME_ROOT, AUBO_RUNTIME_DIR and AUBO_ALLOWED_ROOTS before --execute.\n'
  exit 0
fi
if [[ $quickack == 1 ]]; then
  printf 'QUICKACK field experiment unavailable: SDK RTLD_DEEPBIND bypasses the LD_PRELOAD receive hooks.\n' >&2
  printf 'The 2026-11-02 runs matched zero receives; a validated SDK-compatible intervention is required before retrying.\n' >&2
  exit 2
fi
printf 'Checking vehicle field-run dependencies and runtime...\n'
AUBO_FIELD_ROOT=${AUBO_RUNTIME_DIR:-${TELEOP_RUNTIME_ROOT:-/data/VR_teleoperation_runtime}}
AUBO_FIELD_ROOT=$(realpath -e "$AUBO_FIELD_ROOT")
allowed=0
for root in /data ${AUBO_ALLOWED_ROOTS:-}; do
  [[ -d $root ]] || continue
  root=$(realpath -e "$root")
  case "$AUBO_FIELD_ROOT" in "$root"/*) allowed=1 ;; esac
done
if [[ $allowed != 1 || ! -w $AUBO_FIELD_ROOT ]]; then
  printf 'Runtime must be writable and resolve under /data or AUBO_ALLOWED_ROOTS.\n' >&2; exit 2
fi
export TELEOP_RUNTIME_ROOT=$AUBO_FIELD_ROOT AUBO_RUNTIME_DIR=$AUBO_FIELD_ROOT
cd "$PROJECT_ROOT"
for utility in sudo timeout tcpdump python3 grep adb; do
  if ! command -v "$utility" >/dev/null; then
    printf 'Missing required command: %s\n' "$utility" >&2; exit 2
  fi
done
if [[ ! -d /sys/class/net/eno1 ]]; then
  printf 'Expected vehicle cabinet interface eno1 is absent.\n' >&2; exit 2
fi
if [[ $quickack == 1 ]]; then
  AUBO_FIELD_QUICKACK_LIBRARY=$AUBO_FIELD_ROOT/quickack/libaubo_quickack.so
  python3 - "$AUBO_FIELD_ROOT" "$PROJECT_ROOT" <<'PY'
import hashlib,json,pathlib,sys
runtime,project=map(pathlib.Path,sys.argv[1:])
manifest=json.loads((runtime/'quickack/build.json').read_text())
library=(runtime/'quickack/libaubo_quickack.so').resolve(strict=True)
assert runtime in library.parents,'QUICKACK library escapes runtime'
assert hashlib.sha256(library.read_bytes()).hexdigest()==manifest['library_sha256'],'QUICKACK binary mismatch'
assert hashlib.sha256((project/'teleop/aubo/native/quickack.c').read_bytes()).hexdigest()==manifest['source_sha256'],'QUICKACK source mismatch; rebuild'
print('Explicit QUICKACK experiment: library and source hashes verified; target 192.168.1.5:8899 only.')
PY
fi
if [[ -n $placement_reference && ! -r $placement_reference ]]; then
  printf 'Placement reference is not readable: %s\n' "$placement_reference" >&2; exit 2
fi
printf 'Preflight OK: runtime=%s; interface=eno1; capture commands available.\n' "$AUBO_FIELD_ROOT"
if [[ $check_only == 1 ]]; then
  printf 'Check only: no sudo authentication, SDK connection or movement.\n'
  exit 0
fi
if [[ -z ${DISPLAY:-} && -z ${WAYLAND_DISPLAY:-} ]]; then
  printf 'No graphical desktop environment. Run from a terminal on the vehicle desktop.\n' >&2; exit 2
fi
authorized_devices=()
while read -r serial state rest; do
  if [[ $state == device ]]; then authorized_devices+=("$serial"); fi
done < <(adb devices)
if [[ ${#authorized_devices[@]} != 1 ]]; then
  printf 'Expected exactly one authorized USB headset; found %s. Connect it and accept USB debugging before retrying.\n' "${#authorized_devices[@]}" >&2
  exit 2
fi
adb -s "${authorized_devices[0]}" reverse "tcp:$relay_port" "tcp:$relay_port"
printf 'USB forwarding ready. Headset browser: http://localhost:%s/ then Start Teleop.\n' "$relay_port"
printf 'The webpage closes when this %s-second session finishes; it does not remain running.\n' "$duration"
printf 'Enter the COMPUTER sudo password for packet capture; SDK password follows separately.\n'
sudo -v
AUBO_FIELD_DIR=$(mktemp -d "$AUBO_FIELD_ROOT/home-field-$(date +%Y%m%d-%H%M%S)-XXXXXX")
printf '%s\n' "$AUBO_FIELD_DIR" > "$AUBO_FIELD_ROOT/latest-home-field.txt"
printf 'Evidence directory: %s\n' "$AUBO_FIELD_DIR"
capture_pid=''
cleanup_capture() {
  # Stop only this run's tcpdump, after -Z has dropped it to the operator UID.
  python3 - "$AUBO_FIELD_DIR/wire.pcap" <<'PY'
import os,pathlib,signal,sys
target=os.fsencode(sys.argv[1])
for entry in pathlib.Path('/proc').iterdir():
    if not entry.name.isdigit(): continue
    try:
        if entry.stat().st_uid != os.getuid(): continue
        args=(entry/'cmdline').read_bytes().split(b'\0')
        if (args and pathlib.Path(os.fsdecode(args[0])).name=='tcpdump'
                and b'-w' in args and args[args.index(b'-w')+1]==target):
            os.kill(int(entry.name),signal.SIGINT)
    except (OSError,ValueError,IndexError): pass
PY
  if [[ -n "$capture_pid" ]]; then
    kill -TERM "$capture_pid" 2>/dev/null || true
    wait "$capture_pid" 2>/dev/null || true
    capture_pid=''
  fi
}
trap cleanup_capture EXIT
(
  # The SDK flushes prepare only after authentication and subscription.
  # Capture must never include an outgoing SDK login. Do not reconnect mid-run.
  for ((attempt=0; attempt<1200; attempt++)); do
    if [[ -f "$AUBO_FIELD_DIR/events.jsonl" ]] &&
        grep -Eq '"event"[[:space:]]*:[[:space:]]*"prepare"' "$AUBO_FIELD_DIR/events.jsonl"; then
      printf 'Starting passive eno1 capture after SDK prepare.\n'
      exec sudo -n timeout --signal=INT --kill-after=5 "$((duration+15))" \
        tcpdump -i eno1 -nn -s 0 -B 4096 -U --immediate-mode -C 8 -Z "$(id -un)" \
        -w "$AUBO_FIELD_DIR/wire.pcap" 'host 192.168.1.5 and tcp port 8899'
    fi
    sleep .1
  done
  printf 'Capture failed: no prepare event within 120 seconds.\n' >&2
  exit 1
) > "$AUBO_FIELD_DIR/capture.log" 2>&1 &
capture_pid=$!
set +e
run_session() {
  if [[ $quickack == 1 ]]; then
    # Apply LD_PRELOAD only to the Python SDK process, never sudo/tcpdump.
    source "$PROJECT_ROOT/scripts/teleop_env.sh"
    teleop_env || return
    exec env LD_PRELOAD="$AUBO_FIELD_QUICKACK_LIBRARY" AUBO_QUICKACK=1 \
      AUBO_QUICKACK_HOST=192.168.1.5 AUBO_QUICKACK_PORT=8899 \
      "$TELEOP_PYTHON" -m teleop.aubo.quest_follow "$@"
  else
    exec bash scripts/aubo_quest_follow.sh "$@"
  fi
}
placement_args=()
ack_args=()
gripper_args=()
if [[ $gripper == 1 ]]; then gripper_args=(--gripper); fi
field_task='左Y基线取证；不做Placement或ACK干预'
if [[ -n $placement_reference ]]; then
  placement_args=(--placement-reference "$placement_reference")
  field_task='空载左X搬运验证：手动退出后经左Y中转位直接到篮筐点；无夹爪、不释放'
fi
if [[ -n $ack_mode ]]; then
  ack_args=(--tcp-ack-mode "$ack_mode")
  if [[ -n $placement_reference ]]; then
    field_task="Follow25 ACK链路对照：$ack_mode；左X空载放置已启用"
  else
    field_task="Follow25 ACK链路对照：$ack_mode；不做Placement"
  fi
fi
if [[ $gripper == 1 ]]; then field_task="$field_task；右食指按下切换实体夹爪开合"; fi
run_session "${placement_args[@]}" \
  "${gripper_args[@]}" \
  --execute-six-axis --command-mode follow --follow-hz 25 \
  --waypoint-feedback --serial-state-query --usb \
  --host 192.168.1.5 --port 8899 --user aubo \
  --relay-port "$relay_port" \
  --rotation world --home-trigger oneshot --duration "$duration" \
  --record --task "$field_task" \
  "${ack_args[@]}" \
  --diagnostic-output "$AUBO_FIELD_DIR/events.jsonl" \
  2>&1 | tee "$AUBO_FIELD_DIR/session.log"
session_rc=${PIPESTATUS[0]}
set -e
cleanup_capture
evidence_rc=0
python3 - "$AUBO_FIELD_DIR" "$session_rc" "$quickack" "$placement_reference" "$ack_mode" <<'PY' || evidence_rc=$?
import json,pathlib,sys
root=pathlib.Path(sys.argv[1])
pcaps=sorted(root.glob('wire.pcap*'))
capture_present=any(p.stat().st_size>24 for p in pcaps)
result=dict(schema='aubo_home_field_v1',session_exit_code=int(sys.argv[2]),
    pcap_data_present=capture_present,pcap_files=[p.name for p in pcaps],
    capture_validation='Check capture.log dropped counters and full time window; file presence is not completeness',
    parameters=dict(command_mode='follow',follow_hz=25,state_query='serial',usb=True,home_trigger='oneshot',ack_intervention=sys.argv[3]=='1' or bool(sys.argv[5] and sys.argv[5]!='baseline'),tcp_ack_mode=sys.argv[5] or None,placement_reference=sys.argv[4] or None))
(root/'field-result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False))
raise SystemExit(0 if capture_present else 3)
PY
printf 'Evidence: %s\n' "$AUBO_FIELD_DIR"
tail -8 "$AUBO_FIELD_DIR/capture.log"
if [[ $session_rc != 0 ]]; then exit "$session_rc"; fi
exit "$evidence_rc"
