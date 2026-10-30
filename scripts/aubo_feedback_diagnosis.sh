#!/usr/bin/env bash
# One finite 25 Hz serial SDK session plus a capture started AFTER SDK login.
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"
AUBO_DIAG_ROOT=/data/VR_teleoperation_runtime
AUBO_DIAG_DIR="$AUBO_DIAG_ROOT/feedback-gap-$(date +%Y%m%d-%H%M%S)"
mkdir -m 700 "$AUBO_DIAG_DIR"
printf '%s\n' "$AUBO_DIAG_DIR" > "$AUBO_DIAG_ROOT/latest-feedback-diagnosis.txt"
printf 'Evidence directory: %s\n' "$AUBO_DIAG_DIR"
printf 'First: enter the COMPUTER sudo password for packet capture.\n'
printf 'Then: enter the ROBOT SDK password at its separate prompt.\n'
sudo -v

capture_pid=''
cleanup_capture() {
  # tcpdump drops to this user with -Z. Signal only our exact capture path.
  python3 - "$AUBO_DIAG_DIR/wire.pcap" <<'PY'
import os,pathlib,signal,sys
target=os.fsencode(sys.argv[1])
for entry in pathlib.Path('/proc').iterdir():
    if not entry.name.isdigit():continue
    try:
        if entry.stat().st_uid != os.getuid():continue
        args=(entry/'cmdline').read_bytes().split(b'\0')
        if args and args[0]==b'/usr/bin/tcpdump' and b'-w' in args and args[args.index(b'-w')+1]==target:
            os.kill(int(entry.name),signal.SIGINT)
    except (OSError,ValueError,IndexError):pass
PY
  if [[ -n "$capture_pid" ]]; then
    kill -TERM "$capture_pid" 2>/dev/null || true
    wait "$capture_pid" || true
  fi
}
trap cleanup_capture EXIT
(
  # A prepare record is flushed only after SDK login and subscription.
  # It therefore excludes outgoing login credentials from the capture.
  for ((attempt=0; attempt<480; attempt++)); do
    if [[ -f "$AUBO_DIAG_DIR/events.jsonl" ]] && \
       rg -q '"event": "prepare"' "$AUBO_DIAG_DIR/events.jsonl"; then
      printf 'Starting bounded Ethernet capture after SDK login.\n'
      exec sudo -n /usr/bin/timeout --signal=INT --kill-after=5 330 \
        /usr/bin/tcpdump -i enp3s0 -nn -s 0 -B 4096 -U -C 8 -W 4 \
        -Z "$(id -un)" -w "$AUBO_DIAG_DIR/wire.pcap" \
        'host 192.168.1.5 and tcp port 8899'
    fi
    sleep .25
  done
  printf 'Capture did not start: no SDK prepare record within 120 seconds.\n' >&2
  exit 1
) > "$AUBO_DIAG_DIR/capture.log" 2>&1 &
capture_pid=$!

set +e
bash scripts/aubo_quest_follow.sh \
  --execute-six-axis --command-mode follow --follow-hz 25 \
  --waypoint-feedback --serial-state-query \
  --host 192.168.1.5 --port 8899 --user aubo \
  --rotation world --duration 300 \
  --diagnostic-output "$AUBO_DIAG_DIR/events.jsonl" \
  2>&1 | tee "$AUBO_DIAG_DIR/session.log"
session_rc=${PIPESTATUS[0]}
set -e
cleanup_capture
capture_pid=''
printf '\nSession exit code: %s\nEvidence: %s\n' "$session_rc" "$AUBO_DIAG_DIR"
printf 'Capture result:\n'
tail -8 "$AUBO_DIAG_DIR/capture.log"
exit "$session_rc"
