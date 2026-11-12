#!/usr/bin/env bash
# One explicit QUICKACK + FIFO90 comparison, restoring the host route on exit.
set -euo pipefail
AUBO_TEST_RUNTIME=${AUBO_RUNTIME_DIR:-/data/VR_teleoperation_runtime}
AUBO_TEST_ROOT="$AUBO_TEST_RUNTIME/tcp2can-minimal"
AUBO_TEST_BINARY="$AUBO_TEST_ROOT/tcp2can_minimal"
[[ -x "$AUBO_TEST_BINARY" ]] || { echo 'Compile tcp2can_minimal first.' >&2; exit 1; }
if [[ -n $(ss -Htn state established dst 192.168.1.5:8899) ]]; then
  echo 'Existing controller connection; test not started.' >&2; exit 1
fi
if [[ -n $(ip route show table main exact 192.168.1.5/32) ]]; then
  echo 'Existing controller host route; refusing to overwrite it.' >&2; exit 1
fi
# Authenticate before making any change. The password stays in sudo's prompt.
sudo -v
AUBO_TEST_STAMP=$(TZ=Asia/Shanghai date +%Y%m%d-%H%M%S)
AUBO_TEST_LOG="$AUBO_TEST_ROOT/quickack-fifo90-$AUBO_TEST_STAMP.log"
AUBO_TEST_CSV="$AUBO_TEST_ROOT/quickack-fifo90-$AUBO_TEST_STAMP.csv"
AUBO_TEST_ROUTE_ADDED=0
cleanup() {
  local result=$?
  trap - EXIT
  if [[ $AUBO_TEST_ROUTE_ADDED == 1 ]]; then
    if sudo -n ip route del 192.168.1.5/32 dev eno1; then
      printf 'temporary_host_route_removed=1\n' | tee -a "$AUBO_TEST_LOG"
    else
      printf 'Route cleanup failed: sudo ip route del 192.168.1.5/32 dev eno1\n' >&2
      result=1
    fi
  fi
  if [[ -f "$AUBO_TEST_CSV" ]]; then
    sudo -n chown "$(id -u):$(id -g)" "$AUBO_TEST_CSV" || result=1
  fi
  exit "$result"
}
trap cleanup EXIT
printf 'mode=route_quickack_and_fifo90\n' | tee "$AUBO_TEST_LOG"
ip route get 192.168.1.5 | tee -a "$AUBO_TEST_LOG"
sudo -n ip route add 192.168.1.5/32 dev eno1 src 192.168.1.100 quickack 1
AUBO_TEST_ROUTE_ADDED=1
ip route show table main exact 192.168.1.5/32 | tee -a "$AUBO_TEST_LOG"
# Same executable and 1000-send loop; no memory-locking claim or SDK changes.
sudo -n chrt -f 90 "$AUBO_TEST_BINARY" --execute --timing-csv "$AUBO_TEST_CSV" 2>&1 | tee -a "$AUBO_TEST_LOG"
