#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
AUBO_RUNTIME=${AUBO_RUNTIME_DIR:-/data/VR_teleoperation_runtime}
source "$PROJECT_ROOT/scripts/teleop_env.sh"
teleop_env
AUBO_PYTHON=$TELEOP_PYTHON
if [[ ! -d "$AUBO_RUNTIME" || ! -w "$AUBO_RUNTIME" ]]; then
  echo "A writable runtime directory is required (AUBO_RUNTIME_DIR or /data/VR_teleoperation_runtime)" >&2; exit 1
fi
_AUBO_RUNTIME_RP=$(realpath "$AUBO_RUNTIME")
_AUBO_ROOT_OK=0
for _root in /data ${AUBO_ALLOWED_ROOTS:-}; do
  case "$_AUBO_RUNTIME_RP" in "${_root%/}"/*) _AUBO_ROOT_OK=1;; esac
done
if [[ $_AUBO_ROOT_OK != 1 ]]; then
  echo "Runtime directory must resolve under /data or AUBO_ALLOWED_ROOTS" >&2; exit 1
fi
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
cmake --build "$AUBO_RUNTIME/build-follow" --target _aubo_follow _aubo_follow_fake test_follow_profile test_passthrough test_stop_order_sdk --parallel 2
"$AUBO_RUNTIME/build-follow/test_follow_profile"
"$AUBO_RUNTIME/build-follow/test_passthrough"
"$AUBO_PYTHON" - "$AUBO_RUNTIME" <<'PY'
import hashlib,json,pathlib,sys
r=pathlib.Path(sys.argv[1]).resolve()
m=json.loads((r/'readonly-build.json').read_text())
modules=list((r/'build-follow/python').glob('_aubo_follow*.so'));assert len(modules)==1
out={k:m[k] for k in ('release_library','release_sha256')}
out.update(module_path=str(modules[0]),module_sha256=hashlib.sha256(modules[0].read_bytes()).hexdigest(),
  stop_request_policy='slow_stop_ack_then_fresh_feedback',
  priming=dict(max_s=2.5, age_limit_s=.4, min_steps=12, fast_frames=5, fast_s=.04),
  active_feedback_max_age_s=.2,
  hot_clutch=dict(holding_idle_s=20.,cold_cooldown_s=.5,state_max_age_s=.5,joint_max_age_s=.5,
    active_state_query_period_s=.2,idle_state_query_period_s=.05,warning_age_s=.2,send_max_s=.5,slow_send_s=.25,slow_send_count=3,
    resume_state_max_age_s=.25,resume_joint_max_age_s=.12,resume_fast_frames=5,resume_fast_s=.04),
  fixed_tick_batch_candidate=dict(tick_s=.005,max_points=4,max_pending_points=8,max_horizon_s=.04,
    wall_clock_pacing_supported=True,cushion_points=8,minimum_send_points=4,maximum_catchup_points=8,
    raw_items_per_point=6,queue_observation_required=True,reservation_max_s=.05,default_enabled=False,
    execution_quality_qualified=False,queue_freshness_certified=False,
    consumption_period_status="5ms experimental candidate; stationary watermark units verified, timing not yet certified"),
  trajectory=dict(velocity_deg_s=10., acceleration_deg_s2=30., jerk_deg_s3=300., max_dt_s=.05),
  refined_stream_candidate=dict(default_enabled=False,execution_quality_qualified=False,
    cushion_points=12,minimum_send_points=2,maximum_batch_points=8,
    state_rpc_paused_in=['priming','active','braking','holding'],
    state_rpc_resume='after_exit_and_slow_stop_ack',joint_age_max_s=.5,
    underrun_recovery='net_debt_and_gap_over_60ms_then_immediate_zero_reference_small_batches',
    stationary_debt='truncate_due_over4_to2_no_recovery',hot_resume='braking_or_holding_immediate_preserve_qva',
    recovery_initial_points=2,recovery_warmup_max_points=4,immediate_hot_clutch=True,
    supervised_c1_trial_ready=False),
  feedback_connection_modes=['A_send_B_push_and_state', 'A_send_B_state_C_push'],
  motion_optional_pushes_disabled=['RoadPoint','JointStatus','EndSpeed'],
  scope='supervised J6 or explicit six-axis: six-axis passthrough 200Hz or follow default 20Hz / explicit 25Hz, 50Hz or 200Hz requested; independent spawned read-only state SDK process for six-axis control with B waypoint push or explicit separate C waypoint owner, A native state/joint cache validation and all optional motion telemetry pushes disabled; private state request pipe plus bounded nonblocking joint datagrams preserve original reception timestamps, sequence and session generation; old in-process observer retained for compatibility; passthrough B query 200ms in ACTIVE/BRAKING/HOLDING and 50ms otherwise; B query period measured start-to-start, single inflight, no catch-up; Follow retains 200ms state/joint reception age; passthrough uses State 500ms / Joint 500ms hard limits and three consecutive A calls above 250ms revoke motion without killing healthy queries, no reconnect; streaming diagnostics retain 4096 history events plus bounded queue; passthrough default with enter/send/leave lifecycle, legacy J6 10Hz, 500ms SDK budget, 650ms gap budget, optional waypoint push, controller-reported full range; six-axis Follow direct IK; passthrough stationary native-enforced priming then graded ACTIVE/BRAKING/HOLDING freshness and actual-dt bounded quintic references with velocity acceleration jerk limits and no catchup; no SDK profile writes/reset or unreliable SDK profile readback; explicit profile inspection remains separate; legacy J6 retains explicit 10deg/s and 5deg/s2 and step checks; serial motion/stop, command-feedback differences diagnostic only; no startup/home; build capabilities are not live acceptance')
(r/'follow-build.json').write_text(json.dumps(out,indent=2)+'\n')
print('Supervised follower built. No SDK connection or movement executed.')
PY
