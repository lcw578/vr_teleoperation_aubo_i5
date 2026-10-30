# Local AUBO port — 2026-10-12

Source: https://github.com/Dream-Machines-Robotics/vr-teleop-kit

Pinned commit: `8d37f3d6bd44c8ce646f7bc110ac98e802e0ab57`, Apache-2.0.
The complete small upstream repository is retained, including LICENSE,
examples, tools, web UI, relay, and optional LeRobot interfaces.

## Changes to upstream

- `core/quest_teleop.py`: moved the original `BiQuestTeleoperator` loop and
  configuration out of the mandatory LeRobot import. Same pose EMA, precision
  button, clutch, rest ramp, target mapper, IK call, haptic formulas and settings
  machinery. Added active-hand selection and injection of a compiled MuJoCo
  model. `lerobot/bi_quest_teleop.py` is the optional registered LeRobot facade.
- Added measured-state observation. Engage/recovery anchors and reach limits
  use measured FK; the IK warm start uses the last applied joint reference.
  Backend time drives the rest ramp and gripper feedback velocity. The native
  entry now uses direct kinematic preview by default: observations in that mode
  are the displayed virtual state, identified as `kinematic_preview` in WS/raw
  records, not measured physical feedback.
- Added an explicit HMD-yaw switch; AUBO entry disables it. The Quest HMD still
  supplies XR tracking and video display, not robot steering.
- Added independent binary Trigger operation for AUBO, bypassing the DK1
  gripper slider mirror. Existing continuous engaged-only behavior remains
  the default for the DK1 entry.
- Per-hand freshness handles missing right-hand poses; validation, input epoch,
  sequence, relative queue-age gates and bounded WS publication prevent old
  data being interpreted as fresh movement. The relative age estimate is not
  clock synchronization or proof of absolute one-way latency.
- `core/pose_mapping.py`: added `tool_local` rotation; the original `world`
  path and its pivot/reach-limit formulas remain unchanged. A proper fixed
  controller-local to tool-local rotation is used; AUBO initial choice Rx(pi)
  needs human grip/axis acceptance.
- `ik/decoupled_ik.py`: constructor accepts an already compiled model. The
  FK, tool-to-anchor target conversion, adaptive DLS sub-solves, Tikhonov
  bias, antipode gate, joint clipping and step-cap formulas are unchanged.
- AUBO velocity ceilings cannot be raised by the inherited web sliders.
  Mode switching is rejected while Grip/home control is active.
- `relay/server.py`: optional `/teleop-info`, bounded broadcast writes. The
  native session registers in-memory camera readers in the original
  camera registry; original WebRTC signaling, tracks and H.264 choice remain.
- `relay/web/client.js` and `index.html`: rotation selector/status, right-hand
  wrist calibration, calibration JSON export/import, reference-space epochs,
  empty-controller frames, WS reconnection and send-backlog guard. Pivot fit,
  readout correction, camera panels and XR rendering are the author's code.
  Added `recording_hud.js`, a tested AUBO recording-status HUD rendered as a
  head-locked WebXR canvas panel near the lower field of view in passthrough.
  It polls `/aubo/status` at 2 Hz and shows confirmed episode/save state,
  elapsed time, and stale-connection warnings.

## AUBO backend outside the upstream package

`teleop/mujoco_session.py` adds `tool0` and `j4_anchor` sites to the existing
AUBO model without changing geometry, inertia or camera mounting. The anchor
is the joint-4 origin expressed in foreArm_Link; the DK1 10 cm artificial
anchor extension is not copied. Default `--backend kinematic` displays the
author's step-capped joint-position output using `qpos` + `mj_forward`, matching
the upstream `pure_sim.py` / `viewer_client.py` workflow. It does not call
`mj_step` or run the added acceleration/reference shaping. RG rack coupling
uses the existing compiled joint-equality polynomial; AG95 display uses the
1:1 mimic joints from `dh_ag95_description/urdf/ag95_macro.xacro`. No physical
grasp response or actuator-force feedback is asserted in preview mode.
The native RG initial/home pose is the user-selected upright preview:
`[-1.137355, -0.4, 1.1707963267948966, 1.5707959803847207, 1.137355, 0]` rad.
It has an inclined upper arm, horizontal forearm, level tool and wrist camera
above, with about 168 degrees of wrist3 margin both ways. The TCP location
changes; mounting and original IK formulas do not. The compiled ready key,
home config and original solver rest bias use the same preset. ROS assets
remain historical rollback presets. Both rotation modes passed +/-90-degree
axial twist samples from this preset without detected model contacts.
The old "straight-arm retract" test actually demanded horizontal motion from
a vertically extended arm. It is retained as historical physics characterization,
with its original bias and a corrected name. A new kinematic test uses the current
bias and true inward (-Z) motion; the arm retracts, with about 1.3cm remaining
TCP position residual in that scenario. No solver/bias changes were made based
on the incorrectly classified test. See docs/mujoco_teleop.md for details.
Historical numerical parity fixtures retain their original bias.

Explicit `--backend physics` retains the previous experimental position-actuator
loop (`mj_step`, 1 ms physics / 200 Hz control), reference shaping and
following-error latch. A 5 mm held hand-step reproduced command reversals in
that shaping, so it is not the current acceptance baseline. Reference
acceleration 5 rad/s² is a project simulation setting, not an AUBO manufacturer
specification. Actuator gains and gravity compensation are not an identified
model of the real cabinet servo. Both backends reuse full-state camera
snapshots and opt-in raw recording; no additional control framework was added.

## Verification provenance

`tests/fixtures/vrkit_8d37f3_expected.json` was generated by the unmodified
pinned upstream mapper (200 cases) and solver (100 cases with the same
compiled AUBO model injected in place of the DK1 model builder). Random seeds
and the source commit are included. It lets regression tests run without the
ignored `_refs` checkout and detects changes to the original algorithm.

Full headset feel, natural local-axis alignment, real Quest dropout causes,
real AUBO motion and real gripper force have not been validated by these tests.
