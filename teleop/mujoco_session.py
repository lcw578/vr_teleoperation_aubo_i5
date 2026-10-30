"""AUBO vr-teleop-kit session: author-style kinematic preview by default. No ROS imports."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import fcntl
import json
import logging
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "third_party/vr-teleop-kit/src"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/aubo_i5_teleop/scripts"))

import mujoco
import numpy as np
import yaml
from gripper_profile import resolve
from vr_teleop_kit.core.quest_teleop import QuestTeleoperator, QuestTeleoperatorConfig

LOG = logging.getLogger("aubo_mujoco")
JOINTS = ("shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint")
SPEC = yaml.safe_load((ROOT / "ros2_ws/src/aubo_i5_teleop/config/robot_spec.yaml").read_text())
QUEST_TO_BASE = [[-1., 0., 0.], [0., 0., 1.], [0., 1., 0.]]
# User-selected upright preview: inclined upper arm, horizontal forearm,
# level forward tool, wrist camera above, wrist3 centered for bidirectional roll.
RG_READY = (-1.137355, -0.4, 1.1707963267948966, 1.5707959803847207, 1.137355, 0.)


def load_model(gripper="rg"):
    """Add FK sites and native ready pose; keep geometry and camera mounting."""
    profile = resolve(gripper)
    if gripper == "rg":
        # Keep the old ROS profile/assets available for historical rollback.
        profile["q_rest"] = RG_READY
    scene = ROOT / "assets/aubo_i5" / ("scene_ros2_rg_cameras.xml" if gripper == "rg" else "scene_ros2_demo.xml")
    profile["mjcf"] = str(scene)
    spec = mujoco.MjSpec.from_file(str(scene))
    spec.body(profile["ee_body"]).add_site(name="tool0", pos=profile["tip_offset"], quat=profile["tool_quat"])
    # Joint 4 origin in foreArm_Link, upstream of wrist1_joint. No DK1 10cm offset.
    spec.body("foreArm_Link").add_site(name="j4_anchor", pos=spec.body("wrist1_Link").pos)
    model = spec.compile()
    for j, name in enumerate(JOINTS):
        joint = model.joint(name)
        if joint.id != j or joint.qposadr[0] != j or joint.dofadr[0] != j:
            raise ValueError("upstream 3+3 solver requires the six arm joints first")
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "ready")
    model.key_qpos[key, :6] = profile["q_rest"]
    for name, angle in zip(JOINTS, profile["q_rest"]):
        model.key_ctrl[key, model.actuator(name).id] = angle
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    return model, data, profile, scene


class MuJoCoFollower:
    """Same author IK loop; direct preview or explicitly selected physics execution."""
    def __init__(self, gripper="rg", rotation_mode="world", speed=1., acceleration=5., frequency=200., backend="kinematic"):
        if backend not in ("kinematic", "physics"):
            raise ValueError("backend must be kinematic or physics")
        self.backend = backend
        self.model, self.data, self.profile, self.scene = load_model(gripper)
        self.dt = 1. / frequency
        self.substeps = round(self.dt / self.model.opt.timestep)
        if not np.isclose(self.substeps * self.model.opt.timestep, self.dt):
            raise ValueError("control period must be a multiple of the model timestep")
        self.vmax = np.asarray(SPEC["velocity_limits_rad_s"]) * speed
        self.amax = acceleration if backend == "physics" else None
        self.limit = SPEC["working_limit_rad"]
        cfg = QuestTeleoperatorConfig(
            id="aubo-i5-mujoco", active_hands=("right",), rotation_mode=rotation_mode,
            head_yaw_alignment=False, binary_gripper=True, r_calib=QUEST_TO_BASE,
            publish_ik_state=False,  # Publish after applying the selected backend state.
            rest_qpos_right=list(self.profile["q_rest"]), velocity_limits=self.vmax.tolist(),
            control_dt=self.dt, max_dq_per_joint=(self.vmax*self.dt).tolist(),
            max_dq_per_joint_scalar_pos=float(self.vmax[0]*self.dt),
            max_dq_per_joint_scalar_rot=float(self.vmax[3]*self.dt),
            force_haptic_enabled=False,
        )
        self.teleop = QuestTeleoperator(cfg, models={"right": self.model})
        self.teleop.state_source = "physics" if backend == "physics" else "kinematic_preview"
        self.arm = self.teleop._arms["right"]
        self.arm["solver"].joint_limits[:] = [-self.limit, self.limit]
        self.command = self.data.qpos[:6].copy()
        self.velocity = np.zeros(6)
        self.actuators = [self.model.actuator(name).id for name in JOINTS]
        self.gripper_actuator = self.model.actuator(self.profile["grasp_joint"]).id
        self.gripper_joint = self.model.joint(self.profile["grasp_joint"]).qposadr[0]
        self.gripper_command = self.profile["open"]
        self.following_duration = 0.
        self.fault = None
        self.teleop.observe("right", self.data.qpos, self.data.time)
        self.teleop.mode_switch_allowed = bool(not self.fault and np.max(np.abs(self.velocity)) < .02
                                              and np.max(np.abs(self.data.qvel[:6])) < .02)
        self.arm["qpos"] = self.data.qpos.copy()

    def _preview_gripper(self):
        """Display the existing mechanism; mj_forward does not enforce mimics."""
        self.gripper_command = self.profile["close"] if self.arm["trigger"] else self.profile["open"]
        self.data.qpos[self.gripper_joint] = self.gripper_command
        if self.profile["name"] == "rg":
            # Use the gear/rack polynomial already stored in the compiled MJCF.
            for i in range(self.model.neq):
                if self.model.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT and self.model.eq_obj2id[i] >= 0:
                    a, b = self.model.eq_obj1id[i], self.model.eq_obj2id[i]
                    qa, qb = self.model.jnt_qposadr[a], self.model.jnt_qposadr[b]
                    x = self.data.qpos[qb] - self.model.qpos0[qb]
                    self.data.qpos[qa] = self.model.qpos0[qa] + np.polynomial.polynomial.polyval(x, self.model.eq_data[i, :5])
        else:
            # DH AG95 URDF ag95_macro.xacro finger/inner/outer mimic joints: 1:1.
            for side in ("left", "right"):
                for part in ("inner_knuckle", "outer_knuckle", "finger"):
                    self.data.qpos[self.model.joint(f"{side}_{part}_joint").qposadr[0]] = self.gripper_command

    def _advance_preview(self, moving):
        previous = self.command.copy()
        if moving:
            # Exactly the already step-capped author IK output. No extra trajectory filter.
            self.command = self.arm["qpos"][:6].copy()
        if not np.all(np.isfinite(self.command)):
            raise RuntimeError("non-finite preview joint position")
        self.velocity = (self.command - previous) / self.dt
        self.data.qpos[:6] = self.command
        self.data.qvel[:] = 0.  # No simulated physical velocity or actuator feedback.
        self._preview_gripper()
        self.data.ctrl[self.actuators] = self.command
        self.data.ctrl[self.gripper_actuator] = self.gripper_command
        self.data.time += self.dt  # Preview sample clock, not integrated physics time.
        mujoco.mj_forward(self.model, self.data)
        self.arm["qpos"][:6] = self.command

    def tick(self):
        self.teleop.observe("right", self.data.qpos, self.data.time)
        measured_gripper = float(self.data.qpos[self.gripper_joint])
        normalized = (measured_gripper-self.profile["open"])/(self.profile["close"]-self.profile["open"])
        self.teleop.send_feedback({"torques": {
            "right_gripper.torque": float(self.data.actuator_force[self.gripper_actuator]) if self.backend == "physics" else 0.,
            "right_gripper.pos": float(np.clip(normalized,0,1)),
        }})
        if self.fault:
            # Keep receiving buttons/config, but require an explicit restart for faults.
            self.teleop.reset_input(self.fault)
        self.teleop.get_action()
        moving = not self.fault and not self.arm["paused"] and (self.arm["engaged"] or self.arm["ramp_active"])
        if self.backend == "kinematic":
            self._advance_preview(moving)
            self.teleop.observe("right", self.data.qpos, self.data.time)
            self.teleop.mode_switch_allowed = bool(not moving)
            self.teleop.publish_state(force=True)
            return
        if moving:
            desired = np.clip(self.arm["qpos"][:6], -self.limit, self.limit)
            v = np.clip((desired-self.command)/self.dt, -self.vmax, self.vmax)
            # Begin braking before command position bounds, not after hitting them.
            # v*dt + v^2/(2*a) <= remaining distance. Include this tick's
            # travel; the continuous sqrt(2*a*d) bound brakes one step late.
            adt = self.amax*self.dt
            v = np.clip(v, adt-np.sqrt(adt*adt+2*self.amax*np.maximum(self.command+self.limit, 0)),
                        np.sqrt(adt*adt+2*self.amax*np.maximum(self.limit-self.command, 0))-adt)
        else:
            v = np.zeros(6)
        self.velocity = np.clip(v, self.velocity-self.amax*self.dt, self.velocity+self.amax*self.dt)
        self.velocity = np.clip(self.velocity, (-self.limit-self.command)/self.dt, (self.limit-self.command)/self.dt)
        self.command += self.velocity*self.dt
        if not np.all(np.isfinite(self.command)):
            raise RuntimeError("non-finite position command")
        self.data.ctrl[self.actuators] = self.command
        self.arm["qpos"][:6] = self.command  # Warm start from the applied reference, not a queued IK plan.
        target = self.profile["close"] if self.arm["trigger"] else self.profile["open"]
        if not self.fault:
            self.gripper_command += np.clip(target-self.gripper_command, -1.5*self.dt, 1.5*self.dt)
        self.data.ctrl[self.gripper_actuator] = self.gripper_command
        mujoco.mj_step(self.model, self.data, nstep=self.substeps)
        err = np.abs(self.command-self.data.qpos[:6])
        self.following_duration = self.following_duration+self.dt if np.any(err > self.vmax*.2) else 0.
        if self.following_duration > .2 and self.fault is None:
            self.fault = "following_error"
            LOG.error("following_error: %s; restart after inspecting simulation", err.tolist())
        if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
            raise RuntimeError("non-finite physical state")
        self.teleop.observe("right", self.data.qpos, self.data.time)
        if self.fault:
            self.arm["paused"] = self.fault
        self.teleop.mode_switch_allowed = bool(not self.fault and np.max(np.abs(self.velocity)) < .02
                                              and np.max(np.abs(self.data.qvel[:6])) < .02)
        self.teleop.publish_state(force=True)

    def status(self):
        actual_p, actual_q = self.arm["solver"].fk(self.data.qpos)
        target = self.arm["target"]
        physical = self.backend == "physics"
        return dict(backend=self.backend, state_source="physics" if physical else "kinematic_preview",
                    sim_time=self.data.time, command_apply_sim_time=self.data.time-self.dt,
                    monotonic=time.monotonic(), rotation_mode=self.teleop.config.rotation_mode,
                    engaged=self.arm["engaged"], paused=self.arm["paused"], fault=self.fault,
                    q_measured=self.data.qpos[:6].tolist() if physical else None,
                    q_velocity=self.data.qvel[:6].tolist() if physical else None,
                    q_simulated=self.data.qpos[:6].tolist(),
                    physics_qpos=self.data.qpos.tolist() if physical else None,
                    physics_qvel=self.data.qvel.tolist() if physical else None,
                    q_commanded=self.command.tolist(), command_velocity=self.velocity.tolist(),
                    effective_translation_scale=self.arm["mapper"].scale,
                    effective_rotation_scale=self.arm["mapper"].scale_rotation,
                    pose_filter_alpha=self.teleop.config.pose_filter_alpha,
                    ee_position=actual_p.tolist(), ee_quaternion_wxyz=actual_q.tolist(),
                    target_position=target[0].tolist() if target else None,
                    target_quaternion_wxyz=target[1].tolist() if target else None,
                    gripper_closed_command=bool(self.arm["trigger"]), gripper_command=self.gripper_command,
                    gripper_measured=float(self.data.qpos[self.gripper_joint]) if physical else None,
                    gripper_displayed=float(self.data.qpos[self.gripper_joint]),
                    gripper_simulated_actuator_force=float(self.data.actuator_force[self.gripper_actuator]) if physical else None,
                    contacts=int(self.data.ncon), input_stats=dict(self.teleop.input_stats),
                    input=self.teleop._latest_xr_frame)


class Recorder:
    """Opt-in raw records; camera snapshot times are kept, never fabricated."""
    def __init__(self, path, manifest, images=False):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path/"manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n")
        self.file = (self.path/"signals.jsonl").open("w")
        self.images = images
        self.lock = threading.Lock()

    def write(self, row):
        with self.lock:
            self.file.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+"\n")

    def image(self, camera, frame, sim_time, frame_id):
        if not self.images:
            return
        import cv2
        filename = f"{camera}_{frame_id:08d}.jpg"
        if not cv2.imwrite(str(self.path/filename), frame):
            raise RuntimeError("image write failed")
        self.write(dict(type="image", camera=camera, frame_id=frame_id, sim_time=sim_time,
                        receive_monotonic=time.monotonic(), filename=filename))

    def close(self):
        self.file.close()


def recording_path(requested):
    """New recordings live on /data; existing project data is never moved."""
    if requested == "auto":
        runtime = Path(os.environ.get("AUBO_RUNTIME_DIR", "/data/VR_teleoperation_runtime")).resolve()
        if not runtime.is_relative_to(Path("/data")) or not runtime.is_dir() or not os.access(runtime, os.W_OK):
            raise ValueError("Recording requires an existing writable runtime directory under /data")
        path = (runtime / "recordings/mujoco" / time.strftime("%Y%m%d-%H%M%S")).resolve()
    else:
        path = Path(requested).expanduser().resolve()
    if not path.is_relative_to(Path("/data")):
        raise ValueError("New recordings must be stored under /data")
    return path


class SimCameras:
    """Full-state snapshots rendered once and reused by upstream WebRTC tracks."""
    def __init__(self, follower, relay, recorder=None):
        self.follower, self.recorder, self.relay = follower, recorder, relay
        self.state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
        self.lock, self.stop_event = threading.Lock(), threading.Event()
        self.snapshot = None
        self.error = None
        self.slots = {}
        specs = [("top", "D435 right", "d435_right", 640, 480),
                 ("left_wrist", "D435 left", "d435_left", 640, 480),
                 ("right_wrist", "D405 wrist", "cam_wrist_d405" if follower.profile["name"] == "rg" else "d405_wrist", 1280, 720)]
        relay.CAMERA_SPECS.clear()
        relay.CAMERA_READERS.clear()
        owner = self
        class Slot:
            def __init__(self, spec):
                self.spec, self.frame = spec, None
            def latest(self):
                return self.frame
            def stop(self):
                owner.stop_event.set()
        for cid, label, name, w, h in specs:
            spec = relay.CameraSpec(cid, label, "mujoco", w, h, 30, 0)
            slot = Slot(spec)
            self.slots[cid] = (slot, name)
            relay.CAMERA_SPECS.append(spec)
            relay.CAMERA_READERS[cid] = slot
        self.thread = threading.Thread(target=self._loop, name="mujoco-cameras", daemon=True)
        self.publish()
        self.thread.start()

    def publish(self):
        state = np.empty(mujoco.mj_stateSize(self.follower.model, self.state_spec))
        mujoco.mj_getState(self.follower.model, self.follower.data, state, self.state_spec)
        with self.lock:
            self.snapshot = state

    def _loop(self):
        import cv2
        model = self.follower.model
        data = mujoco.MjData(model)
        renderers = {}
        frame_id = 0
        try:
            for cid, (slot, _) in self.slots.items():
                renderers[cid] = mujoco.Renderer(model, height=slot.spec.height, width=slot.spec.width)
            while not self.stop_event.is_set():
                start = time.monotonic()
                with self.lock:
                    snapshot = self.snapshot
                mujoco.mj_setState(model, data, snapshot, self.state_spec)
                mujoco.mj_forward(model, data)
                # Preserve the AG95 legacy wrist camera's mounting definition.
                if self.follower.profile["name"] == "ag95":
                    bid = model.body("ag95_base").id
                    R, p = data.xmat[bid].reshape(3, 3), data.xpos[bid]
                    pos = p + R @ np.array([0., -.075, .10])
                    forward = p + R @ np.array([0., -.01, .15]) - pos
                    forward /= np.linalg.norm(forward)
                    x = np.cross(forward, R @ np.array([0., -1., 0.])); x /= np.linalg.norm(x)
                    cid = model.camera("d405_wrist").id
                    data.cam_xpos[cid] = pos
                    data.cam_xmat[cid] = np.column_stack((x, np.cross(x, forward), -forward)).ravel()
                for cid, (slot, name) in self.slots.items():
                    renderer = renderers[cid]
                    renderer.update_scene(data, camera=name)
                    slot.frame = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
                    if self.recorder:
                        self.recorder.image(cid, slot.frame, data.time, frame_id)
                frame_id += 1
                self.stop_event.wait(max(0, 1/30-(time.monotonic()-start)))
        except Exception as exc:
            self.error = exc
            LOG.exception("camera rendering failed")
        finally:
            for renderer in renderers.values():
                renderer.close()

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=5.)
        if self.thread.is_alive():
            raise RuntimeError("camera thread did not stop")


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--backend", choices=("kinematic", "physics"), default="kinematic",
                    help="kinematic: author-style direct IK preview (default); physics: experimental position actuators")
    ap.add_argument("--rotation", choices=("world", "tool_local"), default="world")
    ap.add_argument("--gripper", choices=("rg", "ag95"), default="rg")
    ap.add_argument("--headless", action="store_true", help="hide desktop Viewer")
    ap.add_argument("--no-cameras", action="store_true")
    ap.add_argument("--usb", action="store_true", help="HTTP localhost; adb reverse tcp:8443 tcp:8443")
    ap.add_argument("--port", type=int, default=8443)
    ap.add_argument("--speed", type=float, default=1., help="fraction of sourced joint velocity ceilings, 0 < value <= 1")
    ap.add_argument("--acceleration", type=float, help="physics backend only; reference acceleration rad/s^2 (default 5), NOT a manufacturer value")
    ap.add_argument("--record", nargs="?", const="auto", help="opt-in raw JSONL recording directory")
    ap.add_argument("--record-images", action="store_true", help="JPEG images, requires --record and cameras")
    ap.add_argument("--duration", type=float, default=0., help="optional bounded smoke run in wall seconds")
    args = ap.parse_args(argv)
    if args.acceleration is not None and args.backend != "physics":
        ap.error("--acceleration applies only to --backend physics; kinematic preview has no acceleration shaping")
    if args.backend == "physics" and args.acceleration is None:
        args.acceleration = 5.
    if not 0 < args.speed <= 1 or (args.acceleration is not None and (not np.isfinite(args.acceleration) or args.acceleration <= 0)):
        ap.error("speed must be in (0,1]; acceleration must be positive and finite")
    if args.record_images and (not args.record or args.no_cameras):
        ap.error("--record-images requires --record and cameras")
    if not np.isfinite(args.duration) or args.duration < 0 or not 0 < args.port <= 65535:
        ap.error("duration must be finite and nonnegative; port must be in 1..65535")
    return args


def main(argv=None):
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    # Same lock as the ROS supervisor. Own only resources created here.
    with ExitStack() as stack:
        lock = stack.enter_context((ROOT/".teleop_session.lock").open("a"))
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("another teleop session is running; close that session first")
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(("127.0.0.1", args.port))
            except OSError as exc:
                raise RuntimeError(f"relay port {args.port} is already in use; stop the existing relay") from exc
        follower = MuJoCoFollower(args.gripper, args.rotation, args.speed, args.acceleration, backend=args.backend)
        # Disable expensive shadows for the side-by-side Viewer and camera renderers.
        follower.model.vis.quality.shadowsize = 0
        follower.model.vis.global_.offwidth = 1280
        follower.model.vis.global_.offheight = 720
        manifest = dict(upstream=json.loads((ROOT/"third_party/vr-teleop-kit/UPSTREAM.json").read_text()),
                        scene=str(follower.scene), profile=follower.profile, model_spec=SPEC,
                        configuration=vars(args), control_dt=follower.dt,
                        backend=args.backend, state_source="physics" if args.backend == "physics" else "kinematic_preview",
                        physics_dt=follower.model.opt.timestep if args.backend == "physics" else None,
                        rotation_mode=args.rotation, head_yaw_alignment=False,
                        timestamp_notes="sim_time is the preview sample clock for kinematic, integrated snapshot time for physics; monotonic is PC clock; t_client is Quest clock")
        manifest["joints"] = [dict(name=follower.model.joint(i).name,
                                   qpos_address=int(follower.model.jnt_qposadr[i]),
                                   velocity_address=int(follower.model.jnt_dofadr[i]))
                              for i in range(follower.model.njnt)]
        manifest["cameras"] = [dict(name=follower.model.camera(i).name,
                                    body=follower.model.body(int(follower.model.cam_bodyid[i])).name,
                                    local_position=follower.model.cam_pos[i].tolist(),
                                    local_quaternion_wxyz=follower.model.cam_quat[i].tolist(),
                                    fovy_degrees=float(follower.model.cam_fovy[i]))
                               for i in range(follower.model.ncam)]
        recorder = None
        if args.record:
            path = recording_path(args.record)
            recorder = Recorder(path, manifest, args.record_images)
            stack.callback(recorder.close)
            LOG.info("Recording enabled: %s (images=%s)", path, args.record_images)
        else:
            LOG.info("Recording OFF; no state/image data will be written")
        from vr_teleop_kit.relay import server as relay
        import uvicorn
        relay.app.state.teleop_info = dict(backend=args.backend, rotation_mode=args.rotation, active_hands=["right"],
                                           head_yaw_alignment=False, velocity_caps=(follower.vmax*follower.dt).tolist())
        if not args.no_cameras:
            cameras = SimCameras(follower, relay, recorder)
            stack.callback(cameras.close)
        else:
            cameras = None
            relay.CAMERA_SPECS.clear(); relay.CAMERA_READERS.clear()
        tls = {} if args.usb else dict(ssl_keyfile=str(ROOT/"certs/key.pem"), ssl_certfile=str(ROOT/"certs/cert.pem"))
        for name in tls.values():
            if not Path(name).exists():
                raise RuntimeError("LAN TLS certificate missing; use existing certificate setup or --usb")
        server = uvicorn.Server(uvicorn.Config(relay.app, host="127.0.0.1" if args.usb else "0.0.0.0", port=args.port, log_level="warning", **tls))
        server_thread = threading.Thread(target=server.run, name="vr-teleop-relay", daemon=True)
        def stop_server():
            server.should_exit = True
            server_thread.join(timeout=5.)
        stack.callback(stop_server)
        server_thread.start()
        deadline = time.monotonic()+5.
        while not server.started:
            if not server_thread.is_alive() or time.monotonic() > deadline:
                raise RuntimeError("relay failed to start")
            time.sleep(.01)
        follower.teleop.config.ws_url = f"{'ws' if args.usb else 'wss'}://127.0.0.1:{args.port}/ws"
        stack.callback(follower.teleop.disconnect)
        follower.teleop.connect()
        viewer = None
        if not args.headless:
            from mujoco import viewer as viewer_module
            viewer = stack.enter_context(viewer_module.launch_passive(follower.model, follower.data))
        LOG.info("AUBO teleop ready: backend=%s rotation=%s; HMD mapping OFF; Trigger binary independent; ROS not required", args.backend, args.rotation)
        if args.backend == "kinematic":
            LOG.info("Direct author IK preview: mj_forward only; no mj_step, acceleration shaping, or physical grasp response")
        else:
            LOG.warning("Experimental physics backend: reference shaping has reproduced transient command oscillation; not the acceptance baseline")
        if args.usb:
            LOG.info("Quest: http://localhost:%d/ ; adb reverse tcp:%d tcp:%d", args.port, args.port, args.port)
        else:
            addresses = subprocess.check_output(["hostname", "-I"], text=True).split()
            LOG.info("Quest: %s ; choose the PC address on the headset's hotspot/LAN",
                     " ".join(f"https://{address}:{args.port}/" for address in addresses))
        start, next_tick = time.monotonic(), time.monotonic()
        last_status, last_view = start, start
        try:
            while viewer is None or viewer.is_running():
                follower.tick()
                if cameras:
                    cameras.publish()
                    if cameras.error:
                        raise RuntimeError("camera worker failed") from cameras.error
                if recorder:
                    recorder.write(dict(type="control", **follower.status()))
                now = time.monotonic()
                if viewer and now-last_view >= 1/30:
                    viewer.sync(); last_view = now
                if now-last_status >= 5.:
                    LOG.info("mode=%s paused=%s fault=%s max_following=%.4f rad input=%s realtime=%.2f",
                             follower.teleop.config.rotation_mode, follower.arm["paused"], follower.fault,
                             np.max(np.abs(follower.command-follower.data.qpos[:6])), follower.teleop.input_stats,
                             follower.data.time/(now-start))
                    last_status = now
                if args.duration and now-start >= args.duration:
                    break
                next_tick += follower.dt
                if now-next_tick > .2:
                    follower.teleop.reset_input("control_overrun")
                    LOG.warning("control overrun %.3fs; paused and reanchor required", now-next_tick)
                    next_tick = now
                time.sleep(max(0, next_tick-time.monotonic()))
        except KeyboardInterrupt:
            LOG.info("Stopping this MuJoCo session")
        LOG.info("Session closed: sim_time=%.3f input=%s", follower.data.time, follower.teleop.input_stats)


if __name__ == "__main__":
    main()
