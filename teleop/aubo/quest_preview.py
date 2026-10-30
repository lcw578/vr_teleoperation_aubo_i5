"""Quest + live AUBO feedback preview. The native client exports NO motion API."""
import argparse
import asyncio
from contextlib import ExitStack
import fcntl
import getpass
import json
import logging
import math
from pathlib import Path
import socket
import threading
import time

from .feedback import joints
from .probe import json_safe, record_ok
from .readonly import ReadOnlyFollower, load_native

LOG = logging.getLogger("aubo_quest_preview")
ROOT = Path(__file__).resolve().parents[2]


class QuestReceiver:
    """Thin relay transport; all core mutations stay on the control owner.

    Uses the author's WebXR protocol and local TLS policy. Browser slider
    configuration is ignored for this commissioning preview; no home/haptics.
    """
    def __init__(self, runtime, url):
        self.runtime, self.url = runtime, url
        self.stop = threading.Event()
        self.connected = threading.Event()
        self.thread = threading.Thread(target=self._thread_main, name="quest-preview-input", daemon=True)

    def start(self):
        self.thread.start()

    def _thread_main(self):
        asyncio.run(self._run())

    async def _run(self):
        import websockets
        from vr_teleop_kit.core.quest_teleop import QuestTeleoperator
        ssl_context = QuestTeleoperator._ssl_context_for(self, self.url)
        while not self.stop.is_set():
            try:
                async with websockets.connect(self.url, ssl=ssl_context, max_queue=1,
                        open_timeout=3., close_timeout=1.) as ws:
                    self.connected.set()
                    while not self.stop.is_set():
                        try:
                            raw = await asyncio.wait_for(ws.recv(), timeout=.1)
                        except asyncio.TimeoutError:
                            continue
                        try:
                            message = json.loads(raw)
                            if not isinstance(message, dict):
                                raise ValueError("Non-object relay message")
                        except (ValueError, TypeError):
                            self.runtime.submit_fault("invalid_relay_message")
                            continue
                        if message.get("type") == "xr_frame":
                            self.runtime.submit_xr_frame(message, received_monotonic_s=time.monotonic())
            except Exception as exc:
                if not self.stop.is_set():
                    LOG.warning("Quest relay disconnected: %s", type(exc).__name__)
            finally:
                self.connected.clear()
                self.runtime.submit_fault("websocket_disconnected")
            # Reconnecting the transport never authorizes engagement.
            for _ in range(10):
                if self.stop.is_set():
                    return
                await asyncio.sleep(.1)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=6.)
        if self.thread.is_alive():
            raise RuntimeError("Quest transport did not exit")


class FeedbackPoller:
    """One serial SDK query worker, independent of IK. Get-only, no retry."""
    def __init__(self, follower, runtime, period_s=.05):
        self.follower, self.runtime, self.period_s = follower, runtime, period_s
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name="aubo-preview-feedback", daemon=True)
        self._lock = threading.Lock()
        self._status = {"error": None, "queries": 0}

    def start(self):
        self.thread.start()

    def _run(self):
        try:
            while not self.stop.is_set():
                started = time.monotonic()
                q = self.follower.query("joint_angles")
                state = self.follower.query("robot_state")
                elapsed = time.monotonic() - started
                with self._lock:
                    self._status.update(queries=self._status["queries"] + 2,
                        last_joint_result=q, last_state_result=state, query_pair_elapsed_s=elapsed)
                if not record_ok(q) or not record_ok(state) or state["data"] != 0:
                    raise RuntimeError("Feedback failed or controller is no longer stopped")
                if elapsed > .2:
                    raise RuntimeError("Read-only query pair exceeded 200 ms experimental budget")
                self.stop.wait(max(0., self.period_s - (time.monotonic() - started)))
        except Exception as exc:
            self.follower.cache.invalidate("preview_feedback_failed")
            with self._lock:
                self._status["error"] = {"type": type(exc).__name__, "message": str(exc)}
            self.runtime.submit_fault("preview_feedback_failed")

    def status(self):
        with self._lock:
            return dict(self._status)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=6.)
        if self.thread.is_alive():
            # Do not race logout against a blocked query.
            raise RuntimeError("SDK query still in flight; logout cannot run concurrently")


class PreviewStatus:
    def __init__(self):
        self.lock = threading.Lock()
        self.value = {"mode": "initializing", "motion_available": False}

    def update(self, value):
        invalid = []
        value = json_safe(value, invalid=invalid)
        if invalid:
            value["nonfinite_status_fields"] = invalid
        with self.lock:
            self.value = value

    def read(self):
        with self.lock:
            return dict(self.value)


def preview_velocity_limits(preview_speed_deg_s=None):
    """Use the accepted simulation ceiling unless an explicit preview cap is requested."""
    if preview_speed_deg_s is None:
        from teleop.mujoco_session import SPEC
        return tuple(SPEC["velocity_limits_rad_s"])
    if not math.isfinite(preview_speed_deg_s) or not 0 < preview_speed_deg_s <= 180.:
        raise ValueError("Preview speed must be finite and in (0, 180] deg/s")
    return (math.radians(preview_speed_deg_s),) * 6


def make_shadow(q, ranges, *, rotation="world", frequency=200., preview_speed_deg_s=None, quest_to_base=None,
                input_timeout_s=None):
    import numpy as np
    from .model import AuboModel
    from .offline_session import SessionPolicy
    from .shadow import AuboShadowCoordinator
    q = joints(q)
    velocities = preview_velocity_limits(preview_speed_deg_s)
    bounds = np.asarray(ranges, float)
    if bounds.shape != (6, 2) or not np.all(np.isfinite(bounds)) or np.any(bounds[:, 0] >= bounds[:, 1]):
        raise ValueError("Controller must return six ordered finite joint ranges")
    if np.any(np.asarray(q) < bounds[:, 0]) or np.any(np.asarray(q) > bounds[:, 1]):
        raise ValueError("Current joints outside controller-reported range")
    policy = SessionPolicy(tuple(bounds[:, 0]), tuple(bounds[:, 1]), velocities,
                           feedback_max_age_s=.25, input_max_age_s=.25 if input_timeout_s is None else input_timeout_s,
                           command_max_age_s=.1)
    shadow = AuboShadowCoordinator(policy, control_dt=1./frequency, clock=time.monotonic,
        rotation_mode=rotation, geometry=AuboModel(tool_frame="flange"), rest_q=q, quest_to_base=quest_to_base,
        core_input_timeout_s=input_timeout_s)
    arm = shadow.core._arms["right"]
    arm["qpos"][:] = shadow.geometry.qpos(q)
    shadow.core.observe("right", arm["qpos"])
    return shadow


def update_display(shadow, candidate, viewer):
    """Opaque measured arm and cyan translucent candidate; no mj_step."""
    import mujoco
    geometry = shadow.geometry
    sample = shadow.session.cache.latest()
    if sample and sample.valid:
        geometry.data.qpos[:] = geometry.qpos(sample.q_rad)
        mujoco.mj_forward(geometry.model, geometry.data)
    with viewer.lock():
        viewer.user_scn.ngeom = 0
        if candidate is not None:
            ghost = shadow._preview_ghost
            ghost.qpos[:] = geometry.qpos(candidate.q_rad)
            mujoco.mj_forward(geometry.model, ghost)
            mujoco.mjv_updateScene(geometry.model, ghost, viewer.opt, None, viewer.cam,
                                  mujoco.mjtCatBit.mjCAT_DYNAMIC, viewer.user_scn)
            for i in range(viewer.user_scn.ngeom):
                viewer.user_scn.geoms[i].rgba[:] = [.1, .8, 1., .3]
    viewer.sync()


class ControlLoopTiming:
    """Skip expired slots; distinguish a real stall from accumulated lateness."""
    stall_s = .1  # Existing experimental budget, now applied to actual elapsed time.

    def __init__(self, period_s, started):
        self.period_s = period_s
        self.deadline = self.last_finished = started
        self.cycles = self.skipped = self.stalls = 0
        self.last = {}
        self.maximum = {}

    def begin(self, now, fault):
        self.started = now
        self.phases = {}
        self.stalled = False
        self.gap_s = now - self.last_finished
        if self.gap_s > self.stall_s:
            self._stall(fault)

    def _stall(self, fault):
        if not self.stalled:
            self.stalled = True
            self.stalls += 1
            fault("preview_control_stall")

    def checkpoint(self, now, phase, elapsed_s, fault):
        self.phases[phase] = elapsed_s
        if now - self.started > self.stall_s:
            self._stall(fault)

    def finish(self, now, fault):
        if now - self.started > self.stall_s:
            self._stall(fault)
        self.cycles += 1
        self.last_finished = now
        self.last = dict(self.phases, work_s=now-self.started, idle_gap_s=self.gap_s)
        for key, value in self.last.items():
            self.maximum[key] = max(value, self.maximum.get(key, 0.))
        self.deadline += self.period_s
        if self.deadline < now:
            missed = math.ceil((now-self.deadline)/self.period_s)
            self.skipped += missed
            self.deadline += missed*self.period_s
        return max(0., self.deadline-now)

    def status(self):
        return dict(requested_period_s=self.period_s, stall_budget_s=self.stall_s,
            cycles=self.cycles, skipped_deadlines=self.skipped, stalls=self.stalls,
            last=dict(self.last), maximum=dict(self.maximum))


def serve_quest(args, shadow, runtime, stack, tls, *, refresh_feedback, feedback_status, extra_status=None):
    """Shared original relay, desktop display and control-owner loop. No SDK loading."""
    import mujoco
    import uvicorn
    from fastapi.responses import HTMLResponse, JSONResponse
    from vr_teleop_kit.relay import server as relay
    mode = getattr(runtime, "mode", "aubo_readonly_preview")
    motion_available = getattr(runtime, "motion_available", False)
    status = PreviewStatus()
    relay.CAMERA_SPECS.clear()
    relay.CAMERA_READERS.clear()
    relay.app.state.teleop_info = dict(backend=mode, motion_available=motion_available,
        rotation_mode=args.rotation, active_hands=["right"], head_yaw_alignment=False)
    relay.app.add_api_route("/aubo/status", lambda: JSONResponse(status.read()), methods=["GET"])
    if hasattr(runtime, 'request_operator_command'):
        def operator_command(command: str):
            try:
                accepted = runtime.request_operator_command(command)
                return JSONResponse(dict(queued=accepted), status_code=202 if accepted else 503)
            except ValueError as exc:
                return JSONResponse(dict(error=str(exc)), status_code=400)
        relay.app.add_api_route('/aubo/operator/{command}', operator_command, methods=['POST'])
    relay.app.add_api_route("/aubo", lambda: HTMLResponse(
        ('<meta charset="utf-8"><title>AUBO Quest</title><h2>' +
        (('AUBO：六轴实体透传' if mode == 'aubo_six_axis_supervised_passthrough' else
          'AUBO：六轴实体跟随' if mode == 'aubo_six_axis_supervised_follow' else 'AUBO：J6 受限实体跟随') if motion_available else 'AUBO：真机禁止运动') + '</h2>') +
        '<p>实体色＝实际反馈，青色＝候选目标；实际提交见状态字段。网页控制滑条不生效，Trigger 无实体夹爪。</p>'
        '<p>进入头显透视后，视野下缘会显示录制状态卡：当前段号、录制时长、保存确认和连接状态。状态卡只显示后台已确认的信息。</p>'
        '<p id="query-mode"></p><p id="clutch"></p><p id="collection"></p><p id="home"></p>'
        '<p>操作提示：左 Y 按住直接回原位，松开停止，无需右 Grip。左摇杆左推＝作废当前段并重启循环；右推＝保存并退出当前循环；上推＝保存当前段并结束录制会话。右 Grip 松开或输入超时会暂停当前段，重新按下并成功接合后续录同一段；右摇杆按下可添加事件标记。</p>'
        '<div id="operator-buttons"><button data-command="pause">暂停／解除暂停</button> '
        '<button data-command="restart_episode">开始／保存并重开</button> '
        '<button data-command="toggle_episode">开始／仅保存本段</button> '
        '<button data-command="restart_loop">作废并重启当前循环</button> '
        '<button data-command="exit_loop">退出并保存当前循环</button> '
        '<button data-command="exit_record">结束并保存整个录制会话</button> '
        '<button data-command="invalidate">本段无效</button> '
        '<button data-command="mark">事件标记</button> '
        '<button data-command="return_home">准备运动回原位（按住左 Y 执行）</button> '
        '<button data-command="software_reset">软件复位（不回位）</button> '
        '<button data-command="finish_session">停止并结束会话</button></div>'
        '<p id="operator-result"></p><pre id="s"></pre><script>'
        'document.querySelectorAll("[data-command]").forEach(b=>b.onclick=async()=>{'
        'if(["invalidate","exit_record","finish_session"].includes(b.dataset.command)&&!confirm("确认执行："+b.textContent+"？"))return;'
        'try{const r=await fetch("/aubo/operator/"+b.dataset.command,{method:"POST"});'
        'document.getElementById("operator-result").textContent=r.ok?"请求已提交，等待处理结果":"请求失败"}'
        'catch(e){document.getElementById("operator-result").textContent="连接失败"}});'
        'setInterval(async()=>{'
        'const s=await(await fetch("/aubo/status")).json();'
        'document.getElementById("query-mode").textContent="状态查询："+'
        '(s.hardware?.state_query_mode_label||(s.motion_available?"等待模式信息":"未启用实体控制"));'
        'const labels={operator_paused:"操作已锁定暂停：解除后请松开再按 Grip",stopping:"停止中：请保持 Grip 松开，等待停稳",'
        'episode_waiting_start:"本段已结束：保存完成后向左推摇杆开始下一段，再按下 Grip 控制",'
        'ready_to_engage:"可接合：下一次按下 Grip 即可控制",'
        'release_grip:"请松开 Grip，再按下接合",engaged:"已接合",fault:"故障锁定：请查看停止状态"};'
        'document.getElementById("clutch").textContent=labels[s.clutch_state]||"";'
        'const h=s.home_return;const homes={disabled:"不可用",idle:"待请求",waiting_for_stop:"等待停稳",'
        'ready:"已准备：按住左 Y 回位，无需右 Grip",moving:"正在回位（松开左 Y 停止）",stopping:"正在确认停稳",'
        'complete:"已在原位并停稳",cancelled:"已取消，需重新请求",failed:"失败，请检查原因"};'
        'document.getElementById("home").textContent=h?"运动回位："+(homes[h.state]||h.state)+'
        '(h.max_error_deg!=null?"；最大关节误差 "+h.max_error_deg.toFixed(3)+"°":"")+(h.error?"；"+h.error:""):"";'
        'const c=s.collection||{state:"disabled"};'
        'const states={disabled:"录制未启用",ready:"待录制（向左推摇杆开始）",recording:"录制中",paused:"录制已暂停，等待 Grip 重新接合",saving:"保存中",error:"录制失败"};'
        'const results={saved:"已保存",invalid:"已标记无效（原始数据保留）",interrupted:"未正常结束"};'
        'document.getElementById("collection").textContent=(states[c.state]||c.state)+"；"+'
        '(c.active_episode!=null?"当前段 "+c.active_episode+"；":"")+'
        '(c.state==="paused"?"暂停原因："+(c.pause_reason||"等待接合")+"；":"")+'
        '(c.last_result?"最近段 "+c.last_result.episode_id+"："+(results[c.last_result.disposition]||c.last_result.disposition)+"（已落盘）；":"")+'
        '(s.operator_message||"")+(c.error?"；"+c.error:"");'
        'document.getElementById("operator-buttons").hidden=!("operator_paused" in s);'
        'document.getElementById("s").textContent=JSON.stringify(s,null,2)},250)</script>'), methods=["GET"])
    server = uvicorn.Server(uvicorn.Config(relay.app, host="127.0.0.1" if args.usb else "0.0.0.0",
        port=args.relay_port, log_level="warning", **tls))
    thread = threading.Thread(target=server.run, name="quest-preview-relay", daemon=True)
    def close_server():
        server.should_exit = True
        thread.join(timeout=5.)
        if thread.is_alive():
            raise RuntimeError("Relay did not exit")
    stack.callback(close_server)
    thread.start()
    deadline = time.monotonic() + 5.
    while not server.started:
        if not thread.is_alive() or time.monotonic() > deadline:
            raise RuntimeError("Relay failed to start")
        time.sleep(.01)
    receiver = QuestReceiver(runtime, f"{'ws' if args.usb else 'wss'}://127.0.0.1:{args.relay_port}/ws")
    stack.callback(receiver.close)
    receiver.start()
    viewer = None
    if not args.headless:
        from mujoco import viewer as mjviewer
        viewer = stack.enter_context(mjviewer.launch_passive(shadow.geometry.model, shadow.geometry.data))
        viewer.opt.geomgroup[3] = 0
        viewer.cam.lookat[:] = [.2, 0., .35]
        viewer.cam.distance = 1.4
        shadow._preview_ghost = mujoco.MjData(shadow.geometry.model)
    LOG.info("Quest ready: mode=%s motion_available=%s recording=%s; flange tool; HMD yaw OFF", mode, motion_available,
             'READY (button managed)' if getattr(runtime, 'recorder', None) else 'OFF')
    LOG.info("Quest: %s://<PC hotspot/Wi-Fi IP>:%d/ ; status: /aubo ; no need to run a second relay",
             "http" if args.usb else "https", args.relay_port)
    LOG.info("Preview joint ceilings deg/s=%s; core %.0fHz; original simulation caps retained; no physical speed implied",
             [math.degrees(v) for v in shadow.session.policy.velocity_rad_s], 1./runtime.control_dt)
    started = last_log = last_view = last_timing_report = time.monotonic()
    timing = ControlLoopTiming(runtime.control_dt, started)
    logged_hardware_event = None
    logged_input_stop_stamp = None
    try:
        while viewer is None or viewer.is_running():
            tick_started = time.monotonic()
            timing.begin(tick_started, runtime.submit_fault)
            refresh_feedback()
            feedback_finished = time.monotonic()
            timing.checkpoint(feedback_finished, "feedback_s", feedback_finished-tick_started, runtime.submit_fault)
            candidate = runtime.step()
            ik_finished = time.monotonic()
            timing.checkpoint(ik_finished, "runtime_step_s", ik_finished-feedback_finished, runtime.submit_fault)
            sample = shadow.session.cache.latest()
            current = sample.q_rad if sample and sample.valid else None
            value = dict(mode=mode, motion_available=motion_available,
                preview_speed_deg_s=args.preview_speed_deg_s,
                candidate_velocity_limits_deg_s=[math.degrees(v) for v in shadow.session.policy.velocity_rad_s],
                core_frequency_hz=1./runtime.control_dt,
                tool_frame="wrist3_Link/flange", pose_frame="model base (SDK base Z offset documented separately)",
                rotation=args.rotation, head_yaw_alignment=False, gripper_installed=False,
                quest_to_base=shadow.core.config.r_calib,
                workspace_alignment=getattr(args,"workspace_alignment",None),
                alignment_reference_required=runtime.expected_reference_id,
                quest_connected=receiver.connected.is_set(), **{k:v for k,v in runtime.status().items()
                if k not in ("backend", "motion_available")},
                measured_q_rad=current, candidate_q_rad=candidate.q_rad if candidate else None,
                candidate_sent_to_robot=False, feedback=feedback_status(), input_stats=dict(shadow.core.input_stats),
                local_revoke_requests=shadow.session.backend.stop_requests,
                feedback_age_s=time.monotonic()-sample.received_monotonic_s if sample else None)
            value.setdefault('recording', False)
            value["relay_connected"] = receiver.connected.is_set()
            age = value["xr_input"]["valid_frame_age_s"]
            value["quest_input_fresh"] = age is not None and age <= shadow.session.policy.input_max_age_s
            value["candidate_max_delta_deg"] = (max(abs(a-b) for a,b in zip(candidate.q_rad, current)) * 180. / math.pi
                if candidate is not None and current is not None else None)
            if extra_status:
                value.update(extra_status())
            hardware = value.get("hardware", {})
            event = (hardware.get("first_stop_trigger"), hardware.get("first_fatal"),
                     hardware.get("state"), hardware.get("stop_confirmed"))
            if hardware and event != logged_hardware_event:
                input_stop = value.get("last_input_stop")
                input_stop_stamp = input_stop["trigger"]["host_monotonic_s"] if input_stop else None
                # Keep terminal logging small: serializing both rings here
                # would add avoidable I/O to the 200 Hz control loop. Full
                # frozen evidence remains available from /aubo/status.
                input_stop_summary = ({key:input_stop[key] for key in
                    ("trigger", "captured_monotonic_s", "core_interruption", "last_received", "last_processed")}
                    if input_stop and input_stop_stamp != logged_input_stop_stamp else None)
                LOG.info("Hardware transition: %s", json.dumps(json_safe(dict(
                    state=hardware.get("state"), fault=hardware.get("fault"),
                    first_stop_trigger=hardware.get("first_stop_trigger"),
                    first_fatal=hardware.get("first_fatal"),
                    input_stop=input_stop_summary,
                    stop_confirmed=hardware.get("stop_confirmed"),
                    last_attempt=hardware.get("last_attempt"), last_stop=hardware.get("last_stop")))))
                logged_hardware_event = event
                logged_input_stop_stamp = input_stop_stamp
            status_finished = now = time.monotonic()
            timing.checkpoint(now, "status_s", now-ik_finished, runtime.submit_fault)
            if viewer and now - last_view >= 1./30:
                update_display(shadow, candidate, viewer)
                last_view = time.monotonic()
            now = time.monotonic()
            timing.checkpoint(now, "display_s", now-status_finished, runtime.submit_fault)
            if now - last_log >= 2.:
                LOG.info("preview engaged=%s require_release=%s fault=%s input_frames=%s feedback_error=%s",
                    value["engaged"], value["require_release"], value["fault"],
                    value["input_stats"]["frames"], value["feedback"]["error"])
                last_log = now
            finished = time.monotonic()
            timing.finish(finished, runtime.submit_fault)
            value["control_loop_timing"] = timing.status()
            status.update(value)
            # Bounded diagnostic event rate; recording has its own writer.
            if timing.stalled or finished-last_timing_report >= 1.:
                report = getattr(runtime, "record_control_timing", None)
                if report:
                    report(timing.status())
                last_timing_report = finished
            if args.duration and finished - started >= args.duration:
                break
            if getattr(runtime, 'session_finished', False):
                break
            time.sleep(max(0., timing.deadline - time.monotonic()))
    except KeyboardInterrupt:
        LOG.info("Closing Quest interface")
    LOG.info("Quest interface closed; input_frames=%d; mode=%s", shadow.core.input_stats["frames"], mode)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", action="store_true", help="allow SDK login and Get calls only")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int, help="SDK port")
    parser.add_argument("--user")
    parser.add_argument("--runtime")
    parser.add_argument("--workspace-alignment", help="explicit calibrated XR-to-workspace JSON")
    parser.add_argument("--relay-port", type=int, default=8443)
    parser.add_argument("--usb", action="store_true")
    parser.add_argument("--rotation", choices=("world", "tool_local"), default="world")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=0.)
    parser.add_argument("--preview-speed-deg-s", type=float,
                        help="optional extra candidate cap; default matches the accepted simulation baseline")
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0 or not 1 <= args.relay_port <= 65535:
        parser.error("Invalid duration or relay port")
    try:
        velocities = preview_velocity_limits(args.preview_speed_deg_s)
    except ValueError as exc:
        parser.error(str(exc))
    from .workspace import load_alignment
    alignment=load_alignment(args.workspace_alignment) if args.workspace_alignment else None
    if not args.connect:
        if args.host or args.port or args.user:
            parser.error("Device parameters require --connect")
        print(json.dumps(dict(mode="offline_plan", motion_available=False, sdk_loaded=False,
            tool_frame="flange", rotation=args.rotation, head_yaw_alignment=False,
            recording=False, preview_speed_deg_s=args.preview_speed_deg_s,
            candidate_velocity_limits_deg_s=[math.degrees(v) for v in velocities], core_frequency_hz=200.,
            next="Use --connect with verified host, port and user for Get-only live preview"), indent=2))
        return 0
    if not args.host or not args.user or args.port is None or not 1 <= args.port <= 65535:
        parser.error("--connect requires host, port and user")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    from .runtime import OfflineRuntime
    from teleop import mujoco_session  # Establish vendored package import path.

    with ExitStack() as stack:
        lock = stack.enter_context((ROOT / ".teleop_session.lock").open("a"))
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", args.relay_port))
        tls = {} if args.usb else dict(ssl_keyfile=str(ROOT / "certs/key.pem"),
                                       ssl_certfile=str(ROOT / "certs/cert.pem"))
        if any(not Path(p).is_file() for p in tls.values()):
            raise RuntimeError("Missing LAN TLS certificate; use existing certificate setup or --usb")
        follower = ReadOnlyFollower(load_native(args.runtime).ReadOnlyClient())
        password = getpass.getpass("SDK password (not recorded): ")
        try:
            result = follower.connect(host=args.host, port=args.port, user=args.user, password=password)
        finally:
            del password
        if not record_ok(result):
            raise RuntimeError(f"SDK login failed: {result['return_code']}")
        # Registered first so the polling task is joined before logout.
        def logout():
            result = follower.disconnect()
            if result is not None and not record_ok(result):
                raise RuntimeError("Read-only SDK logout failed")
        stack.callback(logout)
        initial = {}
        for name in ("server_version", "is_real_robot", "robot_state", "joint_ranges", "joint_angles"):
            result = follower.query(name)
            initial[name] = result
            if not record_ok(result) or result["duration_s"] > .2:
                raise RuntimeError(f"Read-only precheck failed/late: {name}")
        if initial["is_real_robot"]["data"] is not True or initial["robot_state"]["data"] != 0:
            raise RuntimeError("Preview requires the real robot to be stopped")
        if not initial["joint_ranges"]["data"]["enable_raw"]:
            raise RuntimeError("Reported joint range protection is disabled")
        shadow = make_shadow(initial["joint_angles"]["data"]["joint_positions_rad"],
                             initial["joint_ranges"]["data"]["range_values_raw"], rotation=args.rotation,
                             preview_speed_deg_s=args.preview_speed_deg_s,
                             quest_to_base=alignment["quest_to_base"] if alignment else None)
        runtime = OfflineRuntime(shadow, control_dt=shadow.core.config.control_dt, dispatch_fake=False,
            expected_reference_id=alignment["xr_reference_id"] if alignment else None)
        stack.callback(runtime.close)
        shadow.observe_readonly(follower)
        poller = FeedbackPoller(follower, runtime)
        # If a worker ever fails to join, logout must not run against it.
        def close_poller():
            try:
                poller.close()
            except RuntimeError:
                follower.connected = False
                raise
        stack.callback(close_poller)
        poller.start()
        serve_quest(args, shadow, runtime, stack, tls,
                    refresh_feedback=lambda:shadow.observe_readonly(follower), feedback_status=poller.status)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
