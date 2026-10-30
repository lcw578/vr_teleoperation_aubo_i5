"""Read-only Quest integration boundaries and flange geometry; no robot I/O."""
import asyncio
import json
import os
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mujoco
import numpy as np
from teleop.aubo.model import AuboModel
from teleop.aubo.quest_preview import ControlLoopTiming, FeedbackPoller, QuestReceiver, main, make_shadow
from teleop.aubo.runtime import OfflineRuntime


def frame(seq, grip=False, valid=True):
    return dict(type="xr_frame", epoch="preview-test", seq=seq, t_client=time.monotonic()*1000,
        controllers={"right":dict(position=[0.,0.,0.], orientation=[0.,0.,0.,1.], tracking_valid=valid,
            buttons=[dict(p=False,v=0.),dict(p=grip,v=float(grip))])})


class PreviewTests(unittest.TestCase):
    def test_slow_iterations_skip_slots_without_cumulative_fault_or_catchup(self):
        for work in (.006, .008, .020):
            timer=ControlLoopTiming(.005, 0.)
            fault=Mock(); now=0.
            for _ in range(500):
                timer.begin(now, fault)
                now+=work
                delay=timer.finish(now, fault)
                self.assertGreaterEqual(delay, 0.)
                self.assertLessEqual(delay, .005+1e-12)
                now+=delay
            fault.assert_not_called()
            self.assertGreater(timer.skipped, 0)
            self.assertEqual(timer.cycles, 500)

    def test_fast_iterations_keep_five_ms_schedule(self):
        timer=ControlLoopTiming(.005, 0.); fault=Mock(); now=0.
        for _ in range(100):
            timer.begin(now, fault)
            now+=.002
            now+=timer.finish(now, fault)
        self.assertAlmostEqual(now, .5)
        self.assertEqual(timer.skipped, 0)
        fault.assert_not_called()

    def test_long_display_revokes_once_and_records_phase(self):
        timer=ControlLoopTiming(.005, 0.); fault=Mock()
        timer.begin(0., fault)
        timer.checkpoint(.012, "runtime_step_s", .012, fault)
        timer.checkpoint(.132, "display_s", .120, fault)
        timer.finish(.133, fault)
        fault.assert_called_once_with("preview_control_stall")
        stats=timer.status()
        self.assertAlmostEqual(stats["last"]["work_s"], .133)
        self.assertEqual(stats["maximum"]["display_s"], .120)
        self.assertEqual(stats["stalls"], 1)

    def test_scheduler_suspension_revokes_before_next_control_work(self):
        timer=ControlLoopTiming(.005, 0.); events=[]
        timer.begin(0., events.append); timer.finish(.002, events.append)
        timer.begin(.150, events.append)
        events.append("next_control_step")
        self.assertEqual(events, ["preview_control_stall", "next_control_step"])
        timer.finish(.152, events.append)
        self.assertAlmostEqual(timer.status()["last"]["idle_gap_s"], .148)

    def test_slow_feedback_revokes_at_checkpoint_before_ik(self):
        timer=ControlLoopTiming(.005, 0.); fault=Mock()
        timer.begin(0., fault)
        timer.checkpoint(.120, "feedback_s", .120, fault)
        fault.assert_called_once_with("preview_control_stall")
        timer.finish(.122, fault)
        fault.assert_called_once()

    def test_default_and_bad_args_do_not_load_sdk(self):
        with patch("teleop.aubo.quest_preview.load_native") as load, patch("builtins.print"):
            self.assertEqual(main([]), 0)
            load.assert_not_called()
        with patch("sys.stderr"), patch("teleop.aubo.quest_preview.load_native") as load:
            for args in (["--connect"], ["--host","192.168.1.5"], ["--duration","nan"]):
                with self.assertRaises(SystemExit): main(args)
            load.assert_not_called()

    def test_flange_has_no_gripper_actuators_or_tool_offset(self):
        flange=AuboModel(tool_frame="flange")
        self.assertEqual(flange.model.nq, 6)
        self.assertEqual(flange.model.nu, 0)
        self.assertEqual(flange.model.ncam, 0)
        self.assertEqual(mujoco.mj_name2id(flange.model,mujoco.mjtObj.mjOBJ_BODY,"rg_base"),-1)
        rg=AuboModel()
        for q in ([.18,.26,2.37,2.04,1.51,.8],[.2,-.4,1.1,1.,.7,-.3]):
            np.testing.assert_allclose(flange.pose(q,"tool0"),rg.pose(q,"wrist3_Link"),atol=1e-12)
        for q in ([.18,.26,2.37,2.04,1.51,.8],):
            flange.pose(q,"tool0");rg.pose(q,"tool0")
            np.testing.assert_allclose(flange.data.site("j4_anchor").xpos,rg.data.site("j4_anchor").xpos,atol=1e-12)

    def test_live_initialization_uses_measured_rest_and_gates_first_grip(self):
        q=[.18,.26,2.37,2.04,1.51,.8]
        s=make_shadow(q,[[-3.05,3.05]]*6)
        np.testing.assert_array_equal(s.core._arms["right"]["qpos"],q)
        np.testing.assert_array_equal(s.core.config.rest_qpos_right,q)
        s.feedback(q,received_monotonic_s=time.monotonic(),source="test")
        self.assertTrue(s.accept_xr_frame(frame(0,True)))
        self.assertIsNone(s.tick())
        self.assertTrue(s.session.require_release)
        for i,grip in enumerate([False,True],1):
            s.feedback(q,received_monotonic_s=time.monotonic(),source="test")
            s.accept_xr_frame(frame(i,grip));s.tick()
        self.assertTrue(s.session.engaged)
        np.testing.assert_allclose(s.session.last_reference,q)
        s.close()

    def test_poller_retains_late_results_invalidates_feedback_and_stops(self):
        f=Mock()
        f.query.side_effect=[dict(return_code=0,data={}),dict(return_code=0,data=0)]
        r=Mock();p=FeedbackPoller(f,r)
        with patch("teleop.aubo.quest_preview.time.monotonic",side_effect=[10.,10.23]):p._run()
        self.assertIsNotNone(p.status()["error"])
        self.assertIn("last_joint_result",p.status())
        self.assertEqual(f.query.call_count,2)
        f.cache.invalidate.assert_called_once()
        r.submit_fault.assert_called_once_with("preview_feedback_failed")

    def test_real_websocket_routes_frames_without_running_ik_in_receiver(self):
        s=make_shadow([.18,.26,2.37,2.04,1.51,.8],[[-3.05,3.05]]*6)
        runtime=OfflineRuntime(s,control_dt=.01)
        async def exercise():
            import websockets
            async def handler(ws,path=None):
                await ws.send(json.dumps(frame(0,False)))
                await ws.send(json.dumps(frame(1,True)))
                await ws.wait_closed()
            async with websockets.serve(handler,"127.0.0.1",0) as server:
                port=server.sockets[0].getsockname()[1]
                receiver=QuestReceiver(runtime,f"ws://127.0.0.1:{port}/ws")
                receiver.start()
                try:
                    until=time.monotonic()+2.
                    while runtime._latest_frame is None and time.monotonic()<until:await asyncio.sleep(.01)
                    self.assertIsNotNone(runtime._latest_frame)
                    self.assertIsNone(s.input_time)
                    self.assertEqual(s.session.backend.submitted,[])
                    s.feedback([.18,.26,2.37,2.04,1.51,.8],received_monotonic_s=time.monotonic())
                    runtime.step()
                    self.assertFalse(s.session.engaged)  # Release delivered first.
                finally:
                    receiver.stop.set()
                    await asyncio.to_thread(receiver.close)
        asyncio.run(exercise());runtime.close()


if __name__=="__main__":unittest.main(verbosity=2)
