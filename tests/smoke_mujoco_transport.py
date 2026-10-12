"""Real WS + three WebRTC tracks + opt-in records, against a bounded session."""
import argparse
import asyncio
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request

import numpy as np
import websockets
from aiortc import RTCPeerConnection, RTCConfiguration, RTCSessionDescription

ROOT = Path(__file__).resolve().parents[1]


async def probe(port, backend):
    pc = RTCPeerConnection(RTCConfiguration(iceServers=[]))
    frames = []
    frame_tasks = []
    states = []
    @pc.on("track")
    def on_track(track):
        async def capture():
            frame = await asyncio.wait_for(track.recv(), 10.)
            rgb = frame.to_ndarray(format="rgb24")
            assert rgb.std() > 2, (rgb.shape, rgb.std())
            frames.append(dict(width=frame.width,height=frame.height,std=float(rgb.std())))
        frame_tasks.append(asyncio.create_task(capture()))
    async with websockets.connect(f"ws://127.0.0.1:{port}/ws", max_queue=16) as ws:
        keep_sending = True
        started = time.monotonic()
        async def sender():
            seq=0
            while keep_sending:
                elapsed=time.monotonic()-started
                buttons=[dict(p=False,v=0.) for _ in range(6)]
                buttons[1]=dict(p=elapsed>.1,v=float(elapsed>.1))
                # 2cm up, then hold; reset-safe relative input, 72Hz.
                y=.02*min(max(elapsed-.2,0)/2,1)
                await ws.send(json.dumps(dict(type="xr_frame",epoch="smoke",seq=seq,t_client=time.monotonic()*1000,
                    controllers={"right":dict(position=[0,y,0],orientation=[0,0,0,1],buttons=buttons)})))
                seq+=1
                await asyncio.sleep(1/72)
        sending=asyncio.create_task(sender())
        await ws.send(json.dumps(dict(type="webrtc_request")))
        try:
            deadline=time.monotonic()+20
            while time.monotonic()<deadline:
                msg=json.loads(await asyncio.wait_for(ws.recv(),10))
                if msg.get("type")=="webrtc_offer":
                    assert len(msg["cameras"])==3, msg["cameras"]
                    await pc.setRemoteDescription(RTCSessionDescription(sdp=msg["sdp"],type=msg["sdp_type"]))
                    await pc.setLocalDescription(await pc.createAnswer())
                    await ws.send(json.dumps(dict(type="webrtc_answer",sdp=pc.localDescription.sdp,sdp_type=pc.localDescription.type)))
                elif msg.get("type")=="ik_state":
                    states.append(msg)
                if len(frames)==3 and time.monotonic()-started>10:
                    break
            assert len(frames)==3, frames
            assert all(s["rotation_mode"]=="world" for s in states)
            assert all(s["state_source"] == ("physics" if backend == "physics" else "kinematic_preview") for s in states)
            assert any(s["engaged"] for s in states)
            assert any(s["measured_qpos"] for s in states)
            await asyncio.gather(*frame_tasks)
            return dict(video_frames=frames,states=len(states),input=states[-1]["input_stats"])
        finally:
            keep_sending=False
            await sending
            await pc.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("kinematic", "physics"), default="kinematic")
    args = parser.parse_args()
    with socket.socket() as s:
        s.bind(("127.0.0.1",0)); port=s.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix="aubo-transport-") as base:
        base=Path(base)
        log=(base/"runtime.log").open("w")
        env=dict(os.environ,MUJOCO_GL="egl")
        command=[sys.executable,str(ROOT/"teleop/mujoco_session.py"),"--headless","--usb","--port",str(port),
                 "--backend",args.backend,"--record",str(base/"record"),"--duration","30"]
        process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,cwd=ROOT)
        try:
            deadline=time.monotonic()+10
            while True:
                if process.poll() is not None: raise RuntimeError((base/"runtime.log").read_text())
                try:
                    info=json.load(urllib.request.urlopen(f"http://127.0.0.1:{port}/teleop-info",timeout=.5))
                    assert info["head_yaw_alignment"] is False
                    assert info["backend"] == args.backend
                    break
                except (OSError, TimeoutError):
                    if time.monotonic()>deadline: raise
                    time.sleep(.05)
            result=asyncio.run(probe(port, args.backend))
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=8)
            log.close()
        assert process.returncode==0, (base/"runtime.log").read_text()
        records=[json.loads(line) for line in (base/"record/signals.jsonl").read_text().splitlines()]
        assert len(records)>100
        assert all(r["fault"] is None for r in records)
        assert all(r["command_apply_sim_time"] < r["sim_time"] for r in records)
        assert all(r["backend"] == args.backend for r in records)
        if args.backend == "kinematic":
            assert all(r["q_measured"] is None and r["physics_qpos"] is None for r in records)
        peak=max(max(abs(a-b) for a,b in zip(r["q_simulated"],r["q_commanded"])) for r in records)
        assert peak<.03, peak
        result.update(backend=args.backend,recorded_controls=len(records),max_display_or_following_gap_rad=peak)
        print(json.dumps(result,indent=2))


if __name__=="__main__":
    main()
