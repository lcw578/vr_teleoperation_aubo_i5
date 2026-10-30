"""Fixed workspace alignment from explicit hand sweeps; no SDK or motion API."""
import argparse
from collections import deque
from contextlib import ExitStack
from datetime import datetime
import fcntl
import json
import math
from pathlib import Path
import threading
import time

import numpy as np


def unit(value, name):
    v = np.asarray(value, float)
    if v.shape != (3,) or not np.all(np.isfinite(v)) or np.linalg.norm(v) < 1e-6:
        raise ValueError(f'Invalid {name}')
    return v / np.linalg.norm(v)


def workspace_basis(flange_transform):
    """Photo convention: horizontal flange +Z = forward, +world Z = up."""
    T = np.asarray(flange_transform, float)
    if (T.shape != (4,4) or not np.all(np.isfinite(T)) or
            not np.allclose(T[:3,:3].T @ T[:3,:3], np.eye(3), atol=1e-6) or
            not np.isclose(np.linalg.det(T[:3,:3]), 1., atol=1e-6)):
        raise ValueError('Invalid flange transform')
    forward = T[:3,2].copy(); forward[2] = 0.
    if np.linalg.norm(forward) < .5:
        raise ValueError('Flange too vertical to define workspace forward')
    forward = unit(forward,'workspace forward');up=np.array([0.,0.,1.])
    right = np.cross(forward,up)
    return np.column_stack((right,forward,up))


def fit_alignment(base_basis, sweeps):
    """Yaw-only XR->base rotation; measured up/right/forward validate semantics."""
    B = np.asarray(base_basis,float)
    if (B.shape != (3,3) or not np.all(np.isfinite(B)) or
            not np.allclose(B.T@B,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(B),1.) or
            not np.allclose(B[:,2],[0,0,1],atol=1e-6)):
        raise ValueError('Invalid fixed workspace basis')
    r,f,u = [unit(sweeps[k],k) for k in ('right','forward','up')]
    if abs(r[1]) > .25 or abs(f[1]) > .25 or u[1] < .95:
        raise ValueError('Repeat sweeps: right/forward must be horizontal and up vertical')
    r[1]=f[1]=0.;r=unit(r,'right');f=unit(f,'forward')
    if abs(float(r@f)) > .25 or np.cross(r,f)[1] < .95:
        raise ValueError('Repeat sweeps: right/forward not perpendicular or wrong sign')
    xr_up = np.array([0.,1.,0.])
    xr_right = unit(r+np.cross(f,xr_up),'fitted right')
    xr_forward = np.cross(xr_up,xr_right)
    X = np.column_stack((xr_right,xr_forward,xr_up))
    R=B@X.T
    errors=[math.degrees(math.acos(float(np.clip(R@v@B[:,i],-1,1))))
            for i,v in enumerate((r,f,u))]
    if max(errors)>15.:
        raise ValueError('Sweep residual exceeds 15 degrees; repeat without turning')
    return R, errors


def load_alignment(path):
    data=json.loads(Path(path).expanduser().read_text())
    R=np.asarray(data['quest_to_base'],float)
    B=np.asarray(data['workspace_basis_base'],float)
    ref=data['xr_reference_id']
    if (data.get('schema')!='aubo_fixed_workspace_v1' or R.shape!=(3,3) or B.shape!=(3,3) or
            not np.all(np.isfinite(R)) or not np.all(np.isfinite(B)) or
            not np.allclose(R.T@R,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(R),1.) or
            not np.allclose(R@[0,1,0],[0,0,1],atol=1e-6) or
            not np.allclose(B.T@B,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(B),1.) or
            not isinstance(ref,str) or not ref):
        raise ValueError('Invalid workspace alignment')
    return data


class WorkspaceSampler:
    """One deliberate Grip sweep per stage; bounded raw buffer, no robot access."""
    stages=('right','forward','up')
    def __init__(self, basis, output, *, provenance, clock=time.monotonic):
        self.basis=np.asarray(basis,float);self.output=Path(output);self.provenance=provenance
        self.clock=clock;self.lock=threading.RLock()
        self.state='waiting_start';self.stage=0;self.samples=deque(maxlen=1500)
        self.sweeps={};self.reference=None;self.epoch=None;self.last_seq=None
        self.grip=False;self.need_release=True;self.error=None;self.last_frame=None;self.result=None
    def start(self):
        with self.lock:
            if self.state=='saved':return
            self.state='ready';self.stage=0;self.samples.clear();self.sweeps={}
            self.reference=self.epoch=self.last_seq=None
            self.grip=False;self.need_release=True;self.error=None
    def submit_fault(self,reason):
        with self.lock:
            if self.state=='capturing':
                self.samples.clear();self.state='ready';self.need_release=True
                self.error='Input interrupted; release Grip and repeat: '+str(reason)
    def submit_xr_frame(self,frame,*,received_monotonic_s=None):
        from .shadow import normalize_xr_frame
        try:
            frame=normalize_xr_frame(frame)
            ref=frame.get('reference_id')
            if not isinstance(ref,str) or not ref:raise ValueError('Refresh Quest webpage: missing spatial reference ID')
            stamp=float(self.clock() if received_monotonic_s is None else received_monotonic_s)
            if not math.isfinite(stamp) or not 0<=self.clock()-stamp<=.25:raise ValueError('Input timestamp stale/invalid')
        except (ValueError,TypeError,KeyError,OverflowError) as exc:
            self.submit_fault(str(exc));return False
        with self.lock:
            self.last_frame=stamp
            if self.state in ('waiting_start','saved'):return True
            if self.reference and ref!=self.reference:
                self.state='waiting_start';self.error='Quest reference changed; restart all sweeps';return False
            if self.epoch and frame['epoch']!=self.epoch:
                self.submit_fault('Transport epoch changed');self.last_seq=None
            if self.last_seq is not None and frame['seq']<=self.last_seq:return False
            self.reference=ref;self.epoch=frame['epoch'];self.last_seq=frame['seq']
            ctrl=frame['controllers']['right'];pressed=ctrl['buttons'][1]['p']
            if self.state=='capturing' and self.samples and stamp-self.samples[-1][0]>.25:
                self.submit_fault('Input gap')
            if self.need_release:
                if not pressed:self.need_release=False;self.grip=False
                return True
            if pressed and not self.grip:
                self.samples.clear();self.state='capturing';self.error=None
            if self.state=='capturing':
                self.samples.append((stamp,ctrl['position'],ctrl['orientation']))
                if pressed and self.samples[-1][0]-self.samples[0][0]>15.:
                    self.submit_fault('Sweep exceeded 15 seconds')
                elif not pressed:
                    try:self._finish()
                    except (ValueError,OSError) as exc:
                        self.error=str(exc);self.state='ready';self.need_release=True
                    self.samples.clear()
            self.grip=pressed
            return True
    def _finish(self):
        a=list(self.samples)
        if len(a)<20 or a[-1][0]-a[0][0]<1.:
            raise ValueError('Hold still 0.5s at both ends; sweep must last at least 1s')
        first=[v for v in a if v[0]<=a[0][0]+.25]
        last=[v for v in a if v[0]>=a[-1][0]-.25]
        if min(len(first),len(last))<5:raise ValueError('Insufficient endpoint samples')
        ends=[np.asarray([v[1] for v in group]) for group in (first,last)]
        centres=[np.median(v,axis=0) for v in ends]
        if any(np.max(np.linalg.norm(v-c,axis=1))>.02 for v,c in zip(ends,centres)):
            raise ValueError('Endpoints moved; pause 0.5s before moving and before releasing Grip')
        q0=np.asarray(first[0][2]);qs=np.asarray([v[2] for v in a])
        if np.min(np.abs(qs@q0))<math.cos(math.radians(20)/2):
            raise ValueError('Controller rotated over 20 degrees; repeat translation only')
        delta=centres[1]-centres[0]
        if not .08<=np.linalg.norm(delta)<=.4:
            raise ValueError('Sweep must move 8 to 40cm; recommended 15cm')
        self.sweeps[self.stages[self.stage]]=delta.tolist()
        if self.stage<2:self.stage+=1;self.state='ready';return
        try:R,errors=fit_alignment(self.basis,self.sweeps)
        except ValueError:
            self.stage=0;self.sweeps={};raise
        data=dict(schema='aubo_fixed_workspace_v1',created_at=datetime.now().astimezone().isoformat(),
            workspace_basis_base=self.basis.tolist(),quest_to_base=R.tolist(),
            xr_reference_id=self.reference,sweeps_quest=self.sweeps,residual_deg=errors,
            provenance=self.provenance,head_yaw_alignment=False,motion_available=False)
        with self.output.open('x') as stream:json.dump(data,stream,ensure_ascii=False,indent=2);stream.write('\n')
        self.result=data;self.state='saved'
    def status(self):
        with self.lock:
            return dict(state=self.state,next_direction=self.stages[self.stage],error=self.error,
                completed=list(self.sweeps),reference_id=self.reference,motion_available=False,
                last_frame_age_s=None if self.last_frame is None else self.clock()-self.last_frame,
                output=str(self.output) if self.state=='saved' else None,result=self.result)


PAGE='''<meta charset="utf-8"><title>AUBO 工作区方向采样</title>
<h2>工作区方向采样：机器人不会运动</h2>
<p>先在头显原网页进入 Teleop、完成腕部校准，再点击本页开始。保持同一站位与面向。</p>
<button onclick="fetch('/workspace/start',{method:'POST'})">开始／重新采样</button>
<p>依次：右 → 前（朝玻璃门）→ 上。每项：松开 Grip，按住静止半秒，平移约15cm，手柄停半秒，再松开 Grip。不要转手腕。</p>
<p>成功后保持头显的 Teleop 会话开启，不刷新、不重置空间。转头不影响结果。</p>
<pre id="s"></pre><script>setInterval(async()=>{s.textContent=JSON.stringify(await(await fetch('/workspace/status')).json(),null,2)},250)</script>'''


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--basis-report',required=True,help='Previously saved read-only flange FK report')
    p.add_argument('--output',required=True,help='New small alignment JSON under /data')
    args=p.parse_args(argv)
    output=Path(args.output).expanduser().resolve()
    if Path('/data') not in output.parents or not output.parent.is_dir() or output.exists():
        p.error('Output must be new and under an existing /data directory')
    report=json.loads(Path(args.basis_report).read_text())
    if report['state']['return_code']!=0 or report['state']['data']!=0 or report['joints']['return_code']!=0:
        p.error('Basis report must contain a successful stopped read-only observation')
    basis=workspace_basis(report['model_flange_T'])
    sampler=WorkspaceSampler(basis,output,provenance=dict(
        basis_report=str(Path(args.basis_report).resolve()),joint_positions_rad=report['joints']['data']['joint_positions_rad'],
        definition='Horizontal projection of photo/current flange +Z points forward; world +Z up'))
    from .quest_preview import ROOT,QuestReceiver
    from teleop import mujoco_session  # Use the existing vendored relay.
    from vr_teleop_kit.relay import server as relay
    from fastapi.responses import HTMLResponse
    import uvicorn
    relay.CAMERA_SPECS.clear();relay.CAMERA_READERS.clear()
    relay.app.state.teleop_info=dict(backend='workspace_calibration_only',motion_available=False,
        head_yaw_alignment=False,active_hands=['right'])
    relay.app.add_api_route('/workspace',lambda:HTMLResponse(PAGE))
    relay.app.add_api_route('/workspace/status',sampler.status)
    def start():sampler.start();return sampler.status()
    relay.app.add_api_route('/workspace/start',start,methods=['POST'])
    with ExitStack() as stack:
        lock=stack.enter_context((ROOT/'.teleop_session.lock').open('a'))
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        server=uvicorn.Server(uvicorn.Config(relay.app,host='0.0.0.0',port=8443,log_level='warning',
            ssl_keyfile=str(ROOT/'certs/key.pem'),ssl_certfile=str(ROOT/'certs/cert.pem')))
        thread=threading.Thread(target=server.run,daemon=True);thread.start()
        def close_server():server.should_exit=True;thread.join(5.)
        stack.callback(close_server)
        deadline=time.monotonic()+5.
        while not server.started:
            if not thread.is_alive() or time.monotonic()>deadline:raise RuntimeError('Relay failed to start')
            time.sleep(.02)
        receiver=QuestReceiver(sampler,'wss://127.0.0.1:8443/ws');receiver.start();stack.callback(receiver.close)
        print('Calibration only; no SDK loaded. Desktop: https://127.0.0.1:8443/workspace',flush=True)
        try:
            while thread.is_alive():
                if sampler.status()['state']=='saved':
                    print('Alignment saved: '+str(output),flush=True)
                    # Stay open until controlled close so user keeps XR reference.
                    while thread.is_alive():time.sleep(.2)
                time.sleep(.1)
        except KeyboardInterrupt:pass
    return 0


if __name__=='__main__':raise SystemExit(main())
