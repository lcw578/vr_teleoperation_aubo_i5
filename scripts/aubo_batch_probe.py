"""Stationary-only bounded four-point drain probe, production stop owner."""
import time
from . import aubo_staged_acceptance as acceptance
from teleop.aubo.follow import J6FollowWorker
from teleop.aubo.probe import record_ok

class ProbeWorker(J6FollowWorker):
    def __init__(self,client,*args,**kwargs):
        owner=self
        class Proxy:
            def __init__(self):self.index=0;self.calls=0;self.last_probe=0
            def __getattr__(self,name):return getattr(client,name)
            def waypoint_feedback_since(self,sequence):return client.waypoint_feedback_since(sequence)
            def send_joints(self,q,token):
                self.calls+=1
                if self.index>=8 or self.calls<5 or time.monotonic()-self.last_probe<.3:
                    return client.send_joints(q,token)
                if max(abs(a-b) for a,b in zip(q,owner._priming_anchor))>1e-12:
                    raise RuntimeError('Probe permits stationary anchor only')
                # Let every preceding stationary single point drain before one small burst.
                time.sleep(.05)
                before=client.query_diagnosis()
                n=4;self.index+=1
                result=client.send_batch([q]*n,token)
                diagnostics=[]
                started=time.monotonic()
                while time.monotonic()-started<.08:
                    row=client.query_diagnosis();diagnostics.append(row)
                    if not record_ok(row) or row['duration_s']>.1:break
                self.last_probe=time.monotonic()
                owner._diagnose('stationary_batch_drain_probe',point_count=n,
                    before=before,batch_result=result,diagnosis=diagnostics)
                return result
        super().__init__(Proxy(),*args,**kwargs)

if __name__=='__main__':
    acceptance.J6FollowWorker=ProbeWorker
    raise SystemExit(acceptance.main())
