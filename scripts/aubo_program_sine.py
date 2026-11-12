"""Finite acceptance input: certified entry bridge, then a fixed-tick sine.

Test-only trajectory source. Production VR keeps the online joint planner.
"""
import math
from teleop.aubo.trajectory import JointTrajectory

class ProgramSineTrajectory(JointTrajectory):
    def __init__(self,q,bounds,*,axis,center,amplitude_deg=6.,period_s=4.2,profile=None):
        self.axis=axis;self.center=tuple(center);self.amplitude=math.radians(amplitude_deg)
        self.omega=2*math.pi/period_s
        super().__init__(q,bounds,profile=profile)
    def reset(self,q):
        super().reset(q)
        self.program_time=0.;self.program_enabled=True;self.bridge=None;self.bridge_s=float('inf');self.recovery_bridge=None
        self.entry_center_pending=max(abs(a-b) for a,b in zip(self.q,self.center))>1e-10
    def restart_from_rest(self,q):
        if not self.program_enabled or self.bridge is None or self.program_time<=self.bridge_s:
            self.reset(q);return
        # Rejoin at the next sine turning point (zero speed), using a certified
        # bridge. This test source must not replay its first trough forever.
        JointTrajectory.reset(self,q)
        half_period=math.pi/self.omega
        next_turn=(math.floor((self.program_time-self.bridge_s)/half_period)+1)*half_period
        self.recovery_resume_time=self.bridge_s+next_turn
        duration=.2
        for _ in range(100):
            t=next_turn
            goal=list(self.center);ev=[0.]*6;ea=[0.]*6
            goal[self.axis]-=self.amplitude*math.cos(self.omega*t)
            ev[self.axis]=0.
            ea[self.axis]=self.amplitude*self.omega**2*math.cos(self.omega*t)
            controls=[];valid=True
            for i,(start,end,v,a,bounds) in enumerate(zip(self.q,goal,ev,ea,self.bounds)):
                p=(start,start,start,end-2*v*duration/5+a*duration**2/20,end-v*duration/5,end)
                lo,hi=bounds
                if i==self.axis:lo,hi=max(lo,self.center[i]-self.amplitude),min(hi,self.center[i]+self.amplitude)
                valid &= all(lo-1e-12<=x<=hi+1e-12 for x in p)
                values=p
                for limit in (self.velocity_rad_s,self.acceleration_rad_s2,self.jerk_rad_s3):
                    values=self.derivative(values,duration);valid &= max(map(abs,values))<=limit+1e-12
                controls.append(p)
            if valid:
                self.recovery_bridge=tuple(controls);self.recovery_duration=duration;self.recovery_elapsed=0.;return
            duration*=1.05
        raise RuntimeError('No certified zero-speed sine recovery bridge')
    def _bridge(self):
        goal=list(self.center);goal[self.axis]-=self.amplitude
        end_a=[0.]*6;end_a[self.axis]=self.amplitude*self.omega**2
        duration=.2
        for _ in range(100):
            controls=[];valid=True
            for q,v,a,g,ea,bounds in zip(self.q,self.v,self.a,goal,end_a,self.bounds):
                p=(q,q+v*duration/5,q+2*v*duration/5+a*duration**2/20,
                   g+ea*duration**2/20,g,g)
                valid &= all(bounds[0]<=x<=bounds[1] for x in p)
                valid &= all(max(map(abs,values))<=limit+1e-12 for values,limit in (
                    (self.derivative(p,duration),self.velocity_rad_s),
                    (self.derivative(self.derivative(p,duration),duration),self.acceleration_rad_s2),
                    (self.derivative(self.derivative(self.derivative(p,duration),duration),duration),self.jerk_rad_s3)))
                controls.append(p)
            if valid:self.bridge=tuple(controls);self.bridge_s=duration;return
            duration*=1.05
        raise RuntimeError('No certified sine entry bridge')
    def advance(self,target,real_dt):
        if not self.program_enabled:return super().advance(target,real_dt)
        if self.entry_center_pending:
            # Test-only inter-cycle bridge: return from the stopped brake
            # endpoint to the sine center before its acceleration-matched entry.
            q,detail=super().advance(self.center,real_dt)
            self.entry_center_pending=not super().stopped
            return q,dict(detail,programmed_sine=True,entry_center_bridge=True)
        if self.bridge is None:self._bridge()
        if real_dt!=.005:raise ValueError('Program sine requires fixed 5ms samples')
        self.program_time+=real_dt
        if self.recovery_bridge is not None:
            self.recovery_elapsed=min(self.recovery_duration,self.recovery_elapsed+real_dt)
            fraction=self.recovery_elapsed/self.recovery_duration
            p=self.recovery_bridge
            self.q=tuple(self.evaluate(x,fraction) for x in p)
            self.v=tuple(self.evaluate(self.derivative(x,self.recovery_duration),fraction) for x in p)
            self.a=tuple(self.evaluate(self.derivative(self.derivative(x,self.recovery_duration),self.recovery_duration),fraction) for x in p)
            if self.recovery_elapsed>=self.recovery_duration:
                self.program_time=self.recovery_resume_time
                self.recovery_bridge=None
        elif self.program_time<=self.bridge_s:
            fraction=self.program_time/self.bridge_s
            self.q=tuple(self.evaluate(p,fraction) for p in self.bridge)
            self.v=tuple(self.evaluate(self.derivative(p,self.bridge_s),fraction) for p in self.bridge)
            self.a=tuple(self.evaluate(self.derivative(self.derivative(p,self.bridge_s),self.bridge_s),fraction) for p in self.bridge)
        else:
            t=self.program_time-self.bridge_s
            q=list(self.center);v=[0.]*6;a=[0.]*6
            q[self.axis]-=self.amplitude*math.cos(self.omega*t)
            v[self.axis]=self.amplitude*self.omega*math.sin(self.omega*t)
            a[self.axis]=self.amplitude*self.omega**2*math.cos(self.omega*t)
            self.q,self.v,self.a=tuple(q),tuple(v),tuple(a)
        return self.q,dict(real_dt_s=real_dt,effective_dt_s=real_dt,dt_capped=False,
            target_replan_deferred=False,q_cmd_rad=self.q,v_cmd_rad_s=self.v,a_cmd_rad_s2=self.a,
            programmed_sine=True,program_time_s=self.program_time,bridge_s=self.bridge_s)
    @property
    def stopped(self):return not self.program_enabled and super().stopped
    def brake(self):
        self.program_enabled=False
        return super().brake()
