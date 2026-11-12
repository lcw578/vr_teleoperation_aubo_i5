"""Continuous quintic references checked at whole-interval polynomial extrema.

Endpoints and all real derivative roots are checked, including quartic roots
for position range. Limits describe references, not measured motor derivatives.
"""
import math
from .feedback import joints


class JointTrajectory:
    velocity_rad_s = math.radians(10.)
    acceleration_rad_s2 = math.radians(30.)
    jerk_rad_s3 = math.radians(300.)
    max_dt_s = .05

    def __init__(self, q, bounds, *, profile=None):
        if profile is not None:
            from .motion_profile import motion_profile
            self.velocity_rad_s,self.acceleration_rad_s2,self.jerk_rad_s3=motion_profile(profile).radians
        self.bounds = tuple(bounds)
        self.reset(q)

    def reset(self, q):
        self.q = joints(q)
        self.v = self.a = (0.,)*6
        self.goal = None
        self.controls = None
        self.elapsed = self.duration = 0.

    def restart_from_rest(self,q):
        """Restart the host reference; this does not assert measured motor rest."""
        self.reset(q)

    @staticmethod
    def derivative(points, duration):
        return tuple((len(points)-1)*(b-a)/duration for a,b in zip(points,points[1:]))

    @staticmethod
    def evaluate(points, fraction):
        while len(points)>1:
            points = tuple((1-fraction)*a+fraction*b for a,b in zip(points,points[1:]))
        return points[0]

    @staticmethod
    def _power(points):
        """Bernstein to ascending power coefficients on normalized u in [0,1]."""
        n=len(points)-1;row=tuple(points);result=[]
        for k in range(n+1):
            result.append(math.comb(n,k)*row[0])
            row=tuple(b-a for a,b in zip(row,row[1:]))
        return tuple(result)

    @staticmethod
    def _poly_value(coeff,u):
        value=0.
        for x in reversed(coeff):value=value*u+x
        return value

    @classmethod
    def _roots_unit(cls,coeff):
        """Isolate every real root on [0,1] between derivative critical points.

        Quadratics use the cancellation-resistant formula. Higher degrees
        (at most four) are split into monotonic intervals, then bisected.
        Repeated roots are retained by checking the split points themselves.
        No sample grid is used as a substitute for interval extrema.
        """
        coeff=list(coeff);scale=max(map(abs,coeff),default=0.)
        if not math.isfinite(scale):raise ValueError('Non-finite polynomial')
        if scale==0.:return ()
        coeff=[x/scale for x in coeff]
        while len(coeff)>1 and abs(coeff[-1])<1e-14:coeff.pop()
        degree=len(coeff)-1
        if degree==0:return ()
        if degree==1:
            root=-coeff[0]/coeff[1]
            return (min(1.,max(0.,root)),) if -1e-13<=root<=1.+1e-13 else ()
        if degree==2:
            c,b,a=coeff;disc=b*b-4*a*c
            if disc < -1e-14:return ()
            root=math.sqrt(max(0.,disc));q=-.5*(b+math.copysign(root,b))
            values=(-b/(2*a),) if q==0. else (q/a,c/q)
            return tuple(sorted(set(min(1.,max(0.,x)) for x in values if -1e-13<=x<=1.+1e-13)))
        critical=cls._roots_unit(tuple(i*coeff[i] for i in range(1,len(coeff))))
        cuts=(0.,)+tuple(x for x in critical if 0.<x<1.)+(1.,)
        roots=[x for x in cuts if abs(cls._poly_value(coeff,x))<=1e-13]
        for lo,hi in zip(cuts,cuts[1:]):
            left=cls._poly_value(coeff,lo);right=cls._poly_value(coeff,hi)
            if left*right>=0.:continue
            for _ in range(42):
                mid=(lo+hi)/2;value=cls._poly_value(coeff,mid)
                if value==0.:lo=hi=mid;break
                if left*value<=0.:hi=mid
                else:lo=mid;left=value
            roots.append((lo+hi)/2)
        return tuple(sorted(set(roots)))

    @classmethod
    def extrema(cls,points):
        """Return (u,value) at endpoints and every interior stationary point."""
        critical=cls._roots_unit(cls._power(cls.derivative(points,1.))) if len(points)>1 else ()
        return tuple((u,cls.evaluate(points,u)) for u in (0.,1.)+tuple(x for x in critical if 0.<x<1.))

    @classmethod
    def curve_within(cls,points,duration,bounds,limits):
        levels=[(points,*bounds)]
        for limit in limits:
            points=cls.derivative(points,duration);levels.append((points,-limit,limit))
        for values,lo,hi in levels:
            # A hull entirely inside the interval is a cheap sufficient check;
            # a hull outside it is NEVER a reason to reject a curve.
            if all(lo<=x<=hi for x in values):continue
            if any(not lo-1e-11<=x<=hi+1e-11 for _,x in cls.extrema(values)):return False
        return True

    def _plan(self, target):
        limits=(self.velocity_rad_s,self.acceleration_rad_s2,self.jerk_rad_s3)
        def candidate(duration):
            controls=[]
            for q,v,a,goal,bounds in zip(self.q,self.v,self.a,target,self.bounds):
                p=(q,q+v*duration/5,q+2*v*duration/5+a*duration**2/20,goal,goal,goal)
                if not self.curve_within(p,duration,bounds,limits):return None
                controls.append(p)
            return tuple(controls)
        # Exact rest-to-rest extrema: 15/8, 10*sqrt(3)/3, 60.
        distance=max(abs(x-y) for x,y in zip(self.q,target))
        rest_time=max(.005,1.875*distance/limits[0],
            math.sqrt(10*math.sqrt(3)*distance/(3*limits[1])),(60*distance/limits[2])**(1/3))
        if all(abs(x)<1e-12 for x in self.v+self.a):
            duration=rest_time*1.01;controls=candidate(duration)
            if controls is not None:
                self.controls=controls;self.duration,self.elapsed,self.goal=duration,0.,target;return
        duration=max(.005,rest_time*.4);previous=0.
        for _ in range(45):
            controls=candidate(duration)
            if controls is not None:
                low,high=previous,duration
                for _ in range(10):
                    mid=(low+high)/2;trial=candidate(mid)
                    if trial is None:low=mid
                    else:high=mid;controls=trial
                # Prefer 1% duration margin only if the entire resulting curve
                # still passes (feasibility need not be monotonic in duration).
                trial=candidate(high*1.01)
                if trial is not None:high*=1.01;controls=trial
                self.controls=controls;self.duration,self.elapsed,self.goal=high,0.,target;return
            previous=duration;duration*=1.2
        # Preserve the previous certified curve and q/v/a on an infeasible
        # instantaneous reversal. Retrying next tick never snaps the reference.
        if self.controls is None:
            raise RuntimeError('No bounded joint trajectory; stop and re-anchor required')

    def brake(self):
        """Free-endpoint stop with full-interval extrema checks.

        At fixed duration every polynomial value is affine in the endpoint.
        Constraint generation intersects its feasible interval at violating
        extrema until the complete curve passes, preserving initial q/v/a.
        """
        def feasible(duration):
            controls=[];goals=[]
            for q,v,a,(lo,hi) in zip(self.q,self.v,self.a,self.bounds):
                base=(q,q+v*duration/5,q+2*v*duration/5+a*duration**2/20,0.,0.,0.)
                coeff=(0.,0.,0.,1.,1.,1.)
                levels=[(base,coeff,lo,hi)];b,c=base,coeff
                for limit in (self.velocity_rad_s,self.acceleration_rad_s2,self.jerk_rad_s3):
                    b=self.derivative(b,duration);c=self.derivative(c,duration)
                    levels.append((b,c,-limit,limit))
                low,high=lo,hi;preferred=q+v*duration/2+a*duration**2/12
                for iteration in range(40):
                    goal=min(high,max(low,preferred));valid=True
                    for bs,cs,lower,upper in levels:
                        values=tuple(b+c*goal for b,c in zip(bs,cs))
                        for u,value in self.extrema(values):
                            if lower-1e-11<=value<=upper+1e-11:continue
                            valid=False;b=self.evaluate(bs,u);c=self.evaluate(cs,u)
                            if abs(c)<1e-15:return None
                            x,y=(lower-b)/c,(upper-b)/c
                            low,high=max(low,min(x,y)),min(high,max(x,y))
                            if low>high+1e-13:return None
                    if valid:break
                else:return None
                goals.append(goal);controls.append(base[:3]+(goal,goal,goal))
            return tuple(controls),tuple(goals)
        duration=max(.005,max(map(abs,self.v))/self.acceleration_rad_s2,
            max(map(abs,self.a))/self.jerk_rad_s3);previous=0.
        for _ in range(145):
            plan=feasible(duration)
            if plan is not None:
                low,high=previous,duration
                for _ in range(18):
                    mid=(low+high)/2
                    trial=feasible(mid)
                    if trial is None:low=mid
                    else:high=mid;plan=trial
                margin_plan=feasible(high*(1+1e-7))
                if margin_plan is not None:high*=1+1e-7;plan=margin_plan
                self.controls,self.goal=plan;self.duration,self.elapsed=high,0.
                return self.goal,high
            previous=duration;duration*=1.05
            if duration>3.:break
        raise RuntimeError('No certified braking curve within joint limits')

    @property
    def stopped(self):
        return self.controls is not None and self.elapsed>=self.duration

    def advance(self, target, real_dt):
        target = joints(target)
        if not math.isfinite(real_dt) or real_dt<=0:
            raise ValueError('Positive actual send interval required')
        if any(not lo<=v<=hi for v,(lo,hi) in zip(target,self.bounds)):
            raise ValueError('Trajectory goal outside controller range')
        if target != self.goal:
            self._plan(target)
        effective_dt = min(real_dt,self.max_dt_s)
        self.elapsed = min(self.duration,self.elapsed+effective_dt)
        fraction = self.elapsed/self.duration
        self.q = tuple(self.evaluate(p,fraction) for p in self.controls)
        self.v = tuple(self.evaluate(self.derivative(p,self.duration),fraction) for p in self.controls)
        self.a = tuple(self.evaluate(self.derivative(self.derivative(p,self.duration),self.duration),fraction)
                       for p in self.controls)
        return self.q, dict(real_dt_s=real_dt,effective_dt_s=effective_dt,
            target_replan_deferred=target!=self.goal,
            dt_capped=real_dt>self.max_dt_s,q_cmd_rad=self.q,
            v_cmd_rad_s=self.v,a_cmd_rad_s2=self.a,
            velocity_limit_rad_s=self.velocity_rad_s,acceleration_limit_rad_s2=self.acceleration_rad_s2,
            jerk_limit_rad_s3=self.jerk_rad_s3)

    def sample_fixed_ticks(self, target, count, *, tick_s=.005, first_anchor=False):
        """Ordered, equally spaced samples; never skip to a wall-time endpoint."""
        if type(count) is not int or not 1<=count<=8 or tick_s!=.005:
            raise ValueError('Fixed tick batch requires 1..8 points at 5ms')
        points=[];details=[]
        for index in range(count):
            if first_anchor and index==0:
                q=self.q
                detail=dict(first_active_anchor=True,q_cmd_rad=self.q,
                    v_cmd_rad_s=self.v,a_cmd_rad_s2=self.a,effective_dt_s=0.,
                    real_dt_s=0.,dt_capped=False,target_replan_deferred=False)
            else:q,detail=self.advance(target,tick_s)
            points.append(q);details.append(detail)
        return tuple(points),dict(tick_s=tick_s,point_count=count,points=details,
            q_cmd_rad=self.q,v_cmd_rad_s=self.v,a_cmd_rad_s2=self.a,
            first_active_anchor=first_anchor,target_replan_deferred=any(d.get('target_replan_deferred') for d in details),
            dt_capped=False)
