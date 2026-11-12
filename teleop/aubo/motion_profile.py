"""Explicit host/native reference limits; no writes to cabinet motion profiles."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class MotionProfile:
    name: str
    velocity_deg_s: float
    acceleration_deg_s2: float
    jerk_deg_s3: float

    @property
    def radians(self):
        return tuple(math.radians(v) for v in (
            self.velocity_deg_s,self.acceleration_deg_s2,self.jerk_deg_s3))

    @property
    def step_deg(self):return self.velocity_deg_s*.005

    def metadata(self):
        return dict(name=self.name,velocity_deg_s=self.velocity_deg_s,
            acceleration_deg_s2=self.acceleration_deg_s2,jerk_deg_s3=self.jerk_deg_s3,
            tick_s=.005,max_step_deg=self.step_deg)

PROFILES={p.name:p for p in (MotionProfile('conservative',10.,30.,300.),
    MotionProfile('practical',45.,180.,1800.),MotionProfile('balanced',25.,75.,600.),MotionProfile('advanced',35.,105.,900.))}

def motion_profile(name):
    try:return PROFILES[name]
    except (KeyError,TypeError):raise ValueError('Unknown motion profile: '+str(name)) from None
