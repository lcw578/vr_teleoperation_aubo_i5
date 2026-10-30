"""User-selected joint home and bounded targets; no SDK or device I/O."""
import json
import math
from pathlib import Path

from .feedback import joints


DEFAULT_HOME_PATH = Path('/data/VR_teleoperation_runtime/config/aubo_home_20261029T091225103094Z.json')
HOME_SPEED_RAD_S = math.radians(20.)
HOME_ACCEL_RAD_S2 = math.radians(6.)
HOME_TOLERANCE_RAD = math.radians(.15)


def load_home(path, ranges):
    path = Path(path).expanduser().resolve()
    value = json.loads(path.read_text())
    if (value.get('schema') != 'aubo_reference_home_v1' or value.get('joint_source') != 'sdk_waypoint_push'
            or value.get('joint_names') != [f'joint_{n}' for n in range(1, 7)]):
        raise ValueError('Home must be a measured six-axis aubo_reference_home_v1 reference')
    q = joints(value['q_rad'])
    if len(ranges) != 6 or any(not lo <= angle <= hi for angle, (lo, hi) in zip(q, ranges)):
        raise ValueError('Home outside current controller joint ranges')
    return dict(path=str(path), q_rad=q, captured_at_utc=value.get('captured_at_utc'))


def bounded_home_step(previous, target, velocity, dt):
    """Bound target speed/acceleration, brake before goal, never overshoot.

    dt is capped at one requested send period by the SDK owner. A delayed
    call must not accumulate a large subsequent target jump.
    """
    previous, target, velocity = map(joints, (previous, target, velocity))
    if not math.isfinite(dt) or not 0. < dt <= .05+1e-12:
        raise ValueError('Home step requires one positive supported send period')
    positions, velocities = [], []
    for position, goal, old_velocity in zip(previous, target, velocity):
        error = goal-position
        desired = math.copysign(min(HOME_SPEED_RAD_S,
            math.sqrt((HOME_ACCEL_RAD_S2*dt)**2+2.*HOME_ACCEL_RAD_S2*abs(error))
            -HOME_ACCEL_RAD_S2*dt), error)
        change = HOME_ACCEL_RAD_S2*dt
        speed = max(old_velocity-change, min(old_velocity+change, desired))
        step = speed*dt
        if error == 0. or step*error < 0.:
            step = 0.; speed = 0.
        elif abs(step) >= abs(error):
            step = error; speed = 0.
        positions.append(position+step)
        velocities.append(speed)
    return tuple(positions), tuple(velocities)
