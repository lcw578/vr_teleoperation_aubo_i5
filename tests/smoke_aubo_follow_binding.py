"""Load and inspect supervised follower binding; do not construct vendor ServiceInterface/login."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from teleop.aubo.follow import J6FollowWorker, load_follow_native
from teleop.aubo.readonly import load_native
module=load_follow_native()
# These two constants belong to the retained legacy J6-only trial.
assert module.client_speed_deg_s == J6FollowWorker.speed_deg_s == 10.
assert module.client_acceleration_deg_s2 == J6FollowWorker.client_acceleration_deg_s2 == 5.
assert module.following_error_limit_deg is J6FollowWorker.following_error_limit_deg is None
assert module.six_axis_target_policy == "direct_ik"
assert module.six_axis_sdk_profile_policy == "sdk_unmodified"
assert module.supports_passthrough is True
assert module.supports_follow_rate_selection is True
assert module.supports_parallel_state_query is True
assert module.supports_redundant_waypoint_feedback is True
assert module.send_period_s == J6FollowWorker.period_s == .1
assert module.six_axis_send_period_s == J6FollowWorker.six_axis_period_s == .005
assert module.six_axis_follow_period_s == J6FollowWorker.six_axis_follow_period_s == .05
assert module.send_max_gap_s == J6FollowWorker.send_max_gap_s == .65
assert module.sdk_budget_s == J6FollowWorker.sdk_budget_s == .5
client=module.J6Follower() # Vendor object is lazy until login.
assert {n for n in dir(client) if not n.startswith('_')} == {
    'login','logout','prepare','snapshot','arm','revision','revoke','send_offset','stop',
    'start_waypoint_feedback','waypoint_feedback','send_joints','enter_passthrough',
    'attach_state_observer','abandon_cleanup','waypoint_feedback_since'}
observer=module.RobotStateObserver()
assert {n for n in dir(observer) if not n.startswith('_')} == {
    'login','logout','query_state','invalidate','abandon_cleanup','waypoint_feedback'}
try:observer.waypoint_feedback()
except RuntimeError as exc:assert 'not enabled' in str(exc)
else:raise AssertionError('Unsubscribed observer feedback accepted')
for name in ('query_state','logout'):
    try:getattr(observer,name)()
    except RuntimeError as exc:assert 'Not connected' in str(exc)
    else:raise AssertionError('Unconnected state operation accepted')
observer.invalidate()
assert client.revision()==0
assert client.revoke()==1
for name,args in [('prepare',()),('snapshot',()),('arm',(1,)),('send_offset',(0.,1)),('stop',()),('logout',()),
                  ('start_waypoint_feedback',()),('waypoint_feedback',()),('waypoint_feedback_since',(0,)),
                  ('send_joints',([0.]*6,1)),('enter_passthrough',(1,))]:
    try:getattr(client,name)(*args)
    except RuntimeError as exc:assert 'Not connected' in str(exc)
    else:raise AssertionError('Unconnected SDK operation accepted: '+name)
readonly=load_native().ReadOnlyClient()
assert not hasattr(readonly,'stop') and not hasattr(readonly,'send_offset')
print('Supervised follower binding, unconnected refusal and read-only boundary passed; no vendor client/login/hardware call.')

# Explicit six-axis construction is also lazy and must refuse offline sends.
six=module.J6Follower(six_axis=True)
for name,args in [('prepare',()),('arm',(0,)),('send_joints',([0.]*6,0)),('send_offset',(0.,0))]:
    try:getattr(six,name)(*args)
    except RuntimeError as exc:assert 'Not connected' in str(exc)
    else:raise AssertionError('Unconnected six-axis operation accepted: '+name)
passthrough=module.J6Follower(six_axis=True,passthrough=True)
for name,args in [('enter_passthrough',(0,)),('send_joints',([0.]*6,0)),('stop',())]:
    try:getattr(passthrough,name)(*args)
    except RuntimeError as exc:assert 'Not connected' in str(exc)
    else:raise AssertionError('Unconnected passthrough operation accepted: '+name)
try:module.J6Follower(passthrough=True)
except ValueError:pass
else:raise AssertionError('J6 passthrough scope accepted')

selected=module.J6Follower(six_axis=True,follow_hz=50.)
assert selected.revision()==0
selected25=module.J6Follower(six_axis=True,follow_hz=25.)
assert selected25.revision()==0
selected200=module.J6Follower(six_axis=True,follow_hz=200.)
assert selected200.revision()==0
try:selected200.send_joints([0.]*6,0)
except RuntimeError as exc:assert 'Not connected' in str(exc)
else:raise AssertionError('Unconnected 200 Hz operation accepted')
for kwargs in [dict(follow_hz=50.),dict(six_axis=True,passthrough=True,follow_hz=50.),
               dict(six_axis=True,follow_hz=100.)]:
    try:module.J6Follower(**kwargs)
    except ValueError:pass
    else:raise AssertionError('Invalid Follow rate scope accepted')
