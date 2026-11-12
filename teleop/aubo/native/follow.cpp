// Supervised J6 or explicit six-axis following. No startup, home or reconnect.
// Motion calls are serial; optional state observer owns a different connection.
// revoke() is local and cannot interrupt an in-flight call.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "aubo_sdk/serviceinterface.h"
#include "follow_profile.hpp"
#include "passthrough.hpp"
#include "state_channel.hpp"
#include "stop_compat.hpp"
#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <deque>
namespace py=pybind11;
namespace a=aubo_robot_namespace;
constexpr double DEG=0.017453292519943295;
// Legacy J6 trial parameters only. Six-axis leaves the SDK profile untouched.
constexpr double J6_SPEED_DEG_S=10., CLIENT_ACC_DEG_S2=5.;
// Command/feedback differences are diagnostic only; no host lag threshold.
constexpr double SEND_PERIOD_S=.1, SEND_MAX_GAP_S=.65, SDK_CALL_BUDGET_S=.5;
constexpr double SIX_AXIS_SEND_PERIOD_S=.005, SIX_AXIS_FOLLOW_PERIOD_S=.05;
static double now_s(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
// Subscriptions are connection-local. Disable every optional telemetry stream
// before selecting a channel's role; SDK event/command replies are separate.
static void disable_realtime_pushes(ServiceInterface&sdk){
 auto check=[](auto call){double started=now_s();int rc=call();
  if(rc!=0||now_s()-started>SDK_CALL_BUDGET_S)
   throw std::runtime_error("Telemetry push disable failed or late");};
 check([&]{return sdk.robotServiceSetRealTimeRoadPointPush(false);});
 check([&]{return sdk.robotServiceSetRealTimeJointStatusPush(false);});
 check([&]{return sdk.robotServiceSetRealTimeEndSpeedPush(false);});
}
static py::dict record(const char* op,int rc,double s,double e){
 py::dict d;d["operation"]=op;d["return_code"]=rc;d["host_started_monotonic_s"]=s;
 d["host_received_monotonic_s"]=e;d["duration_s"]=e-s;return d;
}
// Both SDK receive threads publish directly; neither needs the motion I/O lock.
struct WaypointPush {
 std::array<double,6>q;double received;unsigned long long sequence,channel_sequence;
 char connection='A';
};
struct WaypointPushBuffer {
 std::mutex mutex;std::deque<WaypointPush>samples;
 unsigned long long count=0,channel_count[3]{0,0,0};double channel_received[3]{0,0,0};
};
static void publish_waypoint(const a::wayPoint_S*point,void*arg,char connection){
 if(!point)return;auto*buffer=static_cast<WaypointPushBuffer*>(arg);
 WaypointPush sample{};sample.received=now_s();sample.connection=connection;
#ifdef AUBO_FOLLOW_TEST_PUSH_TIMESTAMP
 if(connection=='A')sample.received=AUBO_FOLLOW_TEST_PUSH_TIMESTAMP(sample.received);
#endif
 for(int i=0;i<6;++i)sample.q[i]=point->jointpos[i];
 std::lock_guard<std::mutex>lock(buffer->mutex);
 const int channel=connection=='C'?2:connection=='B'?1:0;
 sample.channel_sequence=++buffer->channel_count[channel];
 buffer->channel_received[channel]=sample.received;
 sample.sequence=++buffer->count;buffer->samples.push_back(sample);
 if(buffer->samples.size()>256)buffer->samples.pop_front();
}
static py::dict push_feedback(const std::shared_ptr<WaypointPushBuffer>&buffer,
                              unsigned long long after_sequence=0){
 std::deque<WaypointPush>samples;unsigned long long count,counts[3];double received[3];
 {py::gil_scoped_release r;std::lock_guard<std::mutex>p(buffer->mutex);
  for(const auto&sample:buffer->samples)if(sample.sequence>after_sequence)samples.push_back(sample);
  // Return the original last sample even with no new pushes, so freshness
  // checks still expire it instead of treating a quiet stream as renewed.
  if(samples.empty()&&!buffer->samples.empty())samples.push_back(buffer->samples.back());
  count=buffer->count;
  for(int i=0;i<3;++i){counts[i]=buffer->channel_count[i];received[i]=buffer->channel_received[i];}}
 py::list out;for(const auto&s:samples){py::dict v;v["q_rad"]=s.q;
  v["host_received_monotonic_s"]=s.received;v["sequence"]=s.sequence;
  v["connection"]=std::string(1,s.connection);v["channel_sequence"]=s.channel_sequence;out.append(v);}
 py::dict channels;for(int i=0;i<3;++i){py::dict c;c["total_received"]=counts[i];
  c["host_received_monotonic_s"]=counts[i]?py::cast(received[i]):py::object(py::none());
  channels[i==0?"A":i==1?"B":"C"]=c;}
 py::dict d;d["samples"]=out;d["total_received"]=count;d["channels"]=channels;
 d["controller_sample_time"]=py::none();return d;
}
// Cross-process feedback keeps the SDK callback's timestamp and sequence.
// Invalidating the session rejects every packet already in the IPC channel.
class WaypointFeedbackCache {
 std::shared_ptr<WaypointPushBuffer>buffer_=std::make_shared<WaypointPushBuffer>();
 unsigned long long generation_=0;
 int channel_=1;
 public:
 explicit WaypointFeedbackCache(const std::string&source="B"){
  if(source!="B"&&source!="C")throw std::invalid_argument("Independent feedback source must be B or C");
  channel_=source=="C"?2:1;
 }
 std::shared_ptr<WaypointPushBuffer>buffer()const{return buffer_;}
 unsigned long long invalidate(){
  std::lock_guard<std::mutex>lock(buffer_->mutex);++generation_;
  buffer_->samples.clear();buffer_->count=0;
  for(int i=0;i<3;++i){buffer_->channel_count[i]=0;buffer_->channel_received[i]=0;}
  return generation_;
 }
 bool publish(std::array<double,6>q,double received,unsigned long long sequence,
              unsigned long long generation){
  std::lock_guard<std::mutex>lock(buffer_->mutex);
  if(generation!=generation_||sequence<=buffer_->count||!std::isfinite(received)||
     received<=0||received>now_s()||
     (!buffer_->samples.empty()&&received<buffer_->samples.back().received))return false;
  for(double value:q)if(!std::isfinite(value))return false;
  WaypointPush sample{};sample.q=q;sample.received=received;
  sample.sequence=sample.channel_sequence=sequence;sample.connection=channel_==2?'C':'B';
  buffer_->count=buffer_->channel_count[channel_]=sequence;buffer_->channel_received[channel_]=received;
  buffer_->samples.push_back(sample);if(buffer_->samples.size()>256)buffer_->samples.pop_front();
  return true;
 }
 py::dict feedback(){return push_feedback(buffer_);}
};
// No motion, stop, profile or startup API is exposed by this class.
class RobotStateObserver {
 std::unique_ptr<ServiceInterface>sdk_;std::mutex io_;
 StateObservationCache cache_;bool connected_=false;
 std::atomic<bool>abandoned_{false};
 std::shared_ptr<WaypointPushBuffer>push_buffer_;bool push_enabled_=false;
 static void waypoint_callback(const a::wayPoint_S*p,void*arg){publish_waypoint(p,arg,'B');}
 public:
 ~RobotStateObserver(){
  if(abandoned_.load()){sdk_.release();if(push_buffer_)new std::shared_ptr<WaypointPushBuffer>(push_buffer_);return;}
  if(connected_)try{
   if(push_enabled_)sdk_->robotServiceRegisterRealTimeRoadPointCallback(nullptr,nullptr);
   sdk_->robotServiceLogout();}catch(...){}
  sdk_.reset();
 }
 void attach_waypoint_buffer(std::shared_ptr<WaypointPushBuffer>buffer){
  std::lock_guard<std::mutex>lock(io_);
  if(!connected_||push_buffer_)throw std::runtime_error("Observer push requires a connected, unsubscribed client");
  push_buffer_=std::move(buffer);double started=now_s();
  int rc=sdk_->robotServiceRegisterRealTimeRoadPointCallback(waypoint_callback,push_buffer_.get());
  push_enabled_=rc==0;
  if(rc!=0||now_s()-started>SDK_CALL_BUDGET_S)throw std::runtime_error("Observer waypoint subscription failed or late");
 }
 py::dict start_waypoint_feedback(){
  double s=now_s();
  {py::gil_scoped_release r;attach_waypoint_buffer(std::make_shared<WaypointPushBuffer>());}
  return record("observer_register_waypoint_feedback",0,s,now_s());
 }
 py::dict waypoint_feedback(){
  if(!push_buffer_||!push_enabled_)throw std::runtime_error("Observer waypoint feedback not enabled");
  return push_feedback(push_buffer_);
 }
 py::dict waypoint_feedback_since(unsigned long long sequence){
  if(!push_buffer_||!push_enabled_)throw std::runtime_error("Observer waypoint feedback not enabled");
  return push_feedback(push_buffer_,sequence);
 }
 StateObservation observation()const{return cache_.read();}
 void invalidate(){cache_.invalidate();}
 void abandon_cleanup(){abandoned_.store(true);invalidate();}
 py::dict login(const std::string&host,int port,const std::string&user,const std::string&password){
  if(host.empty()||port<1||port>65535)throw std::invalid_argument("Invalid address");
  int rc;double s,e;
  {py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);
   if(sdk_)throw std::runtime_error("New observer required; no reconnect");
   cache_.invalidate();sdk_=std::make_unique<ServiceInterface>();s=now_s();
   rc=sdk_->robotServiceLogin(host.c_str(),port,user.c_str(),password.c_str());e=now_s();connected_=rc==0;
   if(connected_)disable_realtime_pushes(*sdk_);}
  return record("state_observer_login",rc,s,e);
 }
 py::dict query_state(){
  int rc;double s,e;a::RobotState state{};unsigned long long generation;
  {py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);
   if(!connected_)throw std::runtime_error("Not connected");
   generation=cache_.read().generation;s=now_s();
   try{rc=sdk_->robotServiceGetRobotCurrentState(state);e=now_s();}
   catch(...){cache_.invalidate();throw;}
   cache_.publish(int(state),rc,s,e,generation);}
  auto d=record("robot_state",rc,s,e);d["data"]=rc==0?py::cast(int(state)):py::none();
  d["generation"]=generation;d["sequence"]=cache_.read().sequence;return d;
 }
 py::dict logout(){
  cache_.invalidate();int rc;double s,e;
  {py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);
   if(!connected_)throw std::runtime_error("Not connected");
   if(push_enabled_){sdk_->robotServiceRegisterRealTimeRoadPointCallback(nullptr,nullptr);push_enabled_=false;}
   s=now_s();rc=sdk_->robotServiceLogout();e=now_s();connected_=false;}
  return record("state_observer_logout",rc,s,e);
 }
};
class RobotStateCache {
 StateObservationCache cache_;
 public:
 unsigned long long invalidate(){return cache_.invalidate();}
 bool publish(int state,int rc,double started,double received,unsigned long long generation){
  return cache_.publish(state,rc,started,received,generation);
 }
 StateObservation observation()const{return cache_.read();}
};
class J6Follower {
 using Push=WaypointPush;
 using PushBuffer=WaypointPushBuffer;
 std::shared_ptr<PushBuffer>push_buffer_=std::make_shared<PushBuffer>();
 std::mutex&push_mutex_=push_buffer_->mutex;
 std::deque<Push>&pushes_=push_buffer_->samples;unsigned long long&push_count_=push_buffer_->count;
 bool push_enabled_=false;std::atomic<bool>push_readable_{false};
 bool six_axis_=false;std::array<double,6>lower_{},upper_{},last_target_{};
 bool passthrough_=false,inspect_profile_=false;PassthroughMode transport_;
 double follow_period_=SIX_AXIS_FOLLOW_PERIOD_S;
 static void waypoint_callback(const a::wayPoint_S*point,void*arg){
  publish_waypoint(point,arg,'A');
 }
 std::unique_ptr<ServiceInterface> sdk_;std::mutex io_;
 std::shared_ptr<RobotStateObserver>state_observer_;
 std::shared_ptr<RobotStateCache>state_cache_;
 std::shared_ptr<WaypointFeedbackCache>process_feedback_;
 std::atomic<bool>abandoned_{false};
 std::atomic<unsigned long long> revision_{0};std::atomic<bool> permitted_{false};
 bool connected_=false,prepared_=false,failed_=false,have_snapshot_=false;
 unsigned batch_pending_upper_=0;double batch_diagnosis_received_=0;
 double clock_batch_epoch_=0;unsigned long long clock_batch_sent_=0,clock_batch_lost_=0;
 unsigned clock_batch_cushion_=8,clock_underrun_ticks_=8;
 double reference_velocity_deg_s_=10.,reference_acceleration_deg_s2_=30.,reference_jerk_deg_s3_=300.;
 unsigned clock_batch_min_=4;bool push_only_stream_=false,clock_reramp_=false,clock_recovery_pending_=false;
 unsigned long long clock_restarts_=0;
 unsigned clock_effective_cushion_=12,clock_warmup_sent_=0;
 bool clock_warmup_complete_=false,clock_small_restart_=false,clock_stationary_=false;
 bool priming_=false;std::array<double,6>priming_anchor_{};
 double last_leave_=0;double priming_started_=0;unsigned priming_steps_=0,priming_fast_=0,slow_sends_=0;
 std::array<double,6> anchor_{},measured_{};int state_=-1;
 double observed_=0,joints_received_=0,state_received_=0,last_send_=0,last_offset_=0,j6_min_=0,j6_max_=0;
 double joint_age_limit()const{return priming_?(push_only_stream_?.5:.4):(passthrough_&&permitted_.load()?.5:.2);}
 double state_age_limit()const{return priming_?.4:(passthrough_&&permitted_.load()?.5:.2);}
 bool stream_push_only()const{return push_only_stream_&&transport_.entered()&&permitted_.load();}
 bool state_fresh(double now,double limit)const{return now>=state_received_&&(stream_push_only()||now-state_received_<=limit);}
 void require_login(){if(!connected_)throw std::runtime_error("Not connected");}
 std::shared_ptr<PushBuffer>feedback_buffer()const{
  return process_feedback_?process_feedback_->buffer():push_buffer_;
 }
 StateObservation refresh_parallel_observation(){
  auto state=state_cache_?state_cache_->observation():state_observer_->observation();have_snapshot_=false;
#ifdef AUBO_FOLLOW_TEST_BEFORE_PUSH_READ
  // Offline binding only: publish a callback at the formerly racy boundary.
  AUBO_FOLLOW_TEST_BEFORE_PUSH_READ(waypoint_callback,push_buffer_.get());
#endif
  auto buffer=feedback_buffer();
  {std::lock_guard<std::mutex>p(buffer->mutex);
   if(!buffer->samples.empty()){
    const auto&v=buffer->samples.back();measured_=v.q;observed_=joints_received_=v.received;
    have_snapshot_=true;
    for(double q:measured_)have_snapshot_=have_snapshot_&&std::isfinite(q);
   }}
  state_received_=state.received;state_=state.state;
  // Both samples must be copied before taking the validation time. A push
  // received between an earlier clock read and this cache read is not future
  // data. Keep the original reception stamps and reject genuinely future data.
  double now=now_s();
  const double joint_limit=joint_age_limit();
  const double state_limit=state_age_limit();
  have_snapshot_=have_snapshot_&&now>=joints_received_&&now-joints_received_<=joint_limit&&
      state.valid&&state_fresh(now,state_limit);return state;
 }
 public:
 void configure_refined_stream(){
  std::lock_guard<std::mutex>lock(io_);require_login();
  if(!six_axis_||!passthrough_||permitted_.load()||transport_.possible()||!state_cache_||!process_feedback_)
   throw std::runtime_error("Refined stream needs disarmed passthrough and independent state/joints");
  clock_batch_cushion_=12;clock_underrun_ticks_=12;clock_batch_min_=2;push_only_stream_=clock_reramp_=true;
 }
 py::dict configure_motion_limits(double velocity,double acceleration,double jerk,unsigned cushion,unsigned underrun_ticks=0){
  std::lock_guard<std::mutex>lock(io_);require_login();
  if(!push_only_stream_||!six_axis_||!passthrough_||permitted_.load()||transport_.possible())
   throw std::runtime_error("Motion limits need disarmed refined stream");
  bool valid=(velocity==10.&&acceleration==30.&&jerk==300.)||
    (velocity==25.&&acceleration==75.&&jerk==600.)||
    (velocity==35.&&acceleration==105.&&jerk==900.)||
    (velocity==45.&&acceleration==180.&&jerk==1800.);
  if(underrun_ticks==0)underrun_ticks=cushion;
  if(!valid||cushion<8||cushion>20||underrun_ticks!=cushion)throw std::invalid_argument("Unsupported motion profile or cushion");
  reference_velocity_deg_s_=velocity;reference_acceleration_deg_s2_=acceleration;reference_jerk_deg_s3_=jerk;
  clock_batch_cushion_=cushion;clock_effective_cushion_=cushion;clock_underrun_ticks_=underrun_ticks;
  py::dict d;d["velocity_deg_s"]=velocity;d["acceleration_deg_s2"]=acceleration;
  d["jerk_deg_s3"]=jerk;d["max_step_deg"]=velocity*.005;d["cushion_points"]=cushion;d["underrun_gap_s"]=underrun_ticks*.005;return d;
 }
 explicit J6Follower(bool six_axis=false,bool passthrough=false,double follow_hz=20.,bool inspect_profile=false):six_axis_(six_axis),passthrough_(passthrough),inspect_profile_(inspect_profile){
  if(passthrough&&!six_axis)throw std::invalid_argument("Passthrough requires six-axis scope");
  if((follow_hz!=20.&&follow_hz!=25.&&follow_hz!=50.&&follow_hz!=200.)||(follow_hz!=20.&&(!six_axis||passthrough)))
   throw std::invalid_argument("Follow rate requires six-axis follow and 20, 25, 50 or 200 Hz");
  follow_period_=1./follow_hz;
 }
 ~J6Follower(){if(abandoned_.load()){sdk_.release();new std::shared_ptr<PushBuffer>(push_buffer_);return;}if(connected_){try{
   if(push_enabled_)sdk_->robotServiceRegisterRealTimeRoadPointCallback(nullptr,nullptr);
   // Destruction/logout is not a substitute for exit+stop confirmation.
   // Do not retry an unknown exit or silently surrender an active mode here.
   if(!transport_.possible())sdk_->robotServiceLogout();}catch(...){}}sdk_.reset();}
 unsigned long long revoke(){permitted_.store(false);return ++revision_;}
 unsigned long long revision()const{return revision_.load();}
 void abandon_cleanup(){abandoned_.store(true);revoke();}
 void attach_state_observer(std::shared_ptr<RobotStateObserver>observer,bool redundant_waypoint_feedback=false){
  py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);require_login();
  if(!six_axis_||!push_enabled_||prepared_||state_observer_||state_cache_||!observer)
   throw std::runtime_error("Observer requires six-axis control with push, attach once before prepare");
  if(!state_observation_recent(observer->observation(),now_s()))
   throw std::runtime_error("Observer initial state missing or expired");
  if(redundant_waypoint_feedback)observer->attach_waypoint_buffer(push_buffer_);
  state_observer_=std::move(observer);
 }
 void attach_state_cache(std::shared_ptr<RobotStateCache>cache){
  py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);require_login();
  if(!six_axis_||!push_enabled_||prepared_||state_observer_||state_cache_||!cache)
   throw std::runtime_error("Process state cache requires six-axis control with push, attached once before prepare");
  if(!state_observation_recent(cache->observation(),now_s()))
   throw std::runtime_error("Process state cache initial observation missing or expired");
  state_cache_=std::move(cache);
 }
 void attach_process_feedback(std::shared_ptr<RobotStateCache>state,
                              std::shared_ptr<WaypointFeedbackCache>joints){
  py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);require_login();
  if(!six_axis_||push_enabled_||prepared_||state_observer_||state_cache_||!state||!joints)
   throw std::runtime_error("Process feedback requires an unsubscribed six-axis connection before prepare");
  if(!state_observation_recent(state->observation(),now_s()))
   throw std::runtime_error("Process feedback initial state missing or expired");
  auto buffer=joints->buffer();
  {std::lock_guard<std::mutex>p(buffer->mutex);
   double now=now_s();
   if(buffer->samples.empty()||now<buffer->samples.back().received||now-buffer->samples.back().received>.2)
    throw std::runtime_error("Process feedback initial joints missing or expired");}
  disable_realtime_pushes(*sdk_);
  state_cache_=std::move(state);process_feedback_=std::move(joints);push_readable_.store(true);
 }
 py::dict login(const std::string&host,int port,const std::string&user,const std::string&password){
  if(host.empty()||port<1||port>65535)throw std::invalid_argument("Invalid address");
  int rc;double s,e;
  {py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);
   if(sdk_)throw std::runtime_error("New client required; no reconnect");
   sdk_=std::make_unique<ServiceInterface>();s=now_s();rc=sdk_->robotServiceLogin(host.c_str(),port,user.c_str(),password.c_str());e=now_s();connected_=rc==0;}
  return record("login",rc,s,e);
 }
 py::dict logout(){
  revoke();push_readable_.store(false);int rc;double s,e;
  {py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);require_login();
   if(transport_.possible())throw std::runtime_error("Passthrough exit not confirmed; operator stop required before logout");
   if(push_enabled_){sdk_->robotServiceRegisterRealTimeRoadPointCallback(nullptr,nullptr);push_enabled_=false;}
   s=now_s();rc=sdk_->robotServiceLogout();e=now_s();connected_=false;}
  return record("logout",rc,s,e);
 }
 py::dict start_waypoint_feedback(){
  int rc;double s,e;
  {py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);require_login();
   if(push_enabled_||process_feedback_||prepared_)throw std::runtime_error("Subscribe once before prepare");
   {std::lock_guard<std::mutex>p(push_mutex_);pushes_.clear();push_count_=0;}
   s=now_s();rc=sdk_->robotServiceRegisterRealTimeRoadPointCallback(waypoint_callback,push_buffer_.get());e=now_s();
   push_enabled_=rc==0;push_readable_.store(push_enabled_);}
  return record("register_waypoint_feedback",rc,s,e);
 }
 py::dict waypoint_feedback(){
  if(!push_readable_.load())throw std::runtime_error("Not connected or waypoint feedback not enabled");
  return push_feedback(feedback_buffer());
 }
 py::dict waypoint_feedback_since(unsigned long long sequence){
  if(!push_readable_.load())throw std::runtime_error("Not connected or waypoint feedback not enabled");
  return push_feedback(feedback_buffer(),sequence);
 }
 py::dict snapshot(){
  a::JointParam q{};a::RobotState state{};int qr,sr;double s,qe,ss,e;
  bool pushed=false;
  {py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);require_login();
   s=now_s();
   if(state_observer_||state_cache_){
    auto v=refresh_parallel_observation();
    bool can_await=v.valid;
    for(double q:measured_)can_await=can_await&&std::isfinite(q);
    // A valid but aged observation can be awaited during post-stop confirmation.
    // It remains unusable by arm/send and must retain its original timestamps.
    if(!have_snapshot_&&!can_await)throw std::runtime_error("Parallel state/joint feedback missing, failed or expired; rc="+
      std::to_string(v.return_code)+", state="+std::to_string(v.state)+
      ", valid="+std::to_string(v.valid)+", state_age_s="+std::to_string(now_s()-v.received)+
      ", joint_age_s="+std::to_string(now_s()-joints_received_));
    pushed=true;qr=0;qe=joints_received_;ss=v.started;e=v.received;sr=v.return_code;
    state=static_cast<a::RobotState>(state_);
    for(int i=0;i<6;++i)q.jointPos[i]=measured_[i];
   }else{
   if(push_enabled_){std::lock_guard<std::mutex>p(push_mutex_);pushed=true;qr=-1;qe=s;
    if(!pushes_.empty()){const auto&v=pushes_.back();qr=0;qe=v.received;
     for(int i=0;i<6;++i)q.jointPos[i]=v.q[i];}}
   else{qr=sdk_->robotServiceGetJointAngleInfo(q);qe=now_s();}
   ss=now_s();sr=sdk_->robotServiceGetRobotCurrentState(state);e=now_s();
   // A synchronous state query can outlast several push frames. Use the
   // newest received joints after it returns, preserving their real timestamp.
   if(pushed){std::lock_guard<std::mutex>p(push_mutex_);
    for(auto it=pushes_.rbegin();it!=pushes_.rend();++it)if(it->received<=e){
     qr=0;qe=it->received;for(int i=0;i<6;++i)q.jointPos[i]=it->q[i];break;}}
   have_snapshot_=qr==0&&sr==0&&e-s<=SDK_CALL_BUDGET_S&&e-qe>=0&&e-qe<=joint_age_limit();
   for(int i=0;i<6;++i){measured_[i]=q.jointPos[i];have_snapshot_=have_snapshot_&&std::isfinite(measured_[i]);}
   observed_=pushed?qe:s;joints_received_=qe;state_received_=e;state_=int(state);
   if(!have_snapshot_)revoke();}}
  auto jq=record("joint_angles",qr,pushed?qe:s,qe);jq["data"]=qr==0?py::cast(std::vector<double>(q.jointPos,q.jointPos+6)):py::none();
  jq["source"]=pushed?"sdk_waypoint_push":"sdk_joint_query";
  auto st=record("robot_state",sr,ss,e);st["data"]=sr==0?py::cast(int(state)):py::none();
  if(state_observer_||state_cache_)st["source"]="independent_sdk_state_cache";
  py::dict d;d["joints"]=jq;d["state"]=st;
  d["state_polling_suppressed"]=stream_push_only();
  if((state_observer_||state_cache_)&&!have_snapshot_)d["observation_pending"]=true;
  if(pushed&&!state_observer_&&!state_cache_&&(qr!=0||e-qe>joint_age_limit())){py::dict err;err["type"]="RuntimeError";err["message"]="Waypoint feedback missing or expired";d["validation_error"]=err;}
  return d;
 }
 py::dict prepare(){
  int rc=0;double s,e;std::string refusal;
  a::RobotState state{};a::RobotDiagnosis diag{};a::JointRangeOfMotion ranges{};
  a::JointParam q{};a::JointVelcAccParam got_acc{},got_vel{};bool real=false;
  struct Check{std::string name;int rc;double s,e;};std::vector<Check> checks;
  {py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);require_login();
   if(prepared_||failed_)throw std::runtime_error("Prepare once per connection");
   s=now_s();
   auto check=[&](const char*name,auto call){double b=now_s();int c=call();double f=now_s();checks.push_back({name,c,b,f});
    if(c!=0||f-b>SDK_CALL_BUDGET_S){rc=c!=0?c:-1;refusal=std::string(name)+"_failed_or_late";return false;}return true;};
   bool ok=check("state",[&]{return sdk_->robotServiceGetRobotCurrentState(state);})&&
    check("real_robot",[&]{return sdk_->robotServiceGetIsRealRobotExist(real);})&&
    check("diagnosis",[&]{return sdk_->robotServiceGetRobotDiagnosisInfo(diag);})&&
    check("ranges",[&]{return sdk_->robotServiceGetJointRangeOfMotion(ranges);})&&
    check("joints",[&]{return sdk_->robotServiceGetJointAngleInfo(q);});
   if(ok&&(state!=a::RobotStopped||!real||!diag.armPowerStatus||diag.softEmergency||diag.remoteEmergency||diag.robotCollision||diag.jointErrorStatus||!ranges.enable)){
    ok=false;refusal="device_not_ready_or_range_disabled";}
   for(int i=0;i<6;++i){anchor_[i]=q.jointPos[i];
    if(ok&&(!std::isfinite(q.jointPos[i])||!std::isfinite(ranges.rangeValues[i].minValue)||!std::isfinite(ranges.rangeValues[i].maxValue)||
     ranges.rangeValues[i].minValue>=ranges.rangeValues[i].maxValue||q.jointPos[i]<ranges.rangeValues[i].minValue||
     q.jointPos[i]>ranges.rangeValues[i].maxValue)){ok=false;refusal="invalid_range_or_initial_position";}}
   if(ok){for(int i=0;i<6;++i){lower_[i]=ranges.rangeValues[i].minValue;upper_[i]=ranges.rangeValues[i].maxValue;}
    j6_min_=lower_[5];j6_max_=upper_[5];}
   if(ok)ok=prepare_follow_profile(*sdk_,six_axis_,inspect_profile_,J6_SPEED_DEG_S*DEG,CLIENT_ACC_DEG_S2*DEG,
                                  got_acc,got_vel,check,refusal);
   prepared_=ok;failed_=!ok;e=now_s();}
  auto d=record(six_axis_?"prepare_six_axis_follower":"prepare_j6_follower",prepared_?0:(rc?rc:-1),s,e);d["six_axis"]=six_axis_;d["prepared"]=prepared_;d["refusal_reason"]=refusal;
  d["anchor_rad"]=anchor_;py::list checks_out;for(auto&c:checks)checks_out.append(record(c.name.c_str(),c.rc,c.s,c.e));d["prechecks"]=checks_out;
  std::vector<std::array<double,2>> bounds;for(int i=0;i<6;++i)bounds.push_back({ranges.rangeValues[i].minValue,ranges.rangeValues[i].maxValue});d["joint_ranges_rad"]=bounds;
  d["client_speed_deg_s"]=six_axis_?py::object(py::none()):py::cast(J6_SPEED_DEG_S);
  d["client_acceleration_deg_s2"]=six_axis_?py::object(py::none()):py::cast(CLIENT_ACC_DEG_S2);
  d["sdk_profile_policy"]=six_axis_?"sdk_unmodified":"legacy_j6_explicit";
  d["command_mode"]=passthrough_?"passthrough":"follow";
  d["sdk_profile_writes"]=!six_axis_;
  py::dict readback;
  if(six_axis_&&!inspect_profile_){
   readback["max_velocity_rad_s"]=py::none();readback["max_acceleration_rad_s2"]=py::none();
   readback["source"]="not_read_six_axis_unmodified";
  }else{
   readback["max_velocity_rad_s"]=std::vector<double>(got_vel.jointPara,got_vel.jointPara+6);
   readback["max_acceleration_rad_s2"]=std::vector<double>(got_acc.jointPara,got_acc.jointPara+6);
   readback["source"]="sdk_client_memory_not_controller_settings";
  }
  d["sdk_profile_readback"]=readback;
  d["following_error_limit_deg"]=py::none();
  d["target_policy"]=six_axis_?"direct_ik":"j6_rate_limited";
  d["host_target_rate_limit_deg_s"]=six_axis_?py::object(py::none()):py::cast(J6_SPEED_DEG_S);
  d["send_period_s"]=six_axis_?(passthrough_?SIX_AXIS_SEND_PERIOD_S:follow_period_):SEND_PERIOD_S;d["send_max_gap_s"]=SEND_MAX_GAP_S;
  d["sdk_budget_s"]=SDK_CALL_BUDGET_S;
  d["profile_scope"]="SDK client memory only; controller/pendant settings and physical response not inferred";return d;
 }
 bool arm(unsigned long long token){
  py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);require_login();
  if(state_observer_||state_cache_)refresh_parallel_observation();
  if(!prepared_||failed_||!have_snapshot_||state_!=0||now_s()-observed_<0||now_s()-observed_>.2||now_s()-state_received_<0||now_s()-state_received_>.2||token!=revision_.load())return false;
  if(!six_axis_)for(int i=0;i<5;++i)if(std::abs(measured_[i]-anchor_[i])>.1*DEG)return false;
  for(int i=0;i<6;++i)if(measured_[i]<lower_[i]||measured_[i]>upper_[i])return false;
  last_target_=measured_;
  last_offset_=(measured_[5]-anchor_[5])/DEG;last_send_=now_s();permitted_.store(true);
  // A concurrent revoke must win even if it arrived between the check and store.
  if(token!=revision_.load()){permitted_.store(false);return false;}return true;
 }
 py::dict enter_passthrough(unsigned long long token){
  int rc;double s,e;
  {py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);require_login();
   if(!passthrough_||!prepared_||failed_||!permitted_.load()||token!=revision_.load())
    throw std::runtime_error("Passthrough entry permission revoked or wrong mode");
   if(last_leave_>0&&now_s()-last_leave_<.5)throw std::runtime_error("Cold entry cooldown not complete");
   priming_anchor_=last_target_;priming_steps_=priming_fast_=slow_sends_=0;
   clock_batch_epoch_=0;clock_batch_sent_=clock_batch_lost_=0;
   clock_recovery_pending_=false;clock_restarts_=0;
   clock_effective_cushion_=clock_batch_cushion_;clock_warmup_sent_=0;
   clock_warmup_complete_=clock_small_restart_=clock_stationary_=false;
   s=now_s();rc=transport_.enter(*sdk_);e=now_s();
   priming_=rc==0;priming_started_=e;
   if(rc!=0||e-s>SDK_CALL_BUDGET_S){failed_=true;revoke();}}
  auto d=record("enter_tcp2canbus",rc,s,e);
  d["anchor_rad"]=priming_anchor_;
  d["revoked_during_call"]=token!=revision_.load();return d;
 }
 py::dict send_priming(unsigned long long token){
  int rc=-1;double s,e;
  {py::gil_scoped_release release;std::lock_guard<std::mutex>lock(io_);require_login();
   if(!priming_||!permitted_.load()||token!=revision_.load())
    throw std::runtime_error("Priming permission revoked or wrong phase");
   if(state_observer_||state_cache_)refresh_parallel_observation();
   double now=now_s();
   if(now-priming_started_>2.5){revoke();throw std::runtime_error("Priming deadline expired");}
   bool valid=have_snapshot_&&(state_==0||state_==1)&&now-priming_started_<=2.5&&
    now-joints_received_>=0&&now-joints_received_<=joint_age_limit()&&state_fresh(now,.4);
   for(int i=0;i<6;++i)valid=valid&&std::isfinite(measured_[i])&&std::abs(measured_[i]-priming_anchor_[i])<=.1*DEG;
   if(!valid){revoke();throw std::runtime_error("Priming age/deadline/pose violated");}
   s=now_s();rc=transport_.send(*sdk_,priming_anchor_.data());e=now_s();last_send_=s;
   last_target_=priming_anchor_;++priming_steps_;
   priming_fast_=rc==0&&e-s<=.04?priming_fast_+1:0;
   if(push_only_stream_)slow_sends_=e-s>.25?slow_sends_+1:0;
   if(rc!=0||e-s>(push_only_stream_?.5:.4)||(push_only_stream_&&slow_sends_>=3)){failed_=true;revoke();}
   else if(e-priming_started_>2.5)revoke();
  }
  auto d=record("priming_fixed_hold",rc,s,e);d["target_rad"]=priming_anchor_;
  d["steps"]=priming_steps_;d["consecutive_fast"]=priming_fast_;
  d["revoked_during_call"]=token!=revision_.load();return d;
 }
 bool finish_priming(unsigned long long token){
  py::gil_scoped_release release;std::lock_guard<std::mutex>lock(io_);require_login();
  if(!priming_||!permitted_.load()||token!=revision_.load())return false;
  if(state_observer_||state_cache_)refresh_parallel_observation();
  double now=now_s();bool valid=priming_steps_>=12&&priming_fast_>=5&&
    now-priming_started_<=2.5&&have_snapshot_&&(state_==0||state_==1)&&
    state_fresh(now,.1)&&now-joints_received_>=0&&now-joints_received_<=.06;
  for(int i=0;i<6;++i)valid=valid&&std::abs(measured_[i]-priming_anchor_[i])<=.1*DEG;
  if(valid)priming_=false;
  return valid;
 }
 py::dict send_offset(double offset,unsigned long long token){
  int rc=-1;double s=0,e=0,checked=0,dt=0,observed=0,joint_received=0,state_received=0,previous_offset=0;
  int measured_state=-1;bool invoked=false;std::string refusal;std::array<double,6>target,measured;
  {py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);require_login();
   if(six_axis_)throw std::runtime_error("Six-axis client requires send_joints");
   double now=now_s();dt=now-last_send_;
   if(!prepared_||failed_||!permitted_.load()||token!=revision_.load())throw std::runtime_error("Send permission revoked");
   checked=now;observed=observed_;joint_received=joints_received_;state_received=state_received_;
   previous_offset=last_offset_;measured=measured_;measured_state=state_;s=e=now;
   target=anchor_;target[5]=std::clamp(anchor_[5]+offset*DEG,j6_min_,j6_max_);
   if(!have_snapshot_||(state_!=0&&state_!=1)||now-observed_>.2||!std::isfinite(offset)||
    offset<(j6_min_-anchor_[5])/DEG||offset>(j6_max_-anchor_[5])/DEG||dt<=0||dt>SEND_MAX_GAP_S||
    std::abs(offset-last_offset_)>J6_SPEED_DEG_S*std::min(dt,SEND_PERIOD_S)+1e-8)refusal="J6 target age/range/step budget violated";
   for(int i=0;i<5;++i)if(std::abs(measured_[i]-anchor_[i])>.1*DEG)refusal="Non-J6 measurement left initial envelope";
   if(!refusal.empty())revoke();
   else{
    // Final local permission check; an already-entered SDK call cannot be cancelled.
    if(!permitted_.load()||token!=revision_.load())throw std::runtime_error("Send permission revoked");
    invoked=true;s=now_s();rc=sdk_->robotServiceFollowModeJointMove(target.data());e=now_s();last_send_=s;last_offset_=offset;
    if(rc!=0||e-s>SDK_CALL_BUDGET_S){failed_=true;revoke();}}}
  auto d=record("follow_j6_target",rc,s,e);d["target_rad"]=target;d["offset_deg"]=offset;
  if(!invoked)d["return_code"]=py::none();
  d["sdk_invoked"]=invoked;d["local_refusal_reason"]=refusal;
  py::dict check;check["checked_monotonic_s"]=checked;check["snapshot_started_monotonic_s"]=observed;
  check["joints_received_monotonic_s"]=joint_received;check["state_received_monotonic_s"]=state_received;
  check["measured_q_rad"]=measured;check["robot_state"]=measured_state;check["previous_offset_deg"]=previous_offset;
  check["command_feedback_error_deg"]=(target[5]-measured[5])/DEG;
  check["following_error_limit_deg"]=py::none();check["send_interval_s"]=dt;
  d["pre_send"]=check;
  d["revoked_during_call"]=invoked&&token!=revision_.load();return d;
 }
 py::dict send_joints(std::array<double,6>target,unsigned long long token){
  int rc=-1,measured_state=-1;bool invoked=false;std::string refusal;
  double s=0,e=0,checked=0,dt=0,jr=0,sr=0;std::array<double,6>measured,previous,error;
  {py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);require_login();
   if(!six_axis_)throw std::runtime_error("J6 client cannot send six-axis targets");
   if(priming_)throw std::runtime_error("Priming accepts only immutable measured hold");
   if(!prepared_||failed_||!permitted_.load()||token!=revision_.load())throw std::runtime_error("Send permission revoked");
   if(state_observer_||state_cache_)refresh_parallel_observation();
   checked=now_s();dt=checked-last_send_;s=e=checked;measured=measured_;previous=last_target_;
   jr=joints_received_;sr=state_received_;measured_state=state_;
   if(!have_snapshot_||(state_!=0&&state_!=1)||checked-jr<0||checked-jr>(passthrough_?.5:.2)||
      checked-sr<0||checked-sr>(passthrough_?.5:.2)||dt<=0||dt>SEND_MAX_GAP_S)refusal="Six-axis feedback/state age or send interval violated";
   for(int i=0;i<6;++i){
    error[i]=(target[i]-measured[i])/DEG;
    if(!std::isfinite(target[i])||target[i]<lower_[i]||target[i]>upper_[i])refusal="Six-axis target nonfinite or outside controller range";
    if(!std::isfinite(measured[i])||measured[i]<lower_[i]-.1*DEG||measured[i]>upper_[i]+.1*DEG)refusal="Six-axis measured joints outside controller range";
   }
   if(!refusal.empty())revoke();
   else{
    if(!permitted_.load()||token!=revision_.load())throw std::runtime_error("Send permission revoked");
    invoked=true;s=now_s();rc=passthrough_?transport_.send(*sdk_,target.data()):sdk_->robotServiceFollowModeJointMove(target.data());e=now_s();last_send_=s;last_target_=target;
    if(passthrough_)slow_sends_=e-s>.25?slow_sends_+1:0;
    if(rc!=0||e-s>SDK_CALL_BUDGET_S||(passthrough_&&slow_sends_>=3)){failed_=true;revoke();}
   }
  }
  auto d=record(passthrough_?"passthrough_six_axis_target":"follow_six_axis_target",rc,s,e);if(!invoked)d["return_code"]=py::none();
  d["consecutive_slow_sends"]=slow_sends_;d["target_rad"]=target;d["sdk_invoked"]=invoked;d["local_refusal_reason"]=refusal;
  py::dict check;check["checked_monotonic_s"]=checked;check["joints_received_monotonic_s"]=jr;
  check["state_received_monotonic_s"]=sr;check["measured_q_rad"]=measured;check["previous_target_rad"]=previous;
  check["robot_state"]=measured_state;check["command_feedback_error_deg"]=error;
  check["following_error_limit_deg"]=py::none();check["send_interval_s"]=dt;d["pre_send"]=check;
  d["revoked_during_call"]=invoked&&token!=revision_.load();return d;
 }
 py::dict query_diagnosis(){
  a::RobotDiagnosis v{};int rc;double s,e;
  {py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);require_login();
   s=now_s();rc=sdk_->robotServiceGetRobotDiagnosisInfo(v);e=now_s();
   if(rc==0&&v.macTargetPosDataSize%6==0){batch_pending_upper_=v.macTargetPosDataSize/6;batch_diagnosis_received_=e;}
   else batch_diagnosis_received_=0;}
  auto d=record("robot_diagnosis",rc,s,e);py::dict data;
  data["orpe_status_raw"]=bool(v.orpeStatus);data["buffer_capacity_raw"]=int(v.macTargetPosBufferSize);
  data["buffer_data_raw"]=int(v.macTargetPosDataSize);data["interrupt_warning_raw"]=int(v.macDataInterruptWarning);
  data["power_on"]=bool(v.armPowerStatus);data["collision"]=bool(v.robotCollision);
  data["joint_error"]=bool(v.jointErrorStatus);d["data"]=data;return d;
 }
 py::dict clock_batch_plan(unsigned long long token,unsigned limit,bool stationary=false){
  if(limit<clock_batch_min_||limit>8)throw std::invalid_argument("Clock batch maximum outside configured minimum..8");
  std::lock_guard<std::mutex>lock(io_);require_login();
  if(priming_||!prepared_||failed_||!passthrough_||!permitted_.load()||token!=revision_.load())
   throw std::runtime_error("Clock batch permission revoked or wrong mode");
  double checked=now_s();if(clock_batch_epoch_<=0)clock_batch_epoch_=checked;
  // Preserve one point ledger across ACTIVE/BRAKING/HOLDING. A transition
  // alone never creates a second cushion on top of outstanding hold points.
  clock_stationary_=clock_reramp_&&stationary;
  unsigned cushion=clock_reramp_?clock_effective_cushion_:clock_batch_cushion_;
  auto elapsed=static_cast<unsigned long long>(std::floor(std::max(0.,checked-clock_batch_epoch_)/.005));
  auto target=static_cast<unsigned long long>(std::max(0.,std::floor((checked-clock_batch_epoch_)/.005)+cushion));
  auto due=target>clock_batch_sent_?target-clock_batch_sent_:0;
  auto missing=elapsed>clock_batch_sent_?elapsed-clock_batch_sent_:0;
  unsigned long long lost=0;bool stationary_rebased=false;
  if(clock_reramp_&&stationary&&(clock_recovery_pending_||(clock_warmup_complete_&&due>4))){
   // All references are identical with zero v/a: discard wall debt, not a
   // moving curve segment. Never wait for a quiet window or prefill 12 again.
   lost=missing;clock_batch_lost_+=lost;
   clock_batch_epoch_=checked+(cushion-2)*.005-clock_batch_sent_*.005;
   due=2;clock_recovery_pending_=false;stationary_rebased=true;
  }else if(clock_reramp_&&!stationary){
   // After prefill, any net wall-time debt may mean the FIFO emptied.
   // Do not require a second gap threshold: several short calls can drain
   // the cushion too. Partial prefill/restart never triggers itself.
   if(clock_recovery_pending_||(clock_warmup_complete_&&missing>0))
    clock_recovery_pending_=true;
  }else if(!clock_reramp_&&due>8){
   lost=due-8;clock_batch_lost_+=lost;
   clock_batch_epoch_+=lost*.005;due=8;
  }
  unsigned cap=clock_small_restart_&&!clock_warmup_complete_?std::min(limit,4u):limit;
  unsigned count=!clock_recovery_pending_&&due>=clock_batch_min_?std::min<unsigned long long>(cap,due):0;
  py::dict d;d["point_count"]=count;d["epoch_monotonic_s"]=clock_batch_epoch_;
  d["checked_monotonic_s"]=checked;d["sent_points"]=clock_batch_sent_;
  d["due_points"]=due;d["cushion_points"]=cushion;
  d["nominal_cushion_points"]=clock_batch_cushion_;d["underrun_gap_s"]=clock_underrun_ticks_*.005;d["net_missing_points"]=missing;
  d["stationary_rebased"]=stationary_rebased;d["warmup_complete"]=clock_warmup_complete_;
  d["discarded_wall_ticks"]=lost;d["total_discarded_wall_ticks"]=clock_batch_lost_;
  d["recovery_required"]=clock_recovery_pending_;d["restart_count"]=clock_restarts_;
  d["minimum_points"]=clock_batch_min_;
  d["next_due_monotonic_s"]=clock_batch_epoch_+(clock_batch_sent_+double(clock_batch_min_)-cushion)*.005;
  d["consumer_period_verified"]=false;return d;
 }
 py::dict restart_clock_from_rest(unsigned long long token){
  std::lock_guard<std::mutex>lock(io_);require_login();
  if(!clock_reramp_||!clock_recovery_pending_||!permitted_.load()||token!=revision_.load())
   throw std::runtime_error("Clock recovery not pending or permit revoked");
  refresh_parallel_observation();double now=now_s();
  if(!have_snapshot_||now<joints_received_||now-joints_received_>.5||
     (state_!=0&&state_!=1)||!state_fresh(now,.5))
   throw std::runtime_error("Clock recovery feedback not valid or fresh");
  // Reference restart, not a measurement or a declaration of physical rest.
  auto elapsed=static_cast<unsigned long long>(std::floor(std::max(0.,now-clock_batch_epoch_)/.005));
  if(elapsed>clock_batch_sent_)clock_batch_lost_+=elapsed-clock_batch_sent_;
  clock_batch_epoch_=now-clock_batch_sent_*.005;clock_recovery_pending_=false;++clock_restarts_;
  clock_effective_cushion_=2;clock_warmup_sent_=0;
  clock_warmup_complete_=false;clock_small_restart_=true;clock_stationary_=false;
  py::dict d;d["q_restart_rad"]=last_target_;d["epoch_monotonic_s"]=clock_batch_epoch_;
  d["sent_points"]=clock_batch_sent_;d["restart_count"]=clock_restarts_;
  d["discarded_wall_ticks"]=clock_batch_lost_;d["initial_points"]=2;return d;
 }
 py::dict send_batch(std::vector<std::array<double,6>>points,unsigned long long token,bool clocked=false){
  if(points.empty()||points.size()>8)throw std::invalid_argument("Batch requires 1..8 fixed-tick points");
  int rc=-1;double s,e,jr,sr,checked;std::array<double,6>previous,measured;
  {py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);require_login();
   if(!six_axis_||!passthrough_||priming_||!prepared_||failed_||!permitted_.load()||token!=revision_.load())
    throw std::runtime_error("Batch send permission revoked or wrong mode");
   if(state_observer_||state_cache_)refresh_parallel_observation();
   checked=now_s();jr=joints_received_;sr=state_received_;previous=last_target_;measured=measured_;
   if(!have_snapshot_||(state_!=0&&state_!=1)||checked-jr<0||checked-jr>.5||!state_fresh(checked,.5))
    {revoke();throw std::runtime_error("Batch feedback/state age violated");}
   if(clocked){
    if(clock_batch_epoch_<=0)throw std::runtime_error("Clock batch plan missing");
    auto elapsed=static_cast<unsigned long long>(std::floor(std::max(0.,checked-clock_batch_epoch_)/.005));
    if(clock_reramp_&&!clock_stationary_&&clock_warmup_complete_&&elapsed>clock_batch_sent_)
     clock_recovery_pending_=true;
    if(clock_recovery_pending_)throw std::runtime_error("Clock recovery required before sending");
    auto target=static_cast<unsigned long long>(std::max(0.,std::floor((checked-clock_batch_epoch_)/.005)+(clock_reramp_?clock_effective_cushion_:clock_batch_cushion_)));
    if(clock_batch_sent_+points.size()>target){revoke();throw std::runtime_error("Clock batch exceeds wall schedule plus cushion");}
   }else if(batch_diagnosis_received_<=0||checked-batch_diagnosis_received_>.05||batch_pending_upper_+points.size()>8)
    {revoke();throw std::runtime_error("Batch queue reservation missing, stale or beyond 40ms bound");}
   for(int i=0;i<6;++i)if(!std::isfinite(measured_[i])||measured_[i]<lower_[i]-.1*DEG||measured_[i]>upper_[i]+.1*DEG)
    {revoke();throw std::runtime_error("Batch measured joints outside controller range");}
   auto last=previous;
   std::vector<a::wayPoint_S>batch;batch.reserve(points.size());
   for(const auto&q:points){a::wayPoint_S point{};
    for(int i=0;i<6;++i){
     if(!std::isfinite(q[i])||q[i]<lower_[i]||q[i]>upper_[i]||std::abs(q[i]-last[i])>reference_velocity_deg_s_*DEG*.005+1e-10)
      {revoke();throw std::runtime_error("Batch point range or fixed-tick step violated");}
     point.jointpos[i]=q[i];}
    batch.push_back(point);last=q;}
   if(!permitted_.load()||token!=revision_.load())throw std::runtime_error("Send permission revoked");
   if(clocked)clock_batch_sent_+=points.size();else batch_pending_upper_+=points.size();
   s=now_s();rc=transport_.send_batch(*sdk_,batch);e=now_s();last_send_=s;last_target_=points.back();
   if(clocked&&clock_reramp_){
    clock_warmup_sent_+=points.size();
    if(clock_small_restart_&&clock_effective_cushion_<clock_batch_cushion_)
     clock_effective_cushion_=std::min(clock_batch_cushion_,clock_effective_cushion_+2);
    clock_warmup_complete_=clock_warmup_sent_>=clock_batch_cushion_&&clock_effective_cushion_==clock_batch_cushion_;
   }
   slow_sends_=e-s>.25?slow_sends_+1:0;
   if(rc!=0||e-s>SDK_CALL_BUDGET_S||slow_sends_>=3){failed_=true;revoke();}
  }
  auto d=record("passthrough_fixed_tick_batch",rc,s,e);d["sdk_invoked"]=true;
  d["target_rad"]=points.back();d["points_rad"]=points;d["point_count"]=points.size();d["max_step_deg"]=reference_velocity_deg_s_*.005;
  d["pending_upper_after_submit"]=clocked?py::none():py::cast(batch_pending_upper_);d["tick_s"]=.005;d["batch_span_s"]=points.size()*.005;
  d["wall_clock_paced"]=clocked;d["clock_sent_points"]=clock_batch_sent_;
  d["clock_epoch_monotonic_s"]=clock_batch_epoch_;d["discarded_wall_ticks"]=clock_batch_lost_;
  d["clock_cushion_points"]=clock_reramp_?clock_effective_cushion_:clock_batch_cushion_;d["clock_minimum_points"]=clock_batch_min_;
  d["clock_restart_count"]=clock_restarts_;
  d["consecutive_slow_sends"]=slow_sends_;d["revoked_during_call"]=token!=revision_.load();
  py::dict pre;pre["checked_monotonic_s"]=checked;pre["joints_received_monotonic_s"]=jr;
  pre["state_received_monotonic_s"]=sr;pre["previous_target_rad"]=previous;pre["measured_q_rad"]=measured;
  d["pre_send"]=pre;return d;
 }
 py::dict stop(){
  revoke();int rc,leave_rc=0;double s,e,ls=0,le=0;bool leave_attempted=false;
  {py::gil_scoped_release r;std::lock_guard<std::mutex>lock(io_);require_login();
   if(transport_.possible()){
    leave_attempted=true;ls=now_s();leave_rc=transport_.leave(*sdk_);le=now_s();last_leave_=le;
    if(leave_rc!=0)failed_=true;
   }
   priming_=false;
   s=now_s();rc=aubo_stop_compat::request_slow_stop(*sdk_);e=now_s();
   // ACK only: Python must confirm fresh stopped state and quiet joints.
   // Do not hold the SDK completion mutex across the response wait.
   if(rc!=0)failed_=true;}
  auto d=record("slow_control_stop",leave_rc!=0?leave_rc:rc,leave_attempted?ls:s,e);
  d["slow_stop_result"]=record("slow_control_stop",rc,s,e);
  d["stop_request_policy"]="slow_stop_ack_then_fresh_feedback";
  d["leave_passthrough_result"]=leave_attempted?py::object(record("leave_tcp2canbus",leave_rc,ls,le)):py::object(py::none());
  d["passthrough_exit_confirmed"]=!transport_.possible();return d;
 }
};
PYBIND11_MODULE(_aubo_follow,m){
 m.doc()="Supervised follower; serial motion connection with optional independent state connection; no startup/home";
 m.attr("following_error_limit_deg")=py::none();
 m.attr("six_axis_target_policy")="direct_ik";
 m.attr("six_axis_sdk_profile_policy")="sdk_unmodified";
 m.attr("supports_passthrough")=true;
 m.attr("supports_fixed_tick_batch")=true;
 m.attr("supports_wall_clock_batch")=true;
 m.attr("supports_motion_profiles")=true;
 m.attr("supports_immediate_hot_clutch")=true;
 m.attr("supports_refined_stream")=true;
 m.attr("supports_graded_hot_feedback")=true;
 m.attr("hot_state_max_age_s")=.5;m.attr("hot_joint_max_age_s")=.5;
 m.attr("supports_priming")=true;
 m.attr("supports_hot_clutch")=true;
 m.attr("priming_max_s")=2.5;
 m.attr("priming_fast_s")=.04;
 m.attr("supports_follow_rate_selection")=true;
 m.attr("supports_parallel_state_query")=true;
 m.attr("supports_process_state_query")=true;
 m.attr("supports_process_waypoint_feedback")=true;
 m.attr("supports_redundant_waypoint_feedback")=true;
 m.attr("client_speed_deg_s")=J6_SPEED_DEG_S;
 m.attr("client_acceleration_deg_s2")=CLIENT_ACC_DEG_S2;
 m.attr("send_period_s")=SEND_PERIOD_S;m.attr("send_max_gap_s")=SEND_MAX_GAP_S;
 m.attr("six_axis_send_period_s")=SIX_AXIS_SEND_PERIOD_S;
 m.attr("six_axis_follow_period_s")=SIX_AXIS_FOLLOW_PERIOD_S;
 m.attr("sdk_budget_s")=SDK_CALL_BUDGET_S;
 py::class_<RobotStateObserver,std::shared_ptr<RobotStateObserver>>(m,"RobotStateObserver")
  .def(py::init<>()).def("login",&RobotStateObserver::login).def("logout",&RobotStateObserver::logout)
  .def("query_state",&RobotStateObserver::query_state).def("invalidate",&RobotStateObserver::invalidate)
  .def("start_waypoint_feedback",&RobotStateObserver::start_waypoint_feedback)
  .def("waypoint_feedback",&RobotStateObserver::waypoint_feedback)
  .def("waypoint_feedback_since",&RobotStateObserver::waypoint_feedback_since)
  .def("abandon_cleanup",&RobotStateObserver::abandon_cleanup);
 py::class_<RobotStateCache,std::shared_ptr<RobotStateCache>>(m,"RobotStateCache")
  .def(py::init<>()).def("invalidate",&RobotStateCache::invalidate).def("publish",&RobotStateCache::publish);
 py::class_<WaypointFeedbackCache,std::shared_ptr<WaypointFeedbackCache>>(m,"WaypointFeedbackCache")
  .def(py::init<const std::string&>(),py::arg("source")="B").def("invalidate",&WaypointFeedbackCache::invalidate)
  .def("publish",&WaypointFeedbackCache::publish).def("feedback",&WaypointFeedbackCache::feedback);
 py::class_<J6Follower>(m,"J6Follower").def(py::init<bool,bool,double,bool>(),py::arg("six_axis")=false,py::arg("passthrough")=false,py::arg("follow_hz")=20.,py::arg("inspect_profile")=false).def("login",&J6Follower::login).def("logout",&J6Follower::logout)
  .def("attach_state_observer",&J6Follower::attach_state_observer,py::arg("observer"),
       py::arg("redundant_waypoint_feedback")=false).def("abandon_cleanup",&J6Follower::abandon_cleanup)
  .def("attach_state_cache",&J6Follower::attach_state_cache)
  .def("attach_process_feedback",&J6Follower::attach_process_feedback)
  .def("enter_passthrough",&J6Follower::enter_passthrough)
  .def("send_priming",&J6Follower::send_priming).def("finish_priming",&J6Follower::finish_priming)
  .def("prepare",&J6Follower::prepare).def("snapshot",&J6Follower::snapshot).def("arm",&J6Follower::arm)
  .def("start_waypoint_feedback",&J6Follower::start_waypoint_feedback).def("waypoint_feedback",&J6Follower::waypoint_feedback)
  .def("waypoint_feedback_since",&J6Follower::waypoint_feedback_since)
  .def("send_batch",&J6Follower::send_batch,py::arg("points"),py::arg("token"),py::arg("clocked")=false)
  .def("clock_batch_plan",&J6Follower::clock_batch_plan,py::arg("token"),py::arg("limit"),py::arg("stationary")=false)
  .def("configure_refined_stream",&J6Follower::configure_refined_stream)
  .def("configure_motion_limits",&J6Follower::configure_motion_limits,py::arg("velocity"),py::arg("acceleration"),py::arg("jerk"),py::arg("cushion"),py::arg("underrun_ticks")=0)
  .def("restart_clock_from_rest",&J6Follower::restart_clock_from_rest)
  .def("query_diagnosis",&J6Follower::query_diagnosis)
  .def("revision",&J6Follower::revision).def("revoke",&J6Follower::revoke).def("send_offset",&J6Follower::send_offset).def("send_joints",&J6Follower::send_joints).def("stop",&J6Follower::stop);
}
