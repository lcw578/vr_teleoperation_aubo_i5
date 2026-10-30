#pragma once
// Offline SDK double for compiling the actual binding, never linked to vendor SDK.
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
namespace aubo_robot_namespace {
struct JointParam{double jointPos[6]{};};
struct JointVelcAccParam{double jointPara[6]{1,1,1,1,1,1};};
struct wayPoint_S{double jointpos[6]{};};
enum RobotState{RobotStopped=0,RobotRunning=1,RobotPaused=2,RobotResumed=3};
enum RobotMoveControlCommand{RobotMoveStop=0};
struct RobotDiagnosis{bool armPowerStatus=true,softEmergency=false,remoteEmergency=false,robotCollision=false,jointErrorStatus=false;};
struct JointRangeOfMotion{bool enable=true;struct Range{double minValue=-3,maxValue=3;}rangeValues[6];};
}
namespace fake_follow {
inline std::atomic<int> instances{0},state_queries_a{0},state_queries_b{0},sends{0},stops{0};
inline std::atomic<int> state{0},query_rc{0};
inline std::atomic<bool> block_b{false},b_entered{false};
inline std::atomic<bool> push_enabled{true};
inline std::atomic<bool> push_enabled_b{true};
inline std::mutex periodic_push_mutex;
inline std::atomic<double> joint{0};
using Callback=void(*)(const aubo_robot_namespace::wayPoint_S*,void*);
inline std::atomic<bool> push_on_observation_read{false};
inline std::atomic<int> interleaved_pushes{0};
inline std::atomic<double> next_push_time_offset{0};
inline thread_local double injected_push_time_offset=0;
inline double push_timestamp(double received){return received+injected_push_time_offset;}
inline void before_push_read(Callback callback,void*arg){
 if(push_on_observation_read.exchange(false)){
  aubo_robot_namespace::wayPoint_S p;for(auto&q:p.jointpos)q=joint.load();
  injected_push_time_offset=next_push_time_offset.exchange(0);
  callback(&p,arg);injected_push_time_offset=0;++interleaved_pushes;
 }
}
}
class ServiceInterface {
 int id_=++fake_follow::instances;
 using Callback=void(*)(const aubo_robot_namespace::wayPoint_S*,void*);
 Callback callback_=nullptr;void*arg_=nullptr;
 std::atomic<bool>quit_{false};std::thread push_;
 aubo_robot_namespace::JointVelcAccParam acc_,vel_;
 void push_one(){aubo_robot_namespace::wayPoint_S p;for(auto&q:p.jointpos)q=fake_follow::joint.load();callback_(&p,arg_);}
 public:
 ~ServiceInterface(){robotServiceLogout();}
 int robotServiceLogin(const char*,int,const char*,const char*){return 0;}
 int robotServiceLogout(){quit_=true;if(push_.joinable())push_.join();return 0;}
 int robotServiceRegisterRealTimeRoadPointCallback(Callback cb,void*arg){
  quit_=true;if(push_.joinable())push_.join();callback_=cb;arg_=arg;
  if(cb){quit_=false;push_one();push_=std::thread([this]{while(!quit_){
   {std::lock_guard<std::mutex>lock(fake_follow::periodic_push_mutex);
    if((id_%2==1?fake_follow::push_enabled:fake_follow::push_enabled_b))push_one();}
   std::this_thread::sleep_for(std::chrono::milliseconds(2));}});}return 0;
 }
 int robotServiceGetRobotCurrentState(aubo_robot_namespace::RobotState&v){
  if(id_==1)++fake_follow::state_queries_a;
  else{++fake_follow::state_queries_b;fake_follow::b_entered=true;
   while(fake_follow::block_b)std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  v=static_cast<aubo_robot_namespace::RobotState>(fake_follow::state.load());return fake_follow::query_rc;
 }
 int robotServiceGetJointAngleInfo(aubo_robot_namespace::JointParam&q){for(auto&v:q.jointPos)v=fake_follow::joint;return 0;}
 int robotServiceGetRobotDiagnosisInfo(aubo_robot_namespace::RobotDiagnosis&){return 0;}
 int robotServiceGetJointRangeOfMotion(aubo_robot_namespace::JointRangeOfMotion&){return 0;}
 int robotServiceGetIsRealRobotExist(bool&v){v=true;return 0;}
 int robotServiceFollowModeJointMove(double*){++fake_follow::sends;return 0;}
 int rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveControlCommand){++fake_follow::stops;return 0;}
 int robotServiceEnterTcp2CanbusMode(){return 0;}
 int robotServiceLeaveTcp2CanbusMode(){return 0;}
 int robotServiceSetRobotPosData2Canbus(double*){return 0;}
 int robotServiceInitGlobalMoveProfile(){return 0;}
 int robotServiceSetGlobalMoveJointMaxAcc(aubo_robot_namespace::JointVelcAccParam v){acc_=v;return 0;}
 int robotServiceSetGlobalMoveJointMaxVelc(aubo_robot_namespace::JointVelcAccParam v){vel_=v;return 0;}
 int robotServiceGetGlobalMoveJointMaxAcc(aubo_robot_namespace::JointVelcAccParam&v){v=acc_;return 0;}
 int robotServiceGetGlobalMoveJointMaxVelc(aubo_robot_namespace::JointVelcAccParam&v){v=vel_;return 0;}
};
