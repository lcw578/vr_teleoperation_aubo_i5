// Supervised single-target and bounded J6 sequence experiments, not a general follower.
// Serial SDK calls intentionally do not assume same-instance concurrency support.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "aubo_sdk/serviceinterface.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
namespace py = pybind11;
namespace a = aubo_robot_namespace;
constexpr double DEG = 0.017453292519943295;
static double now_s() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static py::dict result(const std::string &op,int rc,double start,double end,py::object data=py::none()) {
  py::dict d;d["operation"]=op;d["return_code"]=rc;d["duration_s"]=end-start;
  d["host_started_monotonic_s"]=start;d["host_received_monotonic_s"]=end;
  d["controller_sample_time"]=py::none();d["data"]=rc==0?data:py::none();return d;
}
enum class TrialKind {Single, Sequence5Hz, Reversal10Deg, Sequence10Hz, Sequence20Hz};
static TrialKind sequence_kind(int rate) {
  if(rate==5) return TrialKind::Sequence5Hz;
  if(rate==10) return TrialKind::Sequence10Hz;
  if(rate==20) return TrialKind::Sequence20Hz;
  throw std::invalid_argument("Only fixed 5, 10 or 20 Hz trials supported");
}
class J6Trial {
 public:
  J6Trial():J6Trial(TrialKind::Single) {}
  virtual ~J6Trial() {if(connected_) {try {sdk_->robotServiceLogout();} catch(...) {}}}
  py::dict login(const std::string &host,int port,const std::string &user,const std::string &password) {
    if(host.empty()||port<1||port>65535) throw std::invalid_argument("Invalid address");
    int rc;double s,e;
    {py::gil_scoped_release release;std::lock_guard<std::mutex> lock(io_);
      if(sdk_) throw std::runtime_error("New client required; no reconnect");
      sdk_=std::make_unique<ServiceInterface>();s=now_s();
      rc=sdk_->robotServiceLogin(host.c_str(),port,user.c_str(),password.c_str());e=now_s();connected_=rc==0;}
    return result("login",rc,s,e);
  }
  py::dict logout() {
    int rc;double s,e;
    {py::gil_scoped_release release;std::lock_guard<std::mutex> lock(io_);require_login();
      s=now_s();rc=sdk_->robotServiceLogout();e=now_s();connected_=false;}
    return result("logout",rc,s,e);
  }
  py::dict snapshot() {
    a::JointParam q{};a::RobotState state{};int qr,sr;double qs,qe,ss,se;
    {py::gil_scoped_release release;std::lock_guard<std::mutex> lock(io_);require_login();
      qs=now_s();qr=sdk_->robotServiceGetJointAngleInfo(q);qe=now_s();
      ss=now_s();sr=sdk_->robotServiceGetRobotCurrentState(state);se=now_s();}
    py::dict d;d["joints"]=result("joint_angles",qr,qs,qe,py::cast(std::vector<double>(q.jointPos,q.jointPos+6)));
    d["state"]=result("robot_state",sr,ss,se,py::int_(int(state)));return d;
  }
  py::dict follow_j6_once() {
    if(kind_!=TrialKind::Single) throw std::runtime_error("Single-target method unavailable on sequence trial");
    return prepare_trial(true);
  }
  py::dict stop_once() {
    int rc;double s,e;
    {py::gil_scoped_release release;std::lock_guard<std::mutex> lock(io_);require_login();
      if(stopped_) throw std::runtime_error("One stop attempt only; no retry");
      stopped_=true;stream_ready_=false;s=now_s();rc=sdk_->rootServiceRobotMoveControl(a::RobotMoveStop);e=now_s();}
    return result("slow_control_stop",rc,s,e);
  }
 protected:
  explicit J6Trial(TrialKind kind):kind_(kind) {}
  py::dict prepare_trial(bool send) {
    struct Check {std::string op;int rc;double s,e;};std::vector<Check> checks;
    a::JointParam q{};a::RobotState state{};a::RobotDiagnosis diagnosis{};
    a::JointRangeOfMotion ranges{};a::JointVelcAccParam acc{},vel{},got_acc{},got_vel{};
    bool real=false,sent=false;int rc=0;std::string refusal;
    std::array<double,6> target{};double start=0,end=0;
    {py::gil_scoped_release release;std::lock_guard<std::mutex> lock(io_);require_login();
      if(attempted_||stopped_) throw std::runtime_error("One follow attempt only; no motion after stop");
      attempted_=true;
      auto check=[&](const std::string &name,auto call) {
        double s=now_s();int r=call();double e=now_s();checks.push_back({name,r,s,e});
        if(r!=0) {rc=r;refusal=name+"_failed";return false;}return true;
      };
      bool ok=check("robot_state",[&]{return sdk_->robotServiceGetRobotCurrentState(state);}) &&
        check("real_robot",[&]{return sdk_->robotServiceGetIsRealRobotExist(real);}) &&
        check("diagnosis",[&]{return sdk_->robotServiceGetRobotDiagnosisInfo(diagnosis);}) &&
        check("joint_ranges",[&]{return sdk_->robotServiceGetJointRangeOfMotion(ranges);}) &&
        check("joint_angles",[&]{return sdk_->robotServiceGetJointAngleInfo(q);});
      if(ok && (state!=a::RobotStopped || !real || !diagnosis.armPowerStatus ||
          diagnosis.softEmergency || diagnosis.remoteEmergency || diagnosis.robotCollision || diagnosis.jointErrorStatus)) {
        ok=false;refusal="device_not_ready";
      }
      if(ok && !ranges.enable) {ok=false;refusal="joint_range_protection_not_enabled";}
      for(int i=0;i<6;++i) {
        const double max_offset=send?1.:(kind_==TrialKind::Reversal10Deg?10.:(kind_==TrialKind::Sequence5Hz?.8:2.));
        target[i]=q.jointPos[i]+(i==5?max_offset*DEG:0.);
        acc.jointPara[i]=DEG;vel.jointPara[i]=0.5*DEG;
        if(ok && (!std::isfinite(target[i]) || !std::isfinite(ranges.rangeValues[i].minValue) ||
          !std::isfinite(ranges.rangeValues[i].maxValue) ||
          ranges.rangeValues[i].minValue>=ranges.rangeValues[i].maxValue ||
          q.jointPos[i]<ranges.rangeValues[i].minValue || q.jointPos[i]>ranges.rangeValues[i].maxValue ||
          target[i]<ranges.rangeValues[i].minValue || target[i]>ranges.rangeValues[i].maxValue)) {
          ok=false;refusal="nonfinite_or_outside_reported_joint_range";
        }
      }
      if(ok) ok=check("init_client_profile",[&]{return sdk_->robotServiceInitGlobalMoveProfile();});
      if(ok) ok=check("set_client_acc",[&]{return sdk_->robotServiceSetGlobalMoveJointMaxAcc(acc);});
      if(ok) ok=check("set_client_vel",[&]{return sdk_->robotServiceSetGlobalMoveJointMaxVelc(vel);});
      if(ok) {
        sdk_->robotServiceGetGlobalMoveJointMaxAcc(got_acc);sdk_->robotServiceGetGlobalMoveJointMaxVelc(got_vel);
        for(int i=0;i<6;++i) if(!std::isfinite(got_acc.jointPara[i])||!std::isfinite(got_vel.jointPara[i])||
          std::abs(got_acc.jointPara[i]-acc.jointPara[i])>1e-12 ||
          std::abs(got_vel.jointPara[i]-vel.jointPara[i])>1e-12) {
          ok=false;refusal="client_profile_readback_mismatch";
        }
      }
      start=now_s();
      if(ok && send) {sent=true;rc=sdk_->robotServiceFollowModeJointMove(target.data());}
      if(ok && !send) {
        for(int i=0;i<6;++i) anchor_[i]=q.jointPos[i];
        prepared_at_=now_s();last_send_=prepared_at_;last_offset_=0.;stream_ready_=true;
      }
      end=now_s();
    }
    py::dict d;d["operation"]=send?"follow_j6_once":(kind_==TrialKind::Reversal10Deg?"prepare_j6_reversal_10deg":"prepare_j6_sequence");d["follow_invoked"]=sent;
    d["return_code"]=send?(sent?py::cast(rc):py::none()):py::cast(stream_ready_?0:(rc!=0?rc:-1));
    d["prepared"]=stream_ready_;d["refusal_reason"]=refusal;
    d["host_started_monotonic_s"]=start;d["host_received_monotonic_s"]=end;d["duration_s"]=end-start;
    d["anchor_rad"]=std::vector<double>(q.jointPos,q.jointPos+6);
    d["target_rad"]=std::vector<double>(target.begin(),target.end());
    d["client_velocity_rad_s"]=std::vector<double>(got_vel.jointPara,got_vel.jointPara+6);
    d["client_acceleration_rad_s2"]=std::vector<double>(got_acc.jointPara,got_acc.jointPara+6);
    d["profile_readback_scope"]="SDK client configuration; not proof of controller execution limits";
    py::list list;for(const auto &c:checks) list.append(result(c.op,c.rc,c.s,c.e));d["prechecks"]=list;
    return d;
  }
  py::dict send_offset_impl(double offset_deg) {
    if(kind_!=TrialKind::Sequence5Hz&&kind_!=TrialKind::Sequence10Hz&&kind_!=TrialKind::Sequence20Hz) throw std::runtime_error("Small-step sequence method unavailable on this trial");
    int rc;double s,e;std::array<double,6> target{};
    {py::gil_scoped_release release;std::lock_guard<std::mutex> lock(io_);require_login();
      if(!stream_ready_||stopped_) throw std::runtime_error("Not prepared or already stopped");
      double now=now_s(),dt=now-last_send_;
      const bool higher_rate=kind_!=TrialKind::Sequence5Hz,twenty=kind_==TrialKind::Sequence20Hz;
      const double period=twenty?.05:(higher_rate?.1:.2);
      bool valid=std::isfinite(offset_deg)&&offset_deg>=0.&&offset_deg<=(higher_rate?2.:.8) &&
        now-prepared_at_<=(twenty?30.:(higher_rate?20.:10.)) && dt<=(higher_rate?.3:.5) &&
        stream_count_<(twenty?201:(higher_rate?101:21)) &&
        (stream_count_==0 || dt>=period*.5) &&
        std::abs(offset_deg-last_offset_)<=.5*std::min(dt,period)+1e-9;
      if(!valid) {stream_ready_=false;throw std::runtime_error("Sequence range/timing/step budget violated; no retry");}
      target=anchor_;target[5]+=offset_deg*DEG;
      ++stream_count_;s=now_s();rc=sdk_->robotServiceFollowModeJointMove(target.data());e=now_s();
      last_send_=s;last_offset_=offset_deg;
      if(rc!=0||e-s>.2) stream_ready_=false;
    }
    auto d=result("follow_j6_sequence_target",rc,s,e);
    d["offset_deg"]=offset_deg;d["target_rad"]=std::vector<double>(target.begin(),target.end());
    d["attempt_index"]=stream_count_;return d;
  }
  py::dict send_reversal_impl(bool returning) {
    int rc;double s,e;std::array<double,6> target{};
    const double offset=returning?0.:10.;
    {py::gil_scoped_release release;std::lock_guard<std::mutex> lock(io_);require_login();
      if(kind_!=TrialKind::Reversal10Deg||!stream_ready_||stopped_) throw std::runtime_error("Reversal trial not prepared or already stopped");
      const double elapsed=now_s()-prepared_at_;
      if(stream_count_!=(returning?1:0)||elapsed>(returning?12.:.5)) {
        stream_ready_=false;throw std::runtime_error("Fixed reversal order/timing violated; no retry");
      }
      target=anchor_;target[5]+=offset*DEG;
      ++stream_count_;s=now_s();rc=sdk_->robotServiceFollowModeJointMove(target.data());e=now_s();
      if(returning||rc!=0||e-s>.2) stream_ready_=false;
    }
    auto d=result("follow_j6_reversal_target",rc,s,e);
    d["offset_deg"]=offset;d["target_rad"]=std::vector<double>(target.begin(),target.end());
    d["attempt_index"]=stream_count_;return d;
  }
 private:
  std::unique_ptr<ServiceInterface> sdk_;std::mutex io_;
  bool connected_=false,attempted_=false,stopped_=false;
  const TrialKind kind_;
  bool stream_ready_=false;std::array<double,6> anchor_{};
  double prepared_at_=0.,last_send_=0.,last_offset_=0.;int stream_count_=0;
  void require_login() {if(!connected_) throw std::runtime_error("Not connected");}
};
class J6StreamTrial:public J6Trial {
 public:
  explicit J6StreamTrial(int rate_hz=5):J6Trial(sequence_kind(rate_hz)) {}
  py::dict prepare() {return prepare_trial(false);}
  py::dict send_offset(double offset_deg) {return send_offset_impl(offset_deg);}
};
class J6ReversalTrial:public J6Trial {
 public:
  J6ReversalTrial():J6Trial(TrialKind::Reversal10Deg) {}
  py::dict prepare() {return prepare_trial(false);}
  py::dict send_outward() {return send_reversal_impl(false);}
  py::dict return_to_anchor() {return send_reversal_impl(true);}
};
PYBIND11_MODULE(_aubo_j6_trial,m) {
  m.doc()="Supervised bounded J6 experiments; no startup or general position API";
  py::class_<J6Trial>(m,"J6Trial").def(py::init<>()).def("login",&J6Trial::login)
    .def("logout",&J6Trial::logout).def("snapshot",&J6Trial::snapshot)
    .def("follow_j6_once",&J6Trial::follow_j6_once).def("stop_once",&J6Trial::stop_once);
  py::class_<J6StreamTrial,J6Trial>(m,"J6StreamTrial").def(py::init<int>(),py::arg("rate_hz")=5)
    .def("prepare",&J6StreamTrial::prepare).def("send_offset",&J6StreamTrial::send_offset);
  py::class_<J6ReversalTrial,J6Trial>(m,"J6ReversalTrial").def(py::init<>())
    .def("prepare",&J6ReversalTrial::prepare).def("send_outward",&J6ReversalTrial::send_outward)
    .def("return_to_anchor",&J6ReversalTrial::return_to_anchor);
}
