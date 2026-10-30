// Separate diagnostic: no startup, movement, resume, mode or configuration API.
// A stopped-state precheck is a controller observation, not proof of physical rest.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "aubo_sdk/serviceinterface.h"
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;
namespace a = aubo_robot_namespace;
static double now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static py::dict record(const std::string &op, int rc, double start, double end, py::object data) {
  py::dict d;
  d["operation"]=op; d["return_code"]=rc;
  d["host_started_monotonic_s"]=start; d["host_received_monotonic_s"]=end;
  d["duration_s"]=end-start; d["controller_sample_time"]=py::none();
  d["data"]=rc==0?data:py::none(); return d;
}

class StationaryStopClient {
 public:
  ~StationaryStopClient() {
    if(connected_) {try {sdk_->robotServiceLogout();} catch(...) {}}
  }
  py::dict login(const std::string &host, int port, const std::string &user, const std::string &password) {
    if(host.empty() || port<1 || port>65535) throw std::invalid_argument("Invalid host/port");
    int rc; double start,end;
    {py::gil_scoped_release release; std::lock_guard<std::mutex> lock(io_);
      if(sdk_) throw std::runtime_error("New client required; no automatic reconnect");
      sdk_=std::make_unique<ServiceInterface>(); start=now_s();
      rc=sdk_->robotServiceLogin(host.c_str(),port,user.c_str(),password.c_str());
      end=now_s(); connected_=rc==0;}
    return record("login",rc,start,end,py::none());
  }
  py::dict logout() {
    int rc; double start,end;
    {py::gil_scoped_release release; std::lock_guard<std::mutex> lock(io_); require_login();
      start=now_s(); rc=sdk_->robotServiceLogout(); end=now_s(); connected_=false;}
    return record("logout",rc,start,end,py::none());
  }
  py::dict state() {
    a::RobotState value{}; int rc; double start,end;
    {py::gil_scoped_release release; std::lock_guard<std::mutex> lock(io_); require_login();
      start=now_s(); rc=sdk_->robotServiceGetRobotCurrentState(value); end=now_s();}
    return record("robot_state",rc,start,end,py::int_(int(value)));
  }
  py::dict joints() {
    a::JointParam value{}; int rc; double start,end;
    {py::gil_scoped_release release; std::lock_guard<std::mutex> lock(io_); require_login();
      start=now_s(); rc=sdk_->robotServiceGetJointAngleInfo(value); end=now_s();}
    return record("joint_angles",rc,start,end,py::cast(std::vector<double>(value.jointPos,value.jointPos+6)));
  }
  py::dict stop_at_rest(const std::string &method) {
    if(method!="public_stop" && method!="slow_control") throw std::invalid_argument("Stop method is not allowed");
    a::RobotState state{}; int check_rc,stop_rc=0; double start,end,check_start,check_end;
    bool invoked=false;
    {py::gil_scoped_release release; std::lock_guard<std::mutex> lock(io_); require_login();
      if(attempted_) throw std::runtime_error("Only one stop attempt per client is permitted");
      attempted_=true; check_start=now_s();
      check_rc=sdk_->robotServiceGetRobotCurrentState(state); check_end=now_s();
      start=check_end;
      if(check_rc==0 && state==a::RobotStopped) {
        invoked=true; start=now_s();
        stop_rc=method=="public_stop" ? sdk_->robotMoveStop()
          : sdk_->rootServiceRobotMoveControl(a::RobotMoveStop);
      }
      end=now_s();}
    py::dict d;
    d["operation"]="stationary_stop"; d["method"]=method; d["stop_invoked"]=invoked;
    d["precheck"]=record("robot_state",check_rc,check_start,check_end,py::int_(int(state)));
    d["return_code"]=invoked?py::cast(stop_rc):py::none();
    d["host_started_monotonic_s"]=start; d["host_received_monotonic_s"]=end;
    d["duration_s"]=end-start; d["controller_sample_time"]=py::none();
    if(!invoked) d["refusal_reason"]=check_rc!=0?"state_query_failed":"controller_not_stopped";
    return d;
  }
 private:
  std::unique_ptr<ServiceInterface> sdk_;
  bool connected_=false,attempted_=false;
  std::mutex io_; // Diagnostic calls only; not a concurrent motion/stop design.
  void require_login() {if(!connected_) throw std::runtime_error("Not connected");}
};

PYBIND11_MODULE(_aubo_stop_probe,m) {
  m.doc()="Stationary stop diagnostic only; no movement or startup API";
  m.attr("stop_methods")=std::vector<std::string>{"public_stop","slow_control"};
  py::class_<StationaryStopClient>(m,"StationaryStopClient")
    .def(py::init<>()).def("login",&StationaryStopClient::login)
    .def("logout",&StationaryStopClient::logout).def("state",&StationaryStopClient::state)
    .def("joints",&StationaryStopClient::joints).def("stop_at_rest",&StationaryStopClient::stop_at_rest);
}
