// Read-only preparation: no startup, motion, stop, recovery or configuration writers.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "aubo_sdk/serviceinterface.h"
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
using Clock = std::chrono::steady_clock;
static double seconds() {
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}
template <size_t N> static std::string text_field(const char (&s)[N]) {
  size_t n = 0;
  while (n < N && s[n]) ++n;
  return std::string(s, n);
}
static const std::vector<std::string> queries = {
  "server_version", "device_info", "work_mode", "robot_state", "is_real_robot",
  "online_mode", "online_master_mode", "mac_communication_ok", "joint_angles",
  "current_waypoint", "joint_status", "joint_ranges", "joint_position_limits",
  "joint6_rotate360_enabled", "tool_kinematics", "tool_dynamics", "safety_config",
  "diagnosis"
};
static py::dict waypoint(const a::wayPoint_S &v) {
  py::dict d;
  d["joint_positions_rad"] = std::vector<double>(v.jointpos, v.jointpos + 6);
  d["position_m"] = std::vector<double>{v.cartPos.position.x, v.cartPos.position.y, v.cartPos.position.z};
  d["quaternion_wxyz"] = std::vector<double>{v.orientation.w, v.orientation.x, v.orientation.y, v.orientation.z};
  d["frame_definition"] = "SDK waypoint; flange/TCP convention requires verification";
  return d;
}
static py::dict ranges(const a::JointRangeOfMotion &v) {
  py::dict d;
  d["enable_raw"] = v.enable;
  std::vector<std::array<double, 2>> values;
  for (const auto &r : v.rangeValues) values.push_back({r.minValue, r.maxValue});
  d["range_values_raw"] = values;
  d["notes"] = "Getter semantics/units must be checked against this controller";
  return d;
}

class ReadOnlyClient {
 public:
  // SDK construction is lazy: even constructing this wrapper makes no SDK call.
  ~ReadOnlyClient() {
    if (logged_in_ && sdk_) { try { sdk_->robotServiceLogout(); } catch (...) {} }
  }
  py::dict login(const std::string &host, int port, const std::string &user,
                 const std::string &password) {
    if (host.empty() || port < 1 || port > 65535) throw std::invalid_argument("Invalid host/port");
    int rc; double started, received;
    {
      py::gil_scoped_release release;
      std::lock_guard<std::mutex> lock(io_);
      if (sdk_) throw std::runtime_error("Use a new client for each connection attempt; no automatic retry");
      sdk_ = std::make_unique<ServiceInterface>();
      started = seconds();
      rc = sdk_->robotServiceLogin(host.c_str(), port, user.c_str(), password.c_str());
      received = seconds();
      logged_in_ = rc == 0;
    }
    return result("login", rc, started, received, py::none());
  }
  py::dict logout() {
    int rc; double started, received;
    {
      py::gil_scoped_release release;
      std::lock_guard<std::mutex> lock(io_);
      require_login();
      started = seconds(); rc = sdk_->robotServiceLogout(); received = seconds();
      logged_in_ = false;
    }
    return result("logout", rc, started, received, py::none());
  }
  py::dict query(const std::string &name) {
    if (name == "server_version") return read<std::string>(name,
      [&](auto &v){return sdk_->robotServiceGetServerVersionInfo(v);}, [](auto &v){return py::cast(v);});
    if (name == "device_info") return read<a::RobotDevInfo>(name,
      [&](auto &v){return sdk_->robotServiceGetRobotDevInfoService(v);}, [](auto &v){
        py::dict d; d["type_raw"] = int(v.type); d["revision"] = text_field(v.revision);
        d["manufacturer"] = text_field(v.manu_id); d["joint_type"] = text_field(v.joint_type);
        d["description"] = text_field(v.desc); d["slave_version"] = text_field(v.slave_version);
        d["extio_version"] = text_field(v.extio_version); py::list j;
        for (int i=0;i<8;++i) {py::dict x; x["hardware_version"]=text_field(v.joint_ver[i].hw_version);
          x["software_version"]=text_field(v.joint_ver[i].sw_version);
          x["product_id"]=text_field(v.jointProductID[i].productID); j.append(x);}
        d["joint_and_tool_versions_raw"] = j; return d;});
    if (name == "work_mode") return read<a::RobotWorkMode>(name,
      [&](auto &v){return sdk_->robotServiceGetRobotWorkMode(v);}, [](auto &v){return py::int_(int(v));});
    if (name == "robot_state") return read<a::RobotState>(name,
      [&](auto &v){return sdk_->robotServiceGetRobotCurrentState(v);}, [](auto &v){return py::int_(int(v));});
    if (name == "is_real_robot") return boolean(name, [&](auto &v){return sdk_->robotServiceGetIsRealRobotExist(v);});
    if (name == "online_mode") return boolean(name, [&](auto &v){return sdk_->robotServiceIsOnlineMode(v);});
    if (name == "online_master_mode") return boolean(name, [&](auto &v){return sdk_->robotServiceIsOnlineMasterMode(v);});
    if (name == "mac_communication_ok") return boolean(name, [&](auto &v){return sdk_->robotServiceGetMacCommunicationStatus(v);});
    if (name == "joint6_rotate360_enabled") return boolean(name, [&](auto &v){return sdk_->robotServiceGetJoint6Rotate360EnableFlag(v);});
    if (name == "joint_angles") return read<a::JointParam>(name,
      [&](auto &v){return sdk_->robotServiceGetJointAngleInfo(v);}, [](auto &v){
        py::dict d;d["joint_positions_rad"]=std::vector<double>(v.jointPos,v.jointPos+6);return d;});
    if (name == "current_waypoint") return read<a::wayPoint_S>(name,
      [&](auto &v){return sdk_->robotServiceGetCurrentWaypointInfo(v);}, waypoint);
    if (name == "joint_status") return read<std::array<a::JointStatus,6>>(name,
      [&](auto &v){return sdk_->robotServiceGetRobotJointStatus(v.data(),6);}, [](auto &v){
        py::dict d;std::vector<double> q;py::list raw;
        for (const auto &j:v) {q.push_back(j.jointPosJ);py::dict x;
          x["joint_current_raw"]=int(j.jointCurrentI);x["joint_speed_raw"]=int(j.jointSpeedMoto);
          x["joint_position_rad"]=double(j.jointPosJ);x["driver_target_position_rad"]=double(j.jointTagPosJ);
          x["joint_error_code"]=int(j.jointErrorNum);raw.append(x);}
        d["joint_positions_rad"]=q;d["joints"]=raw;
        d["notes"]="Raw current/speed units unverified; driver target is not host action";return d;});
    if (name == "joint_ranges") return read<a::JointRangeOfMotion>(name,
      [&](auto &v){return sdk_->robotServiceGetJointRangeOfMotion(v);}, ranges);
    if (name == "joint_position_limits") return read<a::JointRangeOfMotion>(name,
      [&](auto &v){return sdk_->robotServiceGetJointPositionLimit(v);}, ranges);
    if (name == "tool_kinematics") return read<a::ToolKinematicsParam>(name,
      [&](auto &v){return sdk_->robotServiceGetToolKinematicsParam(v);}, [](auto &v){
        py::dict d;d["position_m"]=std::vector<double>{v.toolInEndPosition.x,v.toolInEndPosition.y,v.toolInEndPosition.z};
        d["quaternion_wxyz"]=std::vector<double>{v.toolInEndOrientation.w,v.toolInEndOrientation.x,v.toolInEndOrientation.y,v.toolInEndOrientation.z};return d;});
    if (name == "tool_dynamics") return read<a::ToolDynamicsParam>(name,
      [&](auto &v){return sdk_->robotServiceGetToolDynamicsParam(v);}, [](auto &v){
        py::dict d;d["payload_raw"]=v.payload;d["center_of_mass_raw"]=std::vector<double>{v.positionX,v.positionY,v.positionZ};
        d["inertia_reserved_raw"]=std::vector<double>{v.toolInertia.xx,v.toolInertia.xy,v.toolInertia.xz,v.toolInertia.yy,v.toolInertia.yz,v.toolInertia.zz};return d;});
    if (name == "safety_config") return read<a::RobotSafetyConfig>(name,
      [&](auto &v){return sdk_->robotServiceGetRobotSafetyConfig(v);}, [](auto &v){
        py::dict d;d["reduced_joint_speed_raw"]=std::vector<unsigned>(v.robotReducedConfigJointSpeed,v.robotReducedConfigJointSpeed+6);
        d["reduced_tcp_speed_raw"]=unsigned(v.robotReducedConfigTcpSpeed);d["reduced_tcp_force_raw"]=unsigned(v.robotReducedConfigTcpForce);
        d["reduced_momentum_raw"]=unsigned(v.robotReducedConfigMomentum);d["reduced_power_raw"]=unsigned(v.robotReducedConfigPower);
        d["safeguard_reset_raw"]=unsigned(v.robotSafeguradResetConfig);d["operational_mode_raw"]=unsigned(v.robotOperationalModeConfig);return d;});
    if (name == "diagnosis") return read<a::RobotDiagnosis>(name,
      [&](auto &v){return sdk_->robotServiceGetRobotDiagnosisInfo(v);}, [](auto &v){
        py::dict d;d["canbus_status_raw"]=int(v.armCanbusStatus);d["power_on"]=bool(v.armPowerStatus);
        d["soft_emergency"]=bool(v.softEmergency);d["remote_emergency"]=bool(v.remoteEmergency);
        d["collision"]=bool(v.robotCollision);d["brake_status_raw"]=bool(v.brakeStuats);
        d["joint_error"]=bool(v.jointErrorStatus);d["pose_read_enabled"]=bool(v.enableReadPose);
        d["buffer_size_reserved_raw"]=int(v.macTargetPosBufferSize);d["buffer_data_reserved_raw"]=int(v.macTargetPosDataSize);
        d["interrupt_warning_reserved_raw"]=int(v.macDataInterruptWarning);return d;});
    throw std::invalid_argument("Query is not in the read-only allowlist");
  }
  py::dict fk(const std::array<double,6> &q) {
    for(double x:q) if(!std::isfinite(x)) throw std::invalid_argument("FK requires six finite radians");
    return read<a::wayPoint_S>("fk", [&](auto &v){return sdk_->robotServiceRobotFk(q.data(),6,v);}, waypoint);
  }
 private:
  std::unique_ptr<ServiceInterface> sdk_;
  bool logged_in_ = false;
  std::mutex io_;
  void require_login() {if(!logged_in_) throw std::runtime_error("Not connected; explicit login required");}
  static py::dict result(const std::string &name,int rc,double start,double end,py::object data) {
    py::dict d;d["operation"]=name;d["return_code"]=rc;
    d["host_started_monotonic_s"]=start;d["host_received_monotonic_s"]=end;
    d["duration_s"]=end-start;d["controller_sample_time"]=py::none();
    d["data"]=rc==0?data:py::none();return d;
  }
  template<class T,class Call,class Encode> py::dict read(const std::string &name,Call call,Encode encode) {
    T v{};int rc;double start,end;
    {py::gil_scoped_release release;std::lock_guard<std::mutex> lock(io_);require_login();
      start=seconds();rc=call(v);end=seconds();}
    // Do not expose zero-initialized output structures after a failed query.
    py::object data=py::none();if(rc==0) data=encode(v);
    return result(name,rc,start,end,data);
  }
  template<class Call> py::dict boolean(const std::string &name,Call call) {
    return read<bool>(name,call,[](bool v){return py::bool_(v);});
  }
};

PYBIND11_MODULE(_aubo_readonly,m) {
  m.doc()="AUBO SDK 2.5.0 read-only preparation; no motion API exported";
  m.attr("query_names")=queries;
  py::class_<ReadOnlyClient>(m,"ReadOnlyClient")
    .def(py::init<>()).def("login",&ReadOnlyClient::login)
    .def("logout",&ReadOnlyClient::logout).def("query",&ReadOnlyClient::query)
    .def("fk",&ReadOnlyClient::fk);
}
