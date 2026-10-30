#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <aubo/aubo_api.h>
#include <aubo/robot/motion_control.h>
#include <aubo/robot/robot_config.h>
#include <aubo/robot/robot_state.h>
#include <aubo/system_info.h>
#include <aubo_sdk/rpc.h>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace py = pybind11;
using arcs::aubo_sdk::RpcClient;
static double monotonic_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
// Compile-time API check only; no motion is exported or called.
static_assert(std::is_member_function_pointer_v<decltype(&arcs::common_interface::MotionControl::servoJoint)>);

class ServoJReadOnlyProbe {
 public:
  explicit ServoJReadOnlyProbe(int timeout_ms = 1000) {
    if (timeout_ms < 100 || timeout_ms > 5000)
      throw std::invalid_argument("RPC request timeout must be 100..5000 ms");
    py::gil_scoped_release release;
    client_ = std::make_shared<RpcClient>();
    client_->setLogHandler([](int, const char *, int, const std::string &) {});
    if (client_->setRequestTimeout(timeout_ms) != 0 || client_->setExceptionFree(false) != 0)
      throw std::runtime_error("Could not configure read-only RPC client");
  }

  py::dict connect(const std::string &host, int port, const std::string &user,
                   const std::string &password) {
    if (host.empty() || port < 1 || port > 65535 || user.empty())
      throw std::invalid_argument("Explicit host, port and user are required");
    int rc = -1;
    std::string phase = "connect";
    bool threw = false;
    const double started = monotonic_s();
    {
      py::gil_scoped_release release;
      try {
        if (client_->hasConnected() || client_->hasLogined())
          throw std::runtime_error("Already connected");
        rc = client_->connect(host, port);
        if (rc == 0) { phase = "login"; rc = client_->login(user, password); }
      } catch (...) {
        threw = true;
        rc = error_code_or_unknown();
      }
    }
    py::dict result;
    result["phase"] = phase;
    result["return_code"] = rc;
    result["exception"] = threw;
    result["host_started_monotonic_s"] = started;
    result["host_received_monotonic_s"] = monotonic_s();
    return result;
  }

  py::dict inspect(const std::string &robot_name = "") {
    require_login();
    py::dict report;
    py::list queries;
    report["queries"] = queries;
    report["complete"] = false;
    report["snapshot_is_simultaneous"] = false;
    report["servo_joint_api_compiled"] = true;
    report["servo_joint_supported_by_controller"] = py::none();
    report["motion_calls_made"] = false;
    report["configuration_changed"] = false;
    try {
      auto system = client_->getSystemInfo();
      read(report, queries, "controller_version", [&] { return system->getControlSoftwareFullVersion(); });
      read(report, queries, "controller_version_code", [&] { return system->getControlSoftwareVersionCode(); });
      read(report, queries, "interface_version", [&] { return system->getInterfaceVersionCode(); });
      const auto names = read(report, queries, "robot_names", [&] { return client_->getRobotNames(); });
      std::string selected = robot_name;
      if (selected.empty() && names.size() == 1) selected = names.front();
      bool found = false;
      for (const auto &name : names) if (name == selected && !name.empty()) found = true;
      if (!found) { report["error"] = "robot_selection_required_or_not_found"; return report; }
      selected_robot_ = selected;
      report["selected_robot"] = selected;
      const auto robot = client_->getRobotInterface(selected);
      auto state = robot->getRobotState();
      auto motion = robot->getMotionControl();
      auto config = robot->getRobotConfig();
      read(report, queries, "robot_mode", [&] { return static_cast<int>(state->getRobotModeType()); });
      read(report, queries, "safety_mode", [&] { return static_cast<int>(state->getSafetyModeType()); });
      read(report, queries, "is_steady", [&] { return state->isSteady(); });
      read(report, queries, "servo_mode_select", [&] { return motion->getServoModeSelect(); });
      read(report, queries, "joint_positions_rad", [&] { return state->getJointPositions(); });
      read(report, queries, "joint_speeds_rad_s", [&] { return state->getJointSpeeds(); });
      read(report, queries, "joint_min_positions_rad", [&] { return config->getJointMinPositions(); });
      read(report, queries, "joint_max_positions_rad", [&] { return config->getJointMaxPositions(); });
      read(report, queries, "joint_max_speeds_rad_s", [&] { return config->getJointMaxSpeeds(); });
      read(report, queries, "joint_max_accelerations_rad_s2", [&] { return config->getJointMaxAccelerations(); });
      report["complete"] = true;
    } catch (...) { report["error"] = "read_only_query_failed"; }
    return report;
  }

  py::dict sample_joint_positions() {
    require_login();
    if (selected_robot_.empty()) throw std::runtime_error("Inspection required before sampling");
    py::dict result;
    py::list queries;
    result["queries"] = queries;
    try {
      auto state = client_->getRobotInterface(selected_robot_)->getRobotState();
      read(result, queries, "joint_positions_rad", [&] { return state->getJointPositions(); });
      result["complete"] = true;
    } catch (...) { result["complete"] = false; result["error"] = "read_only_query_failed"; }
    return result;
  }

  py::dict logout() {
    int logout_rc = 0, disconnect_rc = 0;
    bool logout_exception = false, disconnect_exception = false;
    {
      py::gil_scoped_release release;
      try { if (client_->hasLogined()) logout_rc = client_->logout(); }
      catch (...) { logout_rc = -1; logout_exception = true; }
      // Disconnect even if logout failed.
      try { if (client_->hasConnected()) disconnect_rc = client_->disconnect(); }
      catch (...) { disconnect_rc = -1; disconnect_exception = true; }
    }
    selected_robot_.clear();
    py::dict result;
    result["logout_return_code"] = logout_rc;
    result["disconnect_return_code"] = disconnect_rc;
    result["logout_exception"] = logout_exception;
    result["disconnect_exception"] = disconnect_exception;
    return result;
  }

 private:
  int error_code_or_unknown() noexcept {
    try {
      const int rc = client_->errorCode();
      return rc == 0 ? -1 : rc;
    } catch (...) { return -1; }
  }
  void require_login() {
    py::gil_scoped_release release;
    if (!client_->hasLogined()) throw std::runtime_error("Read-only login required");
  }
  template <typename F>
  auto read(py::dict &report, py::list &queries, const char *key, F call) -> decltype(call()) {
    py::dict timing;
    timing["field"] = key;
    const double started = monotonic_s();
    timing["host_started_monotonic_s"] = started;
    try {
      auto value = [&] { py::gil_scoped_release release; return call(); }();
      const double received = monotonic_s();
      timing["host_received_monotonic_s"] = received;
      timing["duration_s"] = received - started;
      timing["success"] = true;
      report[key] = value;
      queries.append(timing);
      return value;
    } catch (...) {
      const double received = monotonic_s();
      timing["host_received_monotonic_s"] = received;
      timing["duration_s"] = received - started;
      timing["success"] = false;
      int error_code = -1;
      {
        py::gil_scoped_release release;
        error_code = error_code_or_unknown();
      }
      timing["sdk_error_code"] = error_code;
      queries.append(timing);
      throw;
    }
  }
  std::shared_ptr<RpcClient> client_;
  std::string selected_robot_;
};

PYBIND11_MODULE(_aubo_servoj_probe, m) {
  m.doc() = "Standalone ARCS read-only compatibility probe; no motion API exported";
  m.attr("sdk_package_version") = "0.24.1-rc.3+318754d";
  py::class_<ServoJReadOnlyProbe>(m, "ServoJReadOnlyProbe")
      .def(py::init<int>(), py::arg("request_timeout_ms") = 1000)
      .def("connect", &ServoJReadOnlyProbe::connect)
      .def("inspect", &ServoJReadOnlyProbe::inspect, py::arg("robot_name") = "")
      .def("sample_joint_positions", &ServoJReadOnlyProbe::sample_joint_positions)
      .def("logout", &ServoJReadOnlyProbe::logout);
}
