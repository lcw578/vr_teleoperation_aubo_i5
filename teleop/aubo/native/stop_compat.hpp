#pragma once
class ServiceInterface;
class RobotControlServices;
namespace aubo_stop_compat {
// Request acknowledgement only. The caller must independently confirm fresh
// RobotStopped feedback and stable measured joints before permitting motion.
int request_slow_stop(ServiceInterface& sdk);
int request_slow_stop(RobotControlServices& service);
}
