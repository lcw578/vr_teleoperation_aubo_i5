// Actual vendor SDK, but connection is hard-coded to loopback. No login,
// startup, movement, mode change, profile change or robot address is accepted.
#include <cstring>
#include "aubo_sdk/serviceinterface.h"
#include "aubo_sdk/robotcontrolservices.h"
#include "stop_compat.hpp"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
class LoopbackClient : public RobotControlServices {
public:
 int open(int port){setServerIP("127.0.0.1");setServerPort(port);return connectRobotServer();}
};
int main(int argc, char** argv) {
 if(argc!=3) return 2;
 const int port=std::stoi(argv[1]);const std::string mode=argv[2];
 if(port<1||port>65535||(mode!="original"&&mode!="fixed"))return 2;
 ServiceInterface disconnected;
 if(aubo_stop_compat::request_slow_stop(disconnected)!=aubo_robot_namespace::ErrCode_SocketDisconnect)
  return 3; // Also exercise the real ServiceInterface member accessor.
 auto* client=new LoopbackClient();
 if(client->open(port)!=0)return 4;
 auto start=std::chrono::steady_clock::now();
 int rc=mode=="original"?client->robotMoveControlService(
     CommunicationMateType::CommunicationType_RobotMoveControlType_SlowStop):
     aubo_stop_compat::request_slow_stop(*client);
 double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
 std::cout<<"RESULT "<<rc<<" "<<ms<<std::endl;
 // End this isolated subprocess rather than waiting for SDK heartbeat cleanup.
 std::_Exit(0);
}
