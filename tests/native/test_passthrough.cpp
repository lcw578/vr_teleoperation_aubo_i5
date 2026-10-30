#include "passthrough.hpp"
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
struct Fake {
 std::vector<std::string> calls;std::array<double,6> target{};
 int enter_rc=0,leave_rc=0;
 int robotServiceEnterTcp2CanbusMode(){calls.push_back("enter");return enter_rc;}
 int robotServiceSetRobotPosData2Canbus(double*q){calls.push_back("send");std::copy(q,q+6,target.begin());return 0;}
 int robotServiceLeaveTcp2CanbusMode(){calls.push_back("leave");return leave_rc;}
};
void check(bool v){if(!v)throw std::runtime_error("Passthrough lifecycle regression");}
int main(){
 Fake sdk;PassthroughMode mode;double q[]={.1,-.2,.3,-.4,.5,-.6};
 try{mode.send(sdk,q);throw 1;}catch(const std::runtime_error&){}
 check(sdk.calls.empty());check(mode.enter(sdk)==0);check(mode.entered());
 check(mode.send(sdk,q)==0);check(sdk.target==std::array<double,6>{.1,-.2,.3,-.4,.5,-.6});
 try{mode.enter(sdk);throw 1;}catch(const std::runtime_error&){}
 check(mode.leave(sdk)==0);check(!mode.possible());mode.leave(sdk);
 check(sdk.calls==std::vector<std::string>{"enter","send","leave"});
 check(mode.enter(sdk)==0);check(mode.leave(sdk)==0);
 Fake failure;PassthroughMode unknown;failure.enter_rc=1;
 check(unknown.enter(failure)==1);check(unknown.possible());check(!unknown.entered());
 try{unknown.send(failure,q);throw 1;}catch(const std::runtime_error&){}
 check(unknown.leave(failure)==0);check(!unknown.possible());
 Fake failed_exit;PassthroughMode held;held.enter(failed_exit);failed_exit.leave_rc=1;
 check(held.leave(failed_exit)==1);check(held.possible());check(!held.entered());
 try{held.enter(failed_exit);throw 1;}catch(const std::runtime_error&){}
 std::cout<<"Native passthrough lifecycle checks passed; fake SDK only.\n";
}
