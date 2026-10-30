#include "follow_profile.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

struct Params {double jointPara[6]{};};
struct FakeSdk {
    Params acc{{.3,.4,.5,.6,.7,.8}}, vel{{1.,1.1,1.2,1.3,1.4,1.5}};
    std::vector<std::string> calls;
    int fail=0;
    int robotServiceInitGlobalMoveProfile(){calls.push_back("reset");return fail;}
    int robotServiceSetGlobalMoveJointMaxAcc(const Params& v){calls.push_back("set_acc");acc=v;return fail;}
    int robotServiceSetGlobalMoveJointMaxVelc(const Params& v){calls.push_back("set_vel");vel=v;return fail;}
    void robotServiceGetGlobalMoveJointMaxAcc(Params& v){calls.push_back("get_acc");v=acc;}
    void robotServiceGetGlobalMoveJointMaxVelc(Params& v){calls.push_back("get_vel");v=vel;}
};
void require(bool condition) {if(!condition)throw std::runtime_error("Profile regression failed");}
int main() {
    auto check=[](const char*,auto call){return call()==0;};
    FakeSdk six;Params acc{},vel{};std::string refusal;
    require(prepare_follow_profile(six,true,false,.1745,.0873,acc,vel,check,refusal));
    require(six.calls.empty());
    // Explicit inspection retains the SDK getter path, but it is not used by
    // six-axis motion preparation.
    Params inspected_acc{},inspected_vel{};
    require(prepare_follow_profile(six,true,true,.1745,.0873,inspected_acc,inspected_vel,check,refusal));
    require(six.calls==std::vector<std::string>{"get_acc","get_vel"});
    for(int i=0;i<6;++i){require(inspected_acc.jointPara[i]==six.acc.jointPara[i]);require(inspected_vel.jointPara[i]==six.vel.jointPara[i]);}
    // A bad value must refuse explicit inspection, without repairing/resetting SDK memory.
    for(double bad : {0.,-1.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
        FakeSdk invalid;invalid.vel.jointPara[3]=bad;refusal.clear();
        require(!prepare_follow_profile(invalid,true,true,.1745,.0873,acc,vel,check,refusal));
        require(refusal=="invalid_sdk_client_profile");
        require(invalid.calls==std::vector<std::string>{"get_acc","get_vel"});
    }
    FakeSdk legacy;refusal.clear();
    require(prepare_follow_profile(legacy,false,false,.1745,.0873,acc,vel,check,refusal));
    require(legacy.calls==std::vector<std::string>{"reset","set_acc","set_vel","get_acc","get_vel"});
    for(int i=0;i<6;++i){require(acc.jointPara[i]==.0873);require(vel.jointPara[i]==.1745);}
    FakeSdk failed;failed.fail=1;
    require(!prepare_follow_profile(failed,false,false,.1745,.0873,acc,vel,check,refusal));
    require(failed.calls==std::vector<std::string>{"reset"});
    std::cout<<"Native profile call-order checks passed; no SDK linked, no device I/O.\n";
}
