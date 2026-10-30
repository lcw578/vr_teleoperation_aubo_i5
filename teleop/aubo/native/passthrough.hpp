#pragma once
#include <stdexcept>

// No SDK dependency: the real transport and an offline call-order test share
// this lifecycle. An unsuccessful enter/leave can have an unknown outcome.
class PassthroughMode {
    bool possible_=false, entered_=false;
public:
    bool possible() const {return possible_;}
    bool entered() const {return entered_;}
    template<class Sdk> int enter(Sdk& sdk) {
        if(possible_)throw std::runtime_error("Passthrough already entered or outcome unknown");
        possible_=true;
        int rc=sdk.robotServiceEnterTcp2CanbusMode();
        entered_=rc==0;
        return rc;
    }
    template<class Sdk> int send(Sdk& sdk,double* q) {
        if(!entered_)throw std::runtime_error("Passthrough not entered");
        return sdk.robotServiceSetRobotPosData2Canbus(q);
    }
    template<class Sdk> int leave(Sdk& sdk) {
        if(!possible_)return 0;
        int rc=sdk.robotServiceLeaveTcp2CanbusMode();
        entered_=false;
        if(rc==0)possible_=false;
        return rc;
    }
};
