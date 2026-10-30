#pragma once
#include <cmath>
#include <string>

// Shared by the real prepare path and an offline call-recording test.
// The six-axis motion path deliberately does not call the SDK profile getters:
// those void getters expose client-side storage, are not needed for direct IK
// Follow, and on this SDK/controller combination can contain uninitialized data.
// The optional read_profile path is retained only for explicit inspection.
template<class Sdk, class Params, class Check>
bool prepare_follow_profile(Sdk& sdk, bool six_axis, bool read_profile, double legacy_velocity,
                            double legacy_acceleration, Params& got_acc,
                            Params& got_vel, Check check, std::string& refusal) {
    Params acc{}, vel{};
    if (!six_axis) {
        for (int i=0; i<6; ++i) {
            acc.jointPara[i]=legacy_acceleration;
            vel.jointPara[i]=legacy_velocity;
        }
        if (!check("client_profile", [&]{return sdk.robotServiceInitGlobalMoveProfile();}) ||
            !check("client_acc", [&]{return sdk.robotServiceSetGlobalMoveJointMaxAcc(acc);}) ||
            !check("client_vel", [&]{return sdk.robotServiceSetGlobalMoveJointMaxVelc(vel);})) return false;
    }
    if (six_axis && !read_profile) return true;
    sdk.robotServiceGetGlobalMoveJointMaxAcc(got_acc);
    sdk.robotServiceGetGlobalMoveJointMaxVelc(got_vel);
    for (int i=0; i<6; ++i) {
        if (!std::isfinite(got_acc.jointPara[i]) || !std::isfinite(got_vel.jointPara[i]) ||
            got_acc.jointPara[i]<=0 || got_vel.jointPara[i]<=0) {
            refusal="invalid_sdk_client_profile";
            return false;
        }
        if (!six_axis && (std::abs(got_acc.jointPara[i]-acc.jointPara[i])>1e-12 ||
                         std::abs(got_vel.jointPara[i]-vel.jointPara[i])>1e-12)) {
            refusal="client_profile_readback_mismatch";
            return false;
        }
    }
    return true;
}
