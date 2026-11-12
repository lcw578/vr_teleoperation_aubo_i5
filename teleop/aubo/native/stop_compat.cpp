// Compatibility adapter for the hash-pinned Linux x64 SDK 2.5.0 only.
// robotMoveControlService holds the completion mutex while waiting for ACK.
// Event 44 received before ACK blocks the only receiver on that same mutex,
// preventing ACK dispatch until the five-second request timeout releases it.
#include <cstring>
#include "aubo_sdk/serviceinterface.h"
#include "aubo_sdk/robotcontrolservices.h"
#include "stop_compat.hpp"

namespace aubo_stop_compat {
namespace {
// Explicit template instantiation grants access to this one SDK member under
// C++ access rules. Use the supplied class declaration, no guessed offsets,
// private/public macro, library patch, socket interception or new connection.
struct BaseServiceMember {
    using type = RobotControlServices* ServiceInterface::*;
    friend type member(BaseServiceMember);
};
template <BaseServiceMember::type Pointer> struct BaseServiceAccess {
    friend BaseServiceMember::type member(BaseServiceMember) { return Pointer; }
};
template struct BaseServiceAccess<&ServiceInterface::m_robotBaseService>;

// Call the vendor's own decoder using its verified exported ABI. Including
// protoencodedecode.h pulls obsolete protobuf headers into the application.
extern "C" bool decode_move_control(const char*, int, call_robot_motion_func_result&, int&)
    asm("_ZN17ProtoEncodeDecode33resolveResponse_moveControlResuleEPKciR29call_robot_motion_func_resultRi");
}

int request_slow_stop(RobotControlServices& service) {
    CommunicationResponse response;
    int rc = service.requestServiceGetResponseContentMode(
        CommunicationMateType::CommunicationType_RobotMoveControlType_SlowStop,
        nullptr, 0, response);
    if (rc != 0) return rc;
    if (!response.m_textPtr || response.m_textLength <= 0)
        return aubo_robot_namespace::ErrCode_ResponseInfoIsNULL;
    call_robot_motion_func_result result = call_robot_motion_func_failed;
    int error = 0;
    if (!decode_move_control(response.m_textPtr, response.m_textLength, result, error))
        return aubo_robot_namespace::ErrCode_ResolveResponseFailed;
    if (error != 0) return aubo_robot_namespace::ErrCode_moveControlSlowStopFailed;
    if (result != call_robot_motion_func_succ_wait_done &&
        result != call_robot_motion_func_succ_nowait &&
        result != call_robot_motion_func_succ_nocall_wait)
        return aubo_robot_namespace::ErrCode_moveControlSlowStopFailed;
    return 0;
}

int request_slow_stop(ServiceInterface& sdk) {
    auto* base = sdk.*member(BaseServiceMember{});
    if (!base) return aubo_robot_namespace::ErrCode_RequestRelatedVariableError;
    return request_slow_stop(*base);
}
}
