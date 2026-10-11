#include "M2006.hpp"
#include "bsp_time.hpp"
#include <algorithm>
#include <cmath>
namespace {
bool configured(const PID &p) {
    return std::isfinite(p.kp) && std::isfinite(p.ki) && std::isfinite(p.kd) &&
           std::isfinite(p.maxOut) && p.maxOut > 0 && std::isfinite(p.maxIOut) && p.maxIOut >= 0 &&
           (p.mode == PID_POSITION || p.mode == PID_DELTA);
}
} // namespace
M2006::M2006() {
    gearBox = GearBox_M2006;
    maxCurrent = 10000;
}
M2006::MotorStateTypedef M2006::AliveCheck() {
    MotorState = feedbackReceived && offlineTimeoutMs > 0 && offlineTimeoutMs <= 1000 &&
                         uint32_t(BSP_GetTickMs() - feedbackTick) < offlineTimeoutMs
                     ? MOTOR_ONLINE
                     : MOTOR_OFFLINE;
    Pre_AliveFlag = AliveFlag;
    if (MotorState == MOTOR_OFFLINE) {
        currentSet = 0;
        controlMode = RELAX_MODE;
        speedPid.Clear();
        positionPid.Clear();
        positionValid = false;
    }
    return MotorState;
}
bool M2006::ZeroPosition() {
    if (AliveCheck() != MOTOR_ONLINE)
        return false;
    encoderCounts = 0;
    motorFeedback.positionFdb = motorFeedback.lastPositionFdb = 0;
    positionSet = 0;
    positionPid.Clear();
    speedPid.Clear();
    positionValid = true;
    return true;
}
void M2006::setOutput() {
    currentSet = 0;
    if (previousMode_ != controlMode) {
        speedPid.Clear();
        positionPid.Clear();
        previousMode_ = controlMode;
    }
    if (!hcan || canId < 0x201 || canId > 0x208 || AliveCheck() != MOTOR_ONLINE ||
        !std::isfinite(speedSet) || !std::isfinite(positionSet) ||
        !std::isfinite(motorFeedback.speedFdb) || !std::isfinite(motorFeedback.positionFdb)) {
        speedPid.Clear();
        positionPid.Clear();
        return;
    }
    static_assert(sizeof(int16_t) == 2);
    if (!configured(speedPid) || (controlMode == POS_MODE && !configured(positionPid))) {
        speedPid.Clear();
        positionPid.Clear();
        return;
    }
    const float limit = std::min<unsigned>(maxCurrent, 10000);
    float target = speedSet;
    if (controlMode == POS_MODE && positionValid) {
        positionPid.ref = positionSet;
        positionPid.fdb = motorFeedback.positionFdb;
        positionPid.UpdateResult();
        target = positionPid.result;
    } else if (controlMode != SPD_MODE) {
        speedPid.Clear();
        positionPid.Clear();
        return;
    }
    speedPid.ref = target;
    speedPid.fdb = motorFeedback.speedFdb;
    speedPid.UpdateResult();
    const float command = std::clamp(speedPid.result, -limit, limit);
    currentSet = std::isfinite(command) ? static_cast<int16_t>(command) : 0;
}
