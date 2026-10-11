#include "DJIMotorHandler.hpp"
#include "bsp_time.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
namespace {
int busIndex(CAN_HandleTypeDef *h) { return h == &hcan1 ? 0 : (h == &hcan2 ? 1 : -1); }
int signedBE(const uint8_t *p) {
    int v = (unsigned(p[0]) << 8) | p[1];
    return v >= 32768 ? v - 65536 : v;
}
} // namespace
bool DJIMotorHandler::registerMotor(DJIMotor *motor, CAN_HandleTypeDef *h, uint16_t id) {
    const int bus = busIndex(h);
    if (!motor || bus < 0 || id < 0x201 || id > 0x208 || motor->gearBox != GearBox_M2006)
        return false;
    for (auto &row : DJIMotorList)
        for (auto *registered : row)
            if (registered == motor)
                return motor->hcan == h && motor->canId == id;
    if (DJIMotorList[bus][id - 0x201])
        return false;
    motor->hcan = h;
    motor->canId = id;
    DJIMotorList[bus][id - 0x201] = motor;
    CAN1_0x200_Exist = CAN1_0x200_Exist || (bus == 0 && id <= 0x204);
    CAN1_0x1FF_Exist = CAN1_0x1FF_Exist || (bus == 0 && id >= 0x205);
    CAN2_0x200_Exist = CAN2_0x200_Exist || (bus == 1 && id <= 0x204);
    CAN2_0x1FF_Exist = CAN2_0x1FF_Exist || (bus == 1 && id >= 0x205);
    return true;
}
HAL_StatusTypeDef DJIMotorHandler::sendControlData() {
    uint8_t *packets[2][2] = {{can1_send_data_0, can1_send_data_1},
                              {can2_send_data_0, can2_send_data_1}};
    HAL_StatusTypeDef result = HAL_OK;
    for (int b = 0; b < 2; ++b)
        for (int g = 0; g < 2; ++g) {
            uint8_t *p = packets[b][g];
            std::memset(p, 0, 8);
            bool present = false;
            for (int i = 0; i < 4; ++i)
                if (auto *m = DJIMotorList[b][g * 4 + i]) {
                    present = true;
                    int command = 0;
                    if (m->AliveCheck() == DJIMotor::MOTOR_ONLINE &&
                        (m->controlMode == DJIMotor::SPD_MODE ||
                         (m->controlMode == DJIMotor::POS_MODE && m->positionValid))) {
                        const int limit = std::min<unsigned>(m->maxCurrent, 10000);
                        command = std::clamp<int>(m->currentSet, -limit, limit);
                    }
                    const uint16_t value = static_cast<uint16_t>(command);
                    p[i * 2] = value >> 8;
                    p[i * 2 + 1] = value & 255;
                }
            if (present) {
                auto s = CAN_Transmit(b ? &hcan2 : &hcan1, g ? 0x1ff : 0x200, p, 8);
                if (s != HAL_OK && (result == HAL_OK || s != HAL_BUSY))
                    result = s;
            }
        }
    return result;
}
void DJIMotorHandler::PollFeedback() {
    for (auto *h : {&hcan1, &hcan2}) {
        const int bus = busIndex(h);
        CAN_Stats stats{};
        if (CAN_GetStats(h, &stats) == HAL_OK && stats.dropped != previousDrops_[bus]) {
            previousDrops_[bus] = stats.dropped;
            for (auto *m : DJIMotorList[bus])
                if (m) {
                    m->positionValid = false;
                    m->controlMode = DJIMotor::RELAX_MODE;
                    m->currentSet = 0;
                    m->speedPid.Clear();
                    m->positionPid.Clear();
                }
        }
        for (unsigned i = 0; i < 16; ++i) {
            CAN_RxFrame frame{};
            if (CAN_Read(h, &frame) != HAL_OK)
                break;
            if (frame.id >= 0x201 && frame.id <= 0x208 && frame.length == 8 &&
                DJIMotorList[busIndex(h)][frame.id - 0x201])
                updateFeedback(h, frame.data, frame.id - 0x201);
            else if (unhandledFrame)
                unhandledFrame(h, frame);
        }
    }
}
void DJIMotorHandler::updateFeedback(CAN_HandleTypeDef *h, uint8_t *data, int index) {
    int bus = busIndex(h);
    if (bus < 0 || !data || index < 0 || index >= 8)
        return;
    UpdateSensorData(DJIMotorList[bus][index], data);
}
void DJIMotorHandler::UpdateSensorData(DJIMotor *m, uint8_t *p) {
    if (!m || !p || m->gearBox != GearBox_M2006)
        return;
    unsigned encoder = (unsigned(p[0]) << 8) | p[1];
    if (encoder >= 8192)
        return;
    const uint32_t now = BSP_GetTickMs();
    auto &f = m->motorFeedback;
    const bool continuous =
        m->feedbackReceived && uint32_t(now - m->feedbackTick) < m->offlineTimeoutMs;
    f.last_ecd = f.ecd;
    f.lastSpeedFdb = f.speedFdb;
    f.lastPositionFdb = f.positionFdb;
    if (continuous) {
        int delta = int(encoder) - int(f.ecd);
        if (delta > 4096)
            delta -= 8192;
        else if (delta < -4096)
            delta += 8192;
        m->encoderCounts += delta;
    } else {
        m->positionValid = false;
        m->currentSet = 0;
        m->controlMode = DJIMotor::RELAX_MODE;
        m->speedPid.Clear();
        m->positionPid.Clear();
    }
    f.ecd = encoder;
    f.speed_rpm = signedBE(p + 2);
    f.currentFdb = signedBE(p + 4);
    f.speedFdb = f.speed_rpm * (6.283185307179586f / 60.0f / 36.0f);
    f.positionFdb = double(m->encoderCounts) * (6.283185307179586 / 8192.0 / 36.0);
    f.temperatureFdb = std::numeric_limits<float>::quiet_NaN(); // C610 bytes 6,7 are reserved.
    m->feedbackReceived = true;
    m->feedbackTick = now;
    ++m->AliveFlag;
    m->MotorState = DJIMotor::MOTOR_ONLINE;
}
void DJIMotorHandler::AllMotorAliveCheck() {
    for (auto &row : DJIMotorList)
        for (auto *m : row)
            if (m)
                m->AliveCheck();
}
