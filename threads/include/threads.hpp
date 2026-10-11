#pragma once
#include "tx_api.h"
#include "IMU.hpp"
#include "IST8310.hpp"
#include "ahrs6.hpp"
#include <cstdint>
enum class InitState : uint8_t { Pending, Ready, Failed };
enum class ControlState : uint8_t { Disabled, AwaitFeedback, Running, Fault };
enum class TaskId : uint8_t { Imu, Control };
struct ImuSnapshot {
    acc_data_t acceleration{}; // m/s^2, native axes
    gyro_data_t angularVelocity{}; // rad/s, native axes
    MagneticField magneticField{}; // uT, native IST8310 axes; NOT fused
    Attitude6 attitude{};
    uint32_t sampleMs{}, temperatureMs{}, magneticMs{}, sequence{};
    bool valid{}, temperatureValid{}, magneticValid{}, attitudeValid{};
};
struct AppHealth {
    InitState imuInit = InitState::Pending, controlInit = InitState::Pending;
    ControlState control = ControlState::Disabled;
    bool imuDataOk{}, motorOnline{};
    uint32_t imuReadErrors{}, canSendErrors{}, missedDeadlines{};
    float motorPosition{}, motorSpeed{}, targetPosition{};
    int16_t currentCommand{};
};
struct ControlCommand {
    bool enabled{};
    float positionRad{}; // relative OUTPUT shaft rad; referenced at each explicit arm
    uint32_t armGeneration{};
};
extern TX_SEMAPHORE init_semaphore, imu_semaphore, control_semaphore;
extern TX_MUTEX app_data_mutex;
extern volatile ULONG app_alive_count;
extern volatile UINT app_init_status, app_stack_error;
extern volatile bool app_imu_alive, app_control_alive;
extern AppHealth app_health; // debugger only: application readers use GetAppHealth
extern ImuSnapshot app_imu;
extern ControlCommand app_control_command;
bool GetImuSnapshot(ImuSnapshot *out); // thread context, TX_NO_WAIT; false on contention
bool GetAppHealth(AppHealth *out);
// Requires verified config and build switch. Fault recovery: disable -> enable.
bool SetMotorCommand(bool enable, float positionRad);
void AppPulse(TaskId task);
void AppInitChanged();
void AppSleepUntil(ULONG &deadline, ULONG period);
constexpr ULONG AppTicks(unsigned ms) {
    return (static_cast<ULONG>(ms) * TX_TIMER_TICKS_PER_SECOND + 999) / 1000;
}
void alive_thread_entry(ULONG);
void imu_thread_entry(ULONG);
void control_thread_entry(ULONG);
