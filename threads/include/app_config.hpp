#pragma once
#include <cstdint>
namespace AppConfig {
constexpr unsigned MotorCanBus = 1;
constexpr uint16_t MotorFeedbackId = 0x201; // C610 ID1, command group 0x200
constexpr uint16_t CurrentLimit = 1000; // command units, 10000 = 10 A
constexpr float MaxSpeed = 1.0f; // output shaft rad/s
constexpr float MaxPosition = 6.2831853f; // relative output shaft rad
// Fill using bench measurements. Zero gains intentionally prevent arming.
constexpr float SpeedKp = 0, SpeedKi = 0, SpeedKd = 0, SpeedIntegralLimit = 0;
constexpr float PositionKp = 0, PositionKi = 0, PositionKd = 0, PositionIntegralLimit = 0;
constexpr unsigned MotorFeedbackTimeoutMs = 50;
constexpr unsigned ControlPeriodMs = 2, ImuPeriodMs = 2, AlivePeriodMs = 100;
constexpr unsigned HeartbeatPeriodMs = 20, HeartbeatTimeoutMs = 500, StartupGraceMs = 2000;
constexpr unsigned TemperaturePeriodMs = 200, MagneticPeriodMs = 10, InitRetryMs = 1000;
// Control thread owns both BSP CAN receive queues. No automatic enable at boot.
// IMU snapshots/AHRS use native axes; verify mounting and transform acc/gyro together
// before treating these axes as vehicle body axes. IST8310 axes are separate.
}
