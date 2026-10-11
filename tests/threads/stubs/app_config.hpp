#pragma once
// Explicit HOST-ONLY gains; production header retains all gains at zero.
#define AppConfig ProductionConfig
#include "../../../threads/include/app_config.hpp"
#undef AppConfig
namespace AppConfig {
using ProductionConfig::MotorCanBus;
using ProductionConfig::MotorFeedbackId;
using ProductionConfig::CurrentLimit;
using ProductionConfig::MaxSpeed;
using ProductionConfig::MaxPosition;
using ProductionConfig::SpeedIntegralLimit;
using ProductionConfig::PositionIntegralLimit;
using ProductionConfig::MotorFeedbackTimeoutMs;
using ProductionConfig::ControlPeriodMs;
using ProductionConfig::ImuPeriodMs;
using ProductionConfig::AlivePeriodMs;
using ProductionConfig::HeartbeatPeriodMs;
using ProductionConfig::HeartbeatTimeoutMs;
using ProductionConfig::StartupGraceMs;
using ProductionConfig::TemperaturePeriodMs;
using ProductionConfig::MagneticPeriodMs;
using ProductionConfig::InitRetryMs;
constexpr float SpeedKp=100, SpeedKi=0, SpeedKd=0;
constexpr float PositionKp=4, PositionKi=0, PositionKd=0;
}
