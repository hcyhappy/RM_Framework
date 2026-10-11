#include "threads_internal.hpp"
#include "app_config.hpp"
#include "M2006.hpp"
#include "DJIMotorHandler.hpp"
#include "bsp_usart.hpp"
namespace {
void relax(M2006 &motor) {
    motor.controlMode=DJIMotor::RELAX_MODE; motor.currentSet=0;
    motor.speedPid.Clear(); motor.positionPid.Clear();
}
}
void control_thread_entry(ULONG) {
    // Static lifetime because the registry keeps pointers; never expose motor to another thread.
    static M2006 motor;
    auto *handler=DJIMotorHandler::Instance();
    auto *bus=AppConfig::MotorCanBus==2 ? &hcan2 : &hcan1;
    const bool registered=(AppConfig::MotorCanBus==1 || AppConfig::MotorCanBus==2) &&
                           handler->registerMotor(&motor,bus,AppConfig::MotorFeedbackId);
    motor.maxCurrent=AppConfig::CurrentLimit;
    motor.offlineTimeoutMs=AppConfig::MotorFeedbackTimeoutMs;
    motor.speedPid=PID(AppConfig::SpeedKp,AppConfig::SpeedKi,AppConfig::SpeedKd,
                       AppConfig::CurrentLimit,AppConfig::SpeedIntegralLimit);
    motor.positionPid=PID(AppConfig::PositionKp,AppConfig::PositionKi,AppConfig::PositionKd,
                          AppConfig::MaxSpeed,AppConfig::PositionIntegralLimit);
    { AppLock lock; app_health.controlInit=registered ? InitState::Ready : InitState::Failed; }
    AppInitChanged();
    ULONG deadline=tx_time_get(), lastLoop=deadline;
    uint32_t generation=0;
    bool fault=false;
#if RM_ENABLE_MOTOR_OUTPUT
    bool transmitted=false, abortNeeded=false, stopPending=false;
    unsigned sendFailures=0, abortFailures=0;
#endif
    for (;;) {
        const ULONG now=tx_time_get();
        const bool late=ULONG(now-lastLoop)>AppTicks(10);
        lastLoop=now;
        handler->PollFeedback(); // ONE consumer for both buses, unhandledFrame can dispatch other protocols
        const bool online=registered && motor.AliveCheck()==DJIMotor::MOTOR_ONLINE;
        ControlCommand command;
        { AppLock lock; command=app_control_command; }
        ControlState state=ControlState::Disabled;
#if RM_ENABLE_MOTOR_OUTPUT
        const bool newArm=command.enabled && generation!=command.armGeneration;
        if (newArm) {
            generation=command.armGeneration;
            fault=!registered || !MotorConfigValid() || !online || !motor.ZeroPosition();
            sendFailures=abortFailures=0;
            abortNeeded=true; // remove old pending commands before this new session
        }
        if (!command.enabled) { fault=false; relax(motor); }
        else if (late || !online || !motor.positionValid || !MotorConfigValid()) fault=true;
        if (command.enabled && !fault) {
            motor.positionSet=command.positionRad;
            motor.controlMode=DJIMotor::POS_MODE;
            motor.setOutput(); // outer position -> bounded speed -> inner speed -> bounded current
            state=ControlState::Running;
        } else {
            relax(motor);
            state=command.enabled ? ControlState::Fault : ControlState::Disabled;
            if (transmitted) { abortNeeded=true; stopPending=true; }
        }
        if (abortNeeded) {
            const auto result=CAN_AbortPending(bus);
            if (result==HAL_OK) { abortNeeded=false; transmitted=false; abortFailures=0; }
            // No new nonzero frame until hardware has confirmed pending mailboxes are clear.
            else {
                { AppLock lock; ++app_health.canSendErrors; }
                if (result!=HAL_BUSY || ++abortFailures>=5) fault=command.enabled;
            }
        }
        if (registered && !abortNeeded && (command.enabled || stopPending)) {
            if (fault) relax(motor);
            const auto result=handler->sendControlData();
            transmitted=command.enabled;
            if (result!=HAL_OK) {
                { AppLock lock; ++app_health.canSendErrors; }
                if (result!=HAL_BUSY || ++sendFailures>=5) {
                    fault=command.enabled; abortNeeded=true; relax(motor);
                }
            } else { sendFailures=0; stopPending=false; }
        }
        if (fault && command.enabled) { state=ControlState::Fault; relax(motor); }
#else
        (void)generation; (void)late; (void)fault;
        relax(motor); // receive/monitor only: no CAN control transmission in default build
#endif
        // RX error recovery stays at BSP layer, no printf/DMA TX or blocking UART sends here.
        (void)USART_Service(&huart1);
        (void)USART_Service(&huart6);
        { AppLock lock;
          app_health.control=(!registered) ? ControlState::Fault :
                            (!command.enabled && !online ? ControlState::AwaitFeedback : state);
          app_health.motorOnline=online;
          app_health.motorPosition=motor.motorFeedback.positionFdb;
          app_health.motorSpeed=motor.motorFeedback.speedFdb;
          app_health.targetPosition=command.positionRad;
          app_health.currentCommand=motor.currentSet;
        }
        AppPulse(TaskId::Control);
        AppSleepUntil(deadline,AppTicks(AppConfig::ControlPeriodMs));
    }
}
