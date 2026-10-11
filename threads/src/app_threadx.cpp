#include "app_threadx.h"
#include "threads_internal.hpp"
#include "app_config.hpp"
#include "main.h"
#include "LED.hpp"
#include <cmath>
namespace {
TX_BYTE_POOL thread_pool;
TX_THREAD alive_thread, imu_thread, control_thread;
alignas(8) UCHAR pool_storage[12288];
void stack_error(TX_THREAD *) {
    app_stack_error=1;
    LED::Set(LED::Color::Green,false);
    LED::Set(LED::Color::Blue,false);
    LED::Set(LED::Color::Red,true);
    Error_Handler();
}
void create(TX_THREAD *thread,const char *name,void (*entry)(ULONG),ULONG bytes,UINT priority) {
    void *stack=nullptr;
    AppCheck(tx_byte_allocate(&thread_pool,&stack,bytes,TX_NO_WAIT));
    AppCheck(tx_thread_create(thread,const_cast<CHAR *>(name),entry,0,stack,bytes,
                             priority,priority,TX_NO_TIME_SLICE,TX_DONT_START));
}
ULONG last_pulse[2]{};
}
TX_SEMAPHORE init_semaphore, imu_semaphore, control_semaphore;
TX_MUTEX app_data_mutex;
AppHealth app_health;
ImuSnapshot app_imu;
ControlCommand app_control_command;
volatile ULONG app_alive_count=0;
volatile UINT app_init_status=TX_NOT_DONE, app_stack_error=0;
volatile bool app_imu_alive=false, app_control_alive=false;
void AppCheck(UINT status) {
    if (status!=TX_SUCCESS) {
        app_init_status=status;
        LED::Set(LED::Color::Green,false);
        LED::Set(LED::Color::Blue,false);
        LED::Set(LED::Color::Red,true);
        Error_Handler(); // fatal kernel-object or stack failure: preserve LED fault
    }
}
bool MotorConfigValid() {
    using namespace AppConfig;
    const float values[]={SpeedKp,SpeedKi,SpeedKd,SpeedIntegralLimit,PositionKp,
                          PositionKi,PositionKd,PositionIntegralLimit,MaxSpeed,MaxPosition};
    for (float v:values) if (!std::isfinite(v)||v<0) return false;
    return (MotorCanBus==1||MotorCanBus==2) && MotorFeedbackId>=0x201 && MotorFeedbackId<=0x208 &&
           CurrentLimit>0 && CurrentLimit<=10000 && MaxSpeed>0 && MaxPosition>0 &&
           SpeedKp>0 && PositionKp>0 && SpeedIntegralLimit<=CurrentLimit &&
           PositionIntegralLimit<=MaxSpeed && MotorFeedbackTimeoutMs>0 && MotorFeedbackTimeoutMs<=50;
}
bool GetImuSnapshot(ImuSnapshot *out) {
    if (!out || app_init_status!=TX_SUCCESS || !tx_thread_identify() ||
        tx_mutex_get(&app_data_mutex,TX_NO_WAIT)!=TX_SUCCESS) return false;
    *out=app_imu;
    AppCheck(tx_mutex_put(&app_data_mutex));
    return true; // caller additionally checks valid, sampleMs and per-field validity
}
bool GetAppHealth(AppHealth *out) {
    if (!out || app_init_status!=TX_SUCCESS || !tx_thread_identify() ||
        tx_mutex_get(&app_data_mutex,TX_NO_WAIT)!=TX_SUCCESS) return false;
    *out=app_health;
    AppCheck(tx_mutex_put(&app_data_mutex));
    return true;
}
bool SetMotorCommand(bool enable,float positionRad) {
    if (app_init_status!=TX_SUCCESS || !tx_thread_identify() ||
        (enable && (!std::isfinite(positionRad) || std::fabs(positionRad)>AppConfig::MaxPosition))) return false;
#if !RM_ENABLE_MOTOR_OUTPUT
    if (enable) return false;
#endif
    AppLock lock;
    if (enable && (!MotorConfigValid() || !app_health.motorOnline ||
                   app_health.controlInit!=InitState::Ready ||
                   (app_health.control==ControlState::Fault && app_control_command.enabled))) return false;
    if (enable && !app_control_command.enabled) ++app_control_command.armGeneration;
    app_control_command.enabled=enable;
    app_control_command.positionRad=enable ? positionRad : 0;
    return true;
}
void AppInitChanged() { AppCheck(tx_semaphore_put(&init_semaphore)); }
void AppPulse(TaskId task) {
    const unsigned i=task==TaskId::Imu ? 0 : 1;
    const ULONG now=tx_time_get();
    if (ULONG(now-last_pulse[i])>=AppTicks(AppConfig::HeartbeatPeriodMs)) {
        AppCheck(tx_semaphore_put(i==0 ? &imu_semaphore : &control_semaphore));
        last_pulse[i]=now;
    }
}
void AppSleepUntil(ULONG &deadline,ULONG period) {
    deadline+=period;
    const ULONG now=tx_time_get();
    const int32_t remaining=static_cast<int32_t>(static_cast<uint32_t>(deadline-now));
    if (remaining>0) AppCheck(tx_thread_sleep(static_cast<ULONG>(remaining)));
    else {
        { AppLock lock; ++app_health.missedDeadlines; }
        deadline=now;
        AppCheck(tx_thread_sleep(1)); // no catch-up busy loop, always yield on overrun
    }
}
extern "C" void tx_application_define(void *memory) {
    (void)memory;
    static_assert(AppConfig::ControlPeriodMs>0 && AppConfig::ImuPeriodMs>0);
    AppCheck(tx_thread_stack_error_notify(stack_error));
    AppCheck(tx_byte_pool_create(&thread_pool,const_cast<CHAR *>("thread stacks"),
                                 pool_storage,sizeof(pool_storage)));
    AppCheck(tx_mutex_create(&app_data_mutex,const_cast<CHAR *>("app snapshots"),TX_INHERIT));
    AppCheck(tx_semaphore_create(&init_semaphore,const_cast<CHAR *>("init events"),0));
    AppCheck(tx_semaphore_create(&imu_semaphore,const_cast<CHAR *>("imu heartbeat"),0));
    AppCheck(tx_semaphore_create(&control_semaphore,const_cast<CHAR *>("control heartbeat"),0));
    create(&alive_thread,"alive",alive_thread_entry,1536,4);
    create(&imu_thread,"imu",imu_thread_entry,4096,8);
    create(&control_thread,"control",control_thread_entry,3072,6);
    app_init_status=TX_SUCCESS;
    AppInitChanged(); // reached only after main's bsp_Init succeeded
    AppCheck(tx_thread_resume(&alive_thread));
    AppCheck(tx_thread_resume(&imu_thread));
    AppCheck(tx_thread_resume(&control_thread));
}
extern "C" void app_threadx_start(void) {
    tx_kernel_enter();
    Error_Handler();
}
