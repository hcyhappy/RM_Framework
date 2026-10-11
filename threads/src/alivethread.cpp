#include "threads_internal.hpp"
#include "app_config.hpp"
#include "LED.hpp"
namespace {
bool drain(TX_SEMAPHORE *semaphore) {
    bool received=false;
    for (unsigned n=0;n<16;++n) {
        const UINT s=tx_semaphore_get(semaphore,TX_NO_WAIT);
        if (s==TX_NO_INSTANCE) break;
        AppCheck(s);
        received=true;
    }
    return received;
}
}
void alive_thread_entry(ULONG) {
    const ULONG boot=tx_time_get();
    ULONG imuSeen=boot, controlSeen=boot, deadline=boot;
    bool imuEver=false, controlEver=false;
    for (;;) {
        const ULONG now=tx_time_get();
        drain(&init_semaphore); // init producers update statuses, then put an event
        if (drain(&imu_semaphore)) { imuSeen=now; imuEver=true; }
        if (drain(&control_semaphore)) { controlSeen=now; controlEver=true; }
        app_imu_alive=imuEver && ULONG(now-imuSeen)<AppTicks(AppConfig::HeartbeatTimeoutMs);
        app_control_alive=controlEver && ULONG(now-controlSeen)<AppTicks(AppConfig::HeartbeatTimeoutMs);
        AppHealth health;
        { AppLock lock; health=app_health; }
        const bool grace=ULONG(now-boot)<AppTicks(AppConfig::StartupGraceMs);
        const bool fault=app_init_status!=TX_SUCCESS || app_stack_error ||
            health.imuInit==InitState::Failed || health.controlInit==InitState::Failed ||
            health.control==ControlState::Fault ||
            (!grace && (!app_imu_alive || !app_control_alive || !health.imuDataOk));
        const bool ready=!fault && app_imu_alive && app_control_alive && health.imuDataOk &&
            health.imuInit==InitState::Ready && health.controlInit==InitState::Ready;
        // Only alive owns runtime LED effects. Blue heartbeat: scheduler is making progress.
        LED::Set(LED::Color::Blue,(app_alive_count%10)<2);
        LED::Set(LED::Color::Red,fault || (!ready && (app_alive_count%4)<2));
        LED::Set(LED::Color::Green,ready ?
                 (health.control==ControlState::Running || (app_alive_count%10)<5) :
                 (!fault && (app_alive_count%4)<2));
        ++app_alive_count;
        AppSleepUntil(deadline,AppTicks(AppConfig::AlivePeriodMs));
    }
}
