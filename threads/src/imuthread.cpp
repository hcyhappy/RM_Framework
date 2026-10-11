#include "threads_internal.hpp"
#include "app_config.hpp"
#include "BMI088.hpp"
#include "bsp_time.hpp"
namespace {
void setInit(InitState state) {
    { AppLock lock; app_health.imuInit=state; app_health.imuDataOk=false;
      app_imu.valid=false; app_imu.attitudeValid=false; app_imu.temperatureValid=false; }
    AppInitChanged();
}
}
void imu_thread_entry(ULONG) {
    BMI088::cBMI088 bmi;
#if RM_ENABLE_AHRS6
    Ahrs6 ahrs;
    uint32_t previous=0;
    bool previousValid=false;
#endif
    ImuSnapshot data;
    bool ready=false;
    uint32_t retry=0, temp=0;
    bool firstAttempt=true, firstTemperature=true;
    float cachedTemperature=0;
    unsigned failures=0;
#if RM_ENABLE_IST8310
    IST8310 mag;
    bool magReady=false, firstMag=true;
    uint32_t magRetry=0, magPoll=0;
#endif
    ULONG deadline=tx_time_get();
    for (;;) {
        uint32_t now=BSP_GetTickMs();
        if (!ready && (firstAttempt || uint32_t(now-retry)>=AppConfig::InitRetryMs)) {
            firstAttempt=false;
            setInit(InitState::Pending);
            bmi.Config(); // bounded driver delays use ThreadX sleep
            ready=bmi.Ready(); retry=BSP_GetTickMs();
            setInit(ready ? InitState::Ready : InitState::Failed);
            data.valid=data.attitudeValid=data.temperatureValid=false;
#if RM_ENABLE_AHRS6
            previousValid=false; ahrs.Reset();
#endif
            firstTemperature=true; failures=0;
            deadline=tx_time_get();
        }
        now=BSP_GetTickMs();
        if (ready) {
            acc_data_t acceleration{};
            gyro_data_t angular{};
            bmi.ReadAccData(&acceleration);
            const bool accOk=bmi.LastStatus()==HAL_OK;
            bmi.ReadGyroData(&angular);
            const bool sampleOk=accOk && bmi.LastStatus()==HAL_OK;
            data.valid=sampleOk;
            if (sampleOk) {
                data.acceleration=acceleration;
                data.angularVelocity=angular;
                data.sampleMs=BSP_GetTickMs(); ++data.sequence; failures=0;
#if RM_ENABLE_AHRS6
                const float dt=previousValid ? uint32_t(data.sampleMs-previous)*0.001f :
                                               AppConfig::ImuPeriodMs*0.001f;
                if (!previousValid || dt>0.05f) ahrs.Reset();
                data.attitudeValid=ahrs.Update(acceleration.x,acceleration.y,acceleration.z,
                    angular.x,angular.y,angular.z,dt>0.05f ? AppConfig::ImuPeriodMs*0.001f : dt);
                if (data.attitudeValid) data.attitude=ahrs.Get();
                previous=data.sampleMs; previousValid=true;
#endif
            } else {
                data.attitudeValid=false;
#if RM_ENABLE_AHRS6
                previousValid=false;
#endif
                { AppLock lock; ++app_health.imuReadErrors; }
                if (++failures>=3) { ready=false; retry=now; setInit(InitState::Failed); }
            }
            if (sampleOk && (firstTemperature || uint32_t(now-temp)>=AppConfig::TemperaturePeriodMs)) {
                float temperature=0;
                bmi.ReadAccTemperature(&temperature);
                data.temperatureValid=bmi.LastStatus()==HAL_OK;
                if (data.temperatureValid) { cachedTemperature=temperature; data.temperatureMs=BSP_GetTickMs(); }
                temp=now; firstTemperature=false;
            }
            data.acceleration.temperature=cachedTemperature;
        }
#if RM_ENABLE_IST8310
        if (!magReady && (firstMag || uint32_t(now-magRetry)>=AppConfig::InitRetryMs)) {
            firstMag=false; magReady=mag.Init()==HAL_OK; magRetry=BSP_GetTickMs();
            data.magneticValid=false;
        }
        if (magReady && uint32_t(now-magPoll)>=AppConfig::MagneticPeriodMs) {
            const auto status=mag.Read(&data.magneticField); magPoll=now;
            if (status==HAL_OK) { data.magneticValid=true; data.magneticMs=BSP_GetTickMs(); }
            else if (status!=HAL_BUSY) { data.magneticValid=false; magReady=false; magRetry=now; }
            else if (uint32_t(now-(data.magneticValid ? data.magneticMs : magRetry))>100) {
                data.magneticValid=false; magReady=false; magRetry=now;
            }
        }
#endif
        { AppLock lock; app_imu=data; app_health.imuDataOk=data.valid && ready; }
        AppPulse(TaskId::Imu); // progress heartbeat continues even when sensor is disconnected
        AppSleepUntil(deadline,AppTicks(AppConfig::ImuPeriodMs));
    }
}
