#define RM_THREADS_TEST
#include "../modules/test_modules.cpp" // shared register-map/packet HAL fixture, its main excluded
#include "threads_internal.hpp"
#include "app_config.hpp"
#include "bsp_usart.hpp"
#include <functional>
#include <limits>
#include <string>
extern "C" void tx_application_define(void *);
struct Stop {};
struct Fatal {};
void Error_Handler() { throw Fatal{}; }
UART_HandleTypeDef huart1{1},huart6{6};
std::function<void()> onSleep;
unsigned sleeps=0, aborts=0;
bool createFail=false;
HAL_StatusTypeDef abortResult=HAL_OK;
std::vector<TX_THREAD *> created;
TX_THREAD *tx_thread_identify() { static TX_THREAD dummy; return &dummy; }
ULONG tx_time_get() { return tick; }
UINT tx_thread_sleep(ULONG n) { assert(n>0); tick+=n; ++sleeps; if(onSleep) onSleep(); return TX_SUCCESS; }
UINT tx_semaphore_create(TX_SEMAPHORE *s,CHAR *,ULONG count) { s->count=count; return TX_SUCCESS; }
UINT tx_semaphore_put(TX_SEMAPHORE *s) { ++s->count; return TX_SUCCESS; }
UINT tx_semaphore_get(TX_SEMAPHORE *s,ULONG wait) { assert(wait==TX_NO_WAIT); if(!s->count)return TX_NO_INSTANCE; --s->count; return TX_SUCCESS; }
UINT tx_mutex_create(TX_MUTEX *m,CHAR *,UINT inherit) { assert(inherit==TX_INHERIT); m->created=true; return TX_SUCCESS; }
UINT tx_mutex_get(TX_MUTEX *m,ULONG wait) { assert(m->created); if(m->locked) { assert(wait==TX_NO_WAIT); return TX_NOT_DONE; } m->locked=true; return TX_SUCCESS; }
UINT tx_mutex_put(TX_MUTEX *m) { assert(m->locked); m->locked=false; return TX_SUCCESS; }
UINT tx_byte_pool_create(TX_BYTE_POOL *p,CHAR *,void *mem,ULONG size) { *p={static_cast<UCHAR *>(mem),size,0}; return TX_SUCCESS; }
UINT tx_byte_allocate(TX_BYTE_POOL *p,void **out,ULONG n,ULONG wait) { assert(wait==TX_NO_WAIT && p->used+n<=p->size); *out=p->storage+p->used; p->used+=n; return TX_SUCCESS; }
UINT tx_thread_stack_error_notify(void (*)(TX_THREAD *)) { return TX_SUCCESS; }
UINT tx_thread_create(TX_THREAD *t,CHAR *,void (*entry)(ULONG),ULONG,void *,ULONG bytes,UINT pri,UINT threshold,ULONG slice,UINT start) {
    if(createFail && created.size()==2) return TX_NOT_DONE;
    assert(start==TX_DONT_START && threshold==pri && slice==TX_NO_TIME_SLICE);
    *t={entry,bytes,pri,false}; created.push_back(t); return TX_SUCCESS;
}
UINT tx_thread_resume(TX_THREAD *t) { assert(created.size()==3 && app_init_status==TX_SUCCESS); t->resumed=true; return TX_SUCCESS; }
void tx_kernel_enter() { assert(false); }
HAL_StatusTypeDef CAN_AbortPending(CAN_HandleTypeDef *) { ++aborts; return abortResult; }
HAL_StatusTypeDef USART_Service(UART_HandleTypeDef *) { return HAL_OK; }
void run(void (*entry)(ULONG)) { try { entry(0); assert(false); } catch(const Stop &) {} }
void feedback() { CAN_RxFrame f{0x201,8,{}}; be(f.data,1000); incoming[0].push_back(f); }
void startup() {
    tx_application_define(nullptr);
    assert(created.size()==3 && created[0]->priority==4 && created[1]->priority==8 && created[2]->priority==6);
    for(auto *t:created) assert(t->resumed && t->bytes>=1536);
    assert(init_semaphore.count==1);
    ImuSnapshot snapshot; assert(GetImuSnapshot(&snapshot) && !snapshot.valid);
    app_data_mutex.locked=true; assert(!GetImuSnapshot(&snapshot)); app_data_mutex.locked=false;
    assert(!SetMotorCommand(true,std::numeric_limits<float>::quiet_NaN()));
    assert(!SetMotorCommand(true,0)); // no motor feedback
    ULONG deadline=0xfffffffcu; tick=deadline;
    AppSleepUntil(deadline,8); assert(tick==4); // 32-bit tick wrap
}
void testAhrs() {
    Ahrs6 a;
    for(int i=0;i<500;++i) assert(a.Update(0,0,9.80665f,0,0,1,0.002f));
    close(a.Get().yaw,1.0f);
    const auto old=a.Get();
    assert(!a.Update(0,0,9.80665f,NAN,0,0,0.002f)); close(a.Get().yaw,old.yaw);
    assert(!a.Update(0,0,9.80665f,0,0,0,0.1f));
    assert(a.Update(0,0,0,0,0,1,0.002f)); // gyro-only after initial alignment
    a.Reset();
    assert(!a.Update(0,0,0,0,0,0,0.002f));
    assert(a.Update(0,4.903325f,8.49281f,0,0,0,0.002f));
    close(a.Get().roll,0.5235988f);
    for(int i=0;i<2000;++i) assert(a.Update(0,0,9.80665f,0,0,0,0.002f));
    assert(std::fabs(a.Get().roll)<0.001f);
}
void testAlive() {
    app_health.imuInit=app_health.controlInit=InitState::Ready;
    app_health.imuDataOk=true;
    tick=0;
    tx_semaphore_put(&imu_semaphore); tx_semaphore_put(&control_semaphore);
    onSleep=[] {
        if(sleeps<3) { assert(app_imu_alive && app_control_alive); tx_semaphore_put(&control_semaphore); }
        if(sleeps>=3 && sleeps<25) tx_semaphore_put(&control_semaphore);
        if(sleeps==25) { assert(!app_imu_alive && app_control_alive && (ledOnMask & LED_R_Pin)); throw Stop{}; }
    };
    run(alive_thread_entry);
    assert(imu_semaphore.count==0); // old token cannot keep task alive forever
}
void testImu() {
    regmap[0][0]=0x1e; regmap[1][0]=0x0f;
    regmap[0][0x16]=0x55; regmap[0][0x17]=0x15; // close to +g in +/-6g
    mag[0]=0x10; mag[2]=1; mag[3]=10;
    bool failed=false;
    onSleep=[&] {
        if(app_imu.sequence==4 && !failed) { assert(app_imu.valid && app_imu.temperatureValid && app_imu.attitudeValid); failSPI=true; failed=true; }
        if(failed && app_health.imuInit==InitState::Failed) { assert(!app_imu.valid && app_health.imuReadErrors>=3); throw Stop{}; }
        assert(sleeps<1000);
    };
    run(imu_thread_entry);
    assert(imu_semaphore.count>0 && init_semaphore.count>=3);
}
void testControl() {
    feedback();
#if RM_ENABLE_MOTOR_OUTPUT
    onSleep=[] {
        const unsigned step=sleeps;
        feedback();
        if(step==1) { assert(app_health.motorOnline); assert(SetMotorCommand(true,0.1f)); }
        if(step==2) { assert(app_health.control==ControlState::Running); assert(app_health.currentCommand==40); assert(!packets.empty()); }
        if(step==3) assert(SetMotorCommand(false,0));
        if(step==4) { assert(app_health.currentCommand==0 && packets.back().bytes[1]==0); assert(SetMotorCommand(true,0.2f)); }
        if(step==5) tick+=20; // deadline failure latches fault
        if(step==6) { assert(app_health.control==ControlState::Fault && app_health.currentCommand==0); assert(!SetMotorCommand(true,0)); assert(SetMotorCommand(false,0)); assert(SetMotorCommand(true,0.1f)); }
        if(step==7) { assert(app_health.control==ControlState::Running); sendStatus=HAL_BUSY; }
        if(step==12) { assert(app_health.control==ControlState::Fault && app_health.canSendErrors>=5); assert(aborts>=3); throw Stop{}; }
    };
#else
    onSleep=[] { feedback(); assert(packets.empty()); assert(!SetMotorCommand(true,0.1f)); if(sleeps==5)throw Stop{}; };
#endif
    run(control_thread_entry);
}
void testOffline() {
#if RM_ENABLE_MOTOR_OUTPUT
    feedback();
    onSleep=[] {
        if(sleeps<=3) feedback();
        if(sleeps==1) assert(SetMotorCommand(true,0.1f));
        if(sleeps==31) { assert(app_health.control==ControlState::Fault && !app_health.motorOnline && app_health.currentCommand==0); feedback(); }
        if(sleeps==32) { assert(app_health.motorOnline && app_health.control==ControlState::Fault); assert(!SetMotorCommand(true,0)); assert(SetMotorCommand(false,0)); assert(SetMotorCommand(true,0.1f)); feedback(); }
        if(sleeps==33) { assert(app_health.control==ControlState::Running); throw Stop{}; }
    };
    run(control_thread_entry);
#endif
}
void testAbort() {
#if RM_ENABLE_MOTOR_OUTPUT
    feedback();
    onSleep=[] {
        feedback();
        if(sleeps==1) { assert(SetMotorCommand(true,0.1f)); abortResult=HAL_BUSY; }
        if(sleeps>=2 && sleeps<=6) assert(packets.empty());
        if(sleeps==6) { assert(app_health.control==ControlState::Fault && app_health.currentCommand==0); abortResult=HAL_OK; }
        if(sleeps==7) { assert(!packets.empty() && packets.back().bytes[1]==0); throw Stop{}; }
    };
    run(control_thread_entry);
#endif
}
int main(int argc,char **argv) {
    assert(argc==2);
    const std::string mode=argv[1];
    if(mode=="ahrs") testAhrs();
    else if(mode=="initfail") {
        createFail=true;
        try { tx_application_define(nullptr); assert(false); } catch(const Fatal &) {}
        assert(app_init_status==TX_NOT_DONE && created.size()==2 && (ledOnMask & LED_R_Pin));
        for(auto *t:created) assert(!t->resumed);
    }
    else {
        startup(); sleeps=0; tick=0;
        if(mode=="alive")testAlive(); else if(mode=="imu")testImu(); else if(mode=="control")testControl(); else if(mode=="abort")testAbort(); else if(mode=="offline")testOffline(); else assert(mode=="startup");
    }
    std::cout<<"Thread test passed: "<<mode<<"\n";
}
