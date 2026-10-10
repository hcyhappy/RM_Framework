#include "pid.hpp"
#include "crc.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <cstdio>
#include <cstdint>
#include <initializer_list>
static void near(float a,float b) { assert(std::fabs(a-b)<1e-4f); }
static uint16_t crc16(const uint8_t *d,unsigned n,uint16_t v) {
    while(n--) { v^=*d++; for(unsigned b=0;b<8;++b) v=(v&1)?(v>>1)^0x8408:v>>1; }
    return v;
}
static uint8_t crc8(const uint8_t *d,unsigned n,uint8_t v) {
    while(n--) { v^=*d++; for(unsigned b=0;b<8;++b) v=(v&1)?(v>>1)^0x8c:v>>1; }
    return v;
}
int main() {
    PID pos(2,0.5f,1,100,50),delta(2,0.5f,1,100,50,PID_DELTA);
    float errors[]={2,3,-1,0,4};
    float expected[]={7,9.5f,-4,3,16};
    for(unsigned k=0;k<5;++k) {
        pos.ref=delta.ref=errors[k]; pos.UpdateResult(); delta.UpdateResult();
        near(pos.result,expected[k]); near(delta.result,expected[k]);
    }
    near(pos.err[0],4); near(pos.err[1],0); near(pos.err[2],-1);
    near(delta.pResult,8); near(delta.dResult,3); near(delta.iResult,2);
    for(auto mode:{PID_POSITION,PID_DELTA}) {
        PID p(0,2,0,100,3,mode);
        p.ref=1; p.UpdateResult(); near(p.result,2); near(p.GetIntegralOutput(),2);
        p.UpdateResult(); near(p.result,3); near(p.GetIntegralOutput(),3);
        p.UpdateResult(); near(p.result,3);
        near(p.iResult,mode==PID_DELTA?0:3);
        p.ref=-1; p.UpdateResult(); near(p.result,1);
        p.UpdateResult(); near(p.result,-1);
        p.UpdateResult(); near(p.result,-3);
        p.ref=1; p.UpdateResult(); near(p.result,-1);
        p.maxIOut=0; p.UpdateResult(); near(p.GetIntegralOutput(),0);
        p.Clear(); near(p.result,0); near(p.ref,0); near(p.fdb,0); near(p.GetIntegralOutput(),0);
        PID clip(10,0,0,5,0,mode);
        clip.ref=1; clip.UpdateResult(); near(clip.result,5);
        clip.ref=-1; clip.UpdateResult(); near(clip.result,-5);
        PID zero(1,1,1,0,0,mode); zero.ref=5; zero.UpdateResult(); near(zero.result,0);
    }
    PID d(1,0,0,100,0,PID_DELTA); d.ref=4; d.UpdateResult(); d.UpdateResult(); near(d.result,4);
    PID reversal(0,10,0,100,3,PID_DELTA);
    reversal.ref=1; reversal.UpdateResult(); near(reversal.GetIntegralOutput(),3);
    reversal.ref=-1; reversal.UpdateResult(); near(reversal.GetIntegralOutput(),-3);
    near(reversal.iResult,-6); near(reversal.result,-3);
    uint32_t random=1;
    PID bounded_pos(5,0.4f,2,7,3),bounded_delta(5,0.4f,2,7,3,PID_DELTA);
    for(unsigned k=0;k<10000;++k) {
        random=random*1664525U+1013904223U;
        const float error=static_cast<int>(random%201)-100;
        for(auto *p:{&bounded_pos,&bounded_delta}) {
            p->ref=error; p->UpdateResult();
            assert(std::isfinite(p->result) && std::fabs(p->result)<=7);
            assert(std::fabs(p->GetIntegralOutput())<=3);
        }
    }
    d.ref=6; d.UpdateResult(); near(d.result,6); near(d.pResult,2);
    d.mode=PID_POSITION; d.UpdateResult(); near(d.result,6);
    d.mode=PID_DELTA; d.UpdateResult(); near(d.result,6);
    PID gain(1,1,1,100,10); gain.Tuning(2,3,4); near(gain.kp,2); near(gain.ki,3); near(gain.kd,4);
    const float nan=std::numeric_limits<float>::quiet_NaN(),inf=std::numeric_limits<float>::infinity();
    gain.Tuning(nan,9,9); near(gain.kp,2); near(gain.ki,3);
    gain.ref=nan; gain.UpdateResult(); near(gain.result,0); near(gain.GetIntegralOutput(),0);
    gain.ref=1; gain.UpdateResult(); near(gain.result,9);
    gain.maxOut=-1; gain.UpdateResult(); near(gain.result,0);
    gain.maxOut=inf; gain.UpdateResult(); near(gain.result,0);
    PID invalid(1,1,1,10,10,257); invalid.ref=1; invalid.UpdateResult(); near(invalid.result,0);
    PID large(1e30f,0,0,12,0); large.ref=1e30f; large.UpdateResult(); near(large.result,12);
    assert(std::isfinite(large.pResult));
    large.ref=std::numeric_limits<float>::max(); large.fdb=-large.ref; large.UpdateResult(); near(large.result,0);
    PID dirty(1,0,0,10,1,PID_DELTA); dirty.result=nan; dirty.ref=1; dirty.UpdateResult(); near(dirty.result,0);
    near(Numeric::LimitABS(3,2),2); near(Numeric::LimitABS(-3,2),-2);
    near(Numeric::LimitABS(1,2),1); near(Numeric::LimitABS(1,-2),0);
    near(Numeric::LimitABS(nan,2),0); near(Numeric::LimitABS(inf,2),2);
    near(Numeric::LimitABS(1,inf),0); near(Numeric::LimitABS(1,0),0);
    const uint8_t text[]={'1','2','3','4','5','6','7','8','9'};
    assert(Get_CRC8_Check_Sum(text,9,0xff)==0x0b);
    assert(Get_CRC16_Check_Sum(text,9,0xffff)==0x6f91);
    uint8_t data[64]; uint32_t seed=7;
    for(unsigned n=0;n<64;++n) {
        seed=seed*1664525U+1013904223U; data[n]=seed>>24;
        assert(Get_CRC8_Check_Sum(data,n,0xff)==crc8(data,n,0xff));
        assert(Get_CRC16_Check_Sum(data,n,0xffff)==crc16(data,n,0xffff));
    }
    uint8_t frame[16]{}; for(unsigned i=0;i<14;++i) frame[i]=i;
    Append_CRC16_Check_Sum(frame,16); assert(Verify_CRC16_Check_Sum(frame,16));
    assert(frame[14]==(crc16(frame,14,0xffff)&0xff) && frame[15]==(crc16(frame,14,0xffff)>>8));
    frame[0]^=1; assert(!Verify_CRC16_Check_Sum(frame,16));
    Append_CRC8_Check_Sum(frame,16); assert(Verify_CRC8_Check_Sum(frame,16));
    frame[0]^=1; assert(!Verify_CRC8_Check_Sum(frame,16));
    assert(Get_CRC8_Check_Sum(nullptr,3,0x12)==0x12);
    assert(Get_CRC16_Check_Sum(nullptr,3,0x1234)==0x1234);
    assert(!Verify_CRC8_Check_Sum(nullptr,8) && !Verify_CRC16_Check_Sum(nullptr,8));
    Append_CRC8_Check_Sum(nullptr,8); Append_CRC16_Check_Sum(nullptr,8);
    uint8_t shortbuf[2]={0x12,0x34}; Append_CRC16_Check_Sum(shortbuf,2); Append_CRC8_Check_Sum(shortbuf,2);
    assert(shortbuf[0]==0x12 && shortbuf[1]==0x34);
    std::puts("PASS: PID formulas, limits, recovery, CRC references and numeric guards");
}
