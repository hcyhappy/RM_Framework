#include "ahrs6.hpp"
#include <algorithm>
#include <cmath>
void Ahrs6::Reset() { attitude_ = {}; initialized_ = false; }
bool Ahrs6::Update(float ax,float ay,float az,float gx,float gy,float gz,float dt) {
    if (!std::isfinite(ax)||!std::isfinite(ay)||!std::isfinite(az)||
        !std::isfinite(gx)||!std::isfinite(gy)||!std::isfinite(gz)||
        !std::isfinite(dt)||dt<=0||dt>0.05f) return false;
    const float norm = std::hypot(std::hypot(ax,ay),az);
    const bool gravityUsable = norm>0.8f*9.80665f && norm<1.2f*9.80665f;
    if (!initialized_) {
        if (!gravityUsable) return false;
        const float r=std::atan2(ay,az), p=std::atan2(-ax,std::hypot(ay,az));
        attitude_.w=std::cos(r/2)*std::cos(p/2);
        attitude_.x=std::sin(r/2)*std::cos(p/2);
        attitude_.y=std::cos(r/2)*std::sin(p/2);
        attitude_.z=-std::sin(r/2)*std::sin(p/2);
        initialized_=true;
    }
    const auto q=attitude_;
    if (gravityUsable) {
        ax/=norm; ay/=norm; az/=norm;
        const float vx=2*(q.x*q.z-q.w*q.y), vy=2*(q.w*q.x+q.y*q.z);
        const float vz=1-2*(q.x*q.x+q.y*q.y);
        // Mahony-style proportional gravity correction, Kp=2 s^-1, Ki=0.
        gx+=2*(ay*vz-az*vy); gy+=2*(az*vx-ax*vz); gz+=2*(ax*vy-ay*vx);
    }
    Attitude6 next=q;
    const float h=0.5f*dt;
    next.w+=(-q.x*gx-q.y*gy-q.z*gz)*h;
    next.x+=(q.w*gx+q.y*gz-q.z*gy)*h;
    next.y+=(q.w*gy-q.x*gz+q.z*gx)*h;
    next.z+=(q.w*gz+q.x*gy-q.y*gx)*h;
    const float n=std::hypot(std::hypot(next.w,next.x),std::hypot(next.y,next.z));
    if (!std::isfinite(n)||n<1e-6f) return false;
    next.w/=n; next.x/=n; next.y/=n; next.z/=n;
    next.roll=std::atan2(2*(next.w*next.x+next.y*next.z),1-2*(next.x*next.x+next.y*next.y));
    next.pitch=std::asin(std::clamp(2*(next.w*next.y-next.z*next.x),-1.0f,1.0f));
    next.yaw=std::atan2(2*(next.w*next.z+next.x*next.y),1-2*(next.y*next.y+next.z*next.z));
    attitude_=next;
    return true;
}
