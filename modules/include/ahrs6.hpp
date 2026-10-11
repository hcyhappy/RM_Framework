#pragma once
struct Attitude6 {
    float w = 1, x{}, y{}, z{}; // body -> world, scalar-first unit quaternion
    float roll{}, pitch{}, yaw{}; // intrinsic ZYX rad, yaw has no absolute reference
};
class Ahrs6 {
public:
    void Reset();
    // acc: specific force m/s^2 (+Z at level rest), gyro: rad/s, dt: seconds.
    bool Update(float ax,float ay,float az,float gx,float gy,float gz,float dt);
    const Attitude6 &Get() const { return attitude_; }
private:
    Attitude6 attitude_{};
    bool initialized_ = false;
};
