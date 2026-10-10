#include "pid.hpp"
#include <cmath>
#include <limits>

namespace {
float clamp(double value, float limit)
{
    if (value > limit) return limit;
    if (value < -static_cast<double>(limit)) return -limit;
    return static_cast<float>(value);
}
}

PID::PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode)
    : mode(mode == PID_POSITION || mode == PID_DELTA ? mode : 0),
      kp(kp), ki(ki), kd(kd), maxOut(maxOut), maxIOut(maxIOut)
{
    Clear();
}

void PID::Tuning(float tuning_kp, float tuning_ki, float tuning_kd)
{
    if (!std::isfinite(tuning_kp) || !std::isfinite(tuning_ki) ||
        !std::isfinite(tuning_kd)) return;
    kp = tuning_kp;
    ki = tuning_ki;
    kd = tuning_kd;
}

void PID::UpdateResult()
{
    if (!std::isfinite(ref) || !std::isfinite(fdb) ||
        !std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd) ||
        !std::isfinite(maxOut) || !std::isfinite(maxIOut) ||
        maxOut < 0.0f || maxIOut < 0.0f ||
        (mode != PID_POSITION && mode != PID_DELTA))
    {
        ResetState();
        return;
    }
    if (mode != previous_mode_) ResetState();
    const double error = static_cast<double>(ref) - fdb;
    const double float_max = std::numeric_limits<float>::max();
    if (std::fabs(error) > float_max || !std::isfinite(result) ||
        !std::isfinite(err[0]) || !std::isfinite(err[1]) || !std::isfinite(err[2]))
    {
        ResetState();
        return;
    }
    err[2] = err[1];
    err[1] = err[0];
    err[0] = static_cast<float>(error);

    const float previous_integral = integral_;
    integral_ = clamp(static_cast<double>(integral_) + static_cast<double>(ki) * error,
                      maxIOut);
    double p, i, d, output;
    if (mode == PID_POSITION)
    {
        p = static_cast<double>(kp) * error;
        i = integral_;
        d = static_cast<double>(kd) * (error - err[1]);
        output = p + i + d;
    }
    else
    {
        p = static_cast<double>(kp) * (error - err[1]);
        i = static_cast<double>(integral_) - previous_integral;
        d = static_cast<double>(kd) * (error - 2.0 * err[1] + err[2]);
        output = static_cast<double>(result) + p + i + d;
    }
    // Use double intermediates to avoid overflow before applying float limits.
    pResult = clamp(p, std::numeric_limits<float>::max());
    iResult = clamp(i, std::numeric_limits<float>::max());
    dResult = clamp(d, std::numeric_limits<float>::max());
    result = clamp(output, maxOut);
}

void PID::ResetState()
{
    err[0] = err[1] = err[2] = 0.0f;
    pResult = iResult = dResult = result = 0.0f;
    integral_ = 0.0f;
    previous_mode_ = mode;
}

void PID::Clear()
{
    ref = fdb = 0.0f;
    ResetState();
}
