#ifndef MATH_HPP
#define MATH_HPP

namespace Numeric
{
    // Symmetric clamp. Invalid limit/NaN input -> 0; infinite input saturates.
    float LimitABS(float input, float maxValue);
}

#endif // MATH_HPP
