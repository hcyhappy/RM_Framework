#include "math.hpp"
#include <cmath>

namespace Numeric
{
    float LimitABS(float input, float maxValue)
    {
        if (!std::isfinite(maxValue) || maxValue < 0.0f || std::isnan(input))
            return 0.0f;
        if (input > maxValue)
            return maxValue;
        if (input < -maxValue)
            return -maxValue;
        return input;
    }

}
