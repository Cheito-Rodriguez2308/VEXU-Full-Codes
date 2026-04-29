#include "util/math_utils.hpp"
#include <cmath>

namespace util {

double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

int applyDeadband(int value, int deadband) {
    return std::abs(value) < deadband ? 0 : value;
}

int scaleSpeed(int value, double scale) {
    return static_cast<int>(clamp(value * scale, -127.0, 127.0));
}

} // namespace util
