#pragma once

namespace util {

double clamp(double value, double min, double max);
int applyDeadband(int value, int deadband);
int scaleSpeed(int value, double scale);

} // namespace util
