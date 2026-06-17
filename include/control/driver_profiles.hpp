#pragma once

#include "config/robot_identity.hpp"

namespace control {

enum class DriveMode {
    Tank,
    SingleStickArcade,
    DoubleStickArcade,
    Curvature
};

const char* toString(DriveMode mode);

struct DriverProfile {
    config::DriverName driver;
    DriveMode mode;
    int deadband;
    double throttleMinOutput;
    double throttleCurveGain;
    double steerMinOutput;
    double steerCurveGain;
    double desaturateBias;
    int maxSpeed;
    double precisionSpeedScale;
};

DriverProfile defaultDriverProfile();
DriverProfile kevinDriverProfile();
DriverProfile fabianDriverProfile();
DriverProfile profileForDriver(config::DriverName driver);

} // namespace control
