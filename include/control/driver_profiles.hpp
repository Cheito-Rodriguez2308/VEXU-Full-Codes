#pragma once

namespace control {

enum class DriveMode {
    Tank,
    SingleStickArcade,
    DoubleStickArcade,
    Curvature
};

const char* toString(DriveMode mode);

struct DriverProfile {
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

} // namespace control
