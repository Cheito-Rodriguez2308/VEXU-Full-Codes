#include "control/driver_profiles.hpp"

namespace control {

const char* toString(DriveMode mode) {
    switch (mode) {
    case DriveMode::Tank: return "Tank";
    case DriveMode::SingleStickArcade: return "Single Stick Arcade";
    case DriveMode::DoubleStickArcade: return "Double Stick Arcade";
    case DriveMode::Curvature: return "Curvature";
    }
    return "Unknown";
}

DriverProfile defaultDriverProfile() {
    return {
        DriveMode::DoubleStickArcade, // TODO_PLACEHOLDER preferred driver style.
        5,                            // TODO_PLACEHOLDER joystick deadband.
        12.0,                         // TODO_PLACEHOLDER throttle minimum output.
        1.132,                        // TODO_PLACEHOLDER throttle expo gain.
        12.0,                         // TODO_PLACEHOLDER steer minimum output.
        1.132,                        // TODO_PLACEHOLDER steer expo gain.
        0.5,                          // TODO_PLACEHOLDER arcade desaturate bias.
        127,                          // TODO_PLACEHOLDER max speed.
        0.45                          // TODO_PLACEHOLDER precision mode speed scale.
    };
}

} // namespace control
