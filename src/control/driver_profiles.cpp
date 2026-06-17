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
        config::DriverName::Default,
        DriveMode::DoubleStickArcade, // Matches the AON-style split arcade base.
        5,                            // Retest for stick drift on the match controller.
        12.0,                         // Minimum output that reliably moves the drivetrain.
        1.132,                        // Soften if drivers over-correct near center.
        12.0,                         // Minimum turn output.
        1.132,                        // Tune with full robot weight.
        0.5,                          // Keeps arcade drive from clipping one side too early.
        127,                          // Full driver speed until practice says otherwise.
        0.45                          // Precision mode for lining up game-object pickups.
    };
}

DriverProfile kevinDriverProfile() {
    DriverProfile profile = defaultDriverProfile();
    profile.driver = config::DriverName::Kevin;
    profile.mode = DriveMode::DoubleStickArcade;
    profile.precisionSpeedScale = 0.45;
    return profile;
}

DriverProfile fabianDriverProfile() {
    DriverProfile profile = defaultDriverProfile();
    profile.driver = config::DriverName::Fabian;
    profile.mode = DriveMode::DoubleStickArcade;
    profile.precisionSpeedScale = 0.50;
    return profile;
}

DriverProfile profileForDriver(config::DriverName driver) {
    switch (driver) {
    case config::DriverName::Kevin:
        return kevinDriverProfile();
    case config::DriverName::Fabian:
        return fabianDriverProfile();
    case config::DriverName::Default:
        return defaultDriverProfile();
    }
    return defaultDriverProfile();
}

} // namespace control
