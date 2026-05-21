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
        DriveMode::DoubleStickArcade, // TODO: ask drivers before locking this in.
        5,                            // TODO: retest for stick drift on the match controller.
        12.0,                         // TODO: minimum output that reliably moves the drivetrain.
        1.132,                        // TODO: soften if drivers over-correct near center.
        12.0,                         // TODO: minimum turn output.
        1.132,                        // TODO: tune with full robot weight.
        0.5,                          // Keeps arcade drive from clipping one side too early.
        127,                          // Full driver speed until practice says otherwise.
        0.45                          // Precision mode for lining up game-object pickups.
    };
}

} // namespace control
