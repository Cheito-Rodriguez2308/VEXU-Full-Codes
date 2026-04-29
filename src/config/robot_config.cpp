#include "config/robot_config.hpp"

namespace config {

static lemlib::ControllerSettings placeholderLateral() {
    // TODO_PLACEHOLDER PID values. Tune kP and kD first; keep kI at zero until the team has evidence it is needed.
    return lemlib::ControllerSettings(8.0, 0.0, 28.0, 3.0, 1.0, 100.0, 3.0, 500.0, 8.0);
}

static lemlib::ControllerSettings placeholderAngular() {
    // TODO_PLACEHOLDER PID values. Angular tuning should be done after IMU calibration and drivetrain checks.
    return lemlib::ControllerSettings(3.0, 0.0, 18.0, 3.0, 1.0, 100.0, 3.0, 500.0, 0.0);
}

const RobotConfig& robotAConfig() {
    static const RobotConfig config{
        RobotIdentity::RobotA,
        robotAPorts(),
        {
            pros::E_MOTOR_GEARSET_06, // TODO_PLACEHOLDER blue cartridge.
            12.5,                     // TODO_PLACEHOLDER track width measured wheel-center to wheel-center.
            3.25,                     // TODO_PLACEHOLDER drive wheel diameter.
            360.0,                    // TODO_PLACEHOLDER drivetrain RPM after external gearing.
            2.0,                      // TODO_PLACEHOLDER horizontal drift; start near 2 for drift drive, 8 for traction-center.
            {
                2.75,  // TODO_PLACEHOLDER tracking wheel diameter.
                1.0,   // TODO_PLACEHOLDER tracking wheel gear ratio.
                -1.25, // TODO_PLACEHOLDER vertical wheel offset; sign matters.
                -2.75, // TODO_PLACEHOLDER horizontal wheel offset; sign matters.
                false, // TODO_PLACEHOLDER direction.
                true   // TODO_PLACEHOLDER direction.
            }
        },
        {placeholderLateral(), placeholderAngular()}
    };
    return config;
}

const RobotConfig& robotBConfig() {
    static const RobotConfig config{
        RobotIdentity::RobotB,
        robotBPorts(),
        {
            pros::E_MOTOR_GEARSET_06, // TODO_PLACEHOLDER blue cartridge.
            12.0,                     // TODO_PLACEHOLDER Robot B track width.
            3.25,                     // TODO_PLACEHOLDER Robot B wheel diameter.
            360.0,                    // TODO_PLACEHOLDER Robot B drivetrain RPM.
            2.0,                      // TODO_PLACEHOLDER Robot B horizontal drift.
            {
                2.75,  // TODO_PLACEHOLDER Robot B tracking wheel diameter.
                1.0,   // TODO_PLACEHOLDER Robot B tracking wheel gear ratio.
                -1.0,  // TODO_PLACEHOLDER Robot B vertical offset.
                -2.5,  // TODO_PLACEHOLDER Robot B horizontal offset.
                false, // TODO_PLACEHOLDER direction.
                false  // TODO_PLACEHOLDER direction.
            }
        },
        {placeholderLateral(), placeholderAngular()}
    };
    return config;
}

const RobotConfig& activeRobotConfig() {
    return selectedRobot() == RobotIdentity::RobotA ? robotAConfig() : robotBConfig();
}

} // namespace config
