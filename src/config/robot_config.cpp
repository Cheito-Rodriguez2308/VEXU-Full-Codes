#include "config/robot_config.hpp"

namespace config {

static lemlib::ControllerSettings todoLateralSettings() {
    // TODO: lateral PID.
    // Controlled variable: distance error in inches from LemLib odometry.
    // Sensor source: tracking wheels plus IMU.
    // Tuning order: kP until it reaches the target, kD until overshoot settles, slew only if it slips.
    // Keep kI at zero unless steady-state error remains after mechanical checks.
    return lemlib::ControllerSettings(8.0, 0.0, 28.0, 3.0, 1.0, 100.0, 3.0, 500.0, 8.0);
}

static lemlib::ControllerSettings todoAngularSettings() {
    // TODO: angular PID.
    // Controlled variable: heading error in degrees.
    // Sensor source: IMU through LemLib odometry.
    // Tuning order: kP for response, kD for damping, exit ranges for reliable settle time.
    // Angular slew is usually left disabled unless the robot tips or breaks traction.
    return lemlib::ControllerSettings(3.0, 0.0, 18.0, 3.0, 1.0, 100.0, 3.0, 500.0, 0.0);
}

const RobotConfig& robotAConfig() {
    static const RobotConfig config{
        RobotIdentity::RobotA,
        robotAPorts(),
        {
            pros::E_MOTOR_GEARSET_06, // TODO: drive cartridge.
            12.5,                     // TODO: track width, wheel center to wheel center.
            3.25,                     // TODO: drive wheel diameter.
            360.0,                    // TODO: drivetrain RPM after external gearing.
            2.0,                      // TODO: horizontal drift; start near 2 or 8 depending on drive style.
            {
                2.75,  // TODO: tracking wheel diameter.
                1.0,   // TODO: tracking wheel gear ratio.
                -1.25, // TODO: vertical wheel offset; sign matters.
                -2.75, // TODO: horizontal wheel offset; sign matters.
                false, // TODO: vertical wheel direction.
                true   // TODO: horizontal wheel direction.
            }
        },
        {todoLateralSettings(), todoAngularSettings()}
    };
    return config;
}

const RobotConfig& robotBConfig() {
    static const RobotConfig config{
        RobotIdentity::RobotB,
        robotBPorts(),
        {
            pros::E_MOTOR_GEARSET_06, // TODO: Robot B drive cartridge.
            12.0,                     // TODO: Robot B track width.
            3.25,                     // TODO: Robot B wheel diameter.
            360.0,                    // TODO: Robot B drivetrain RPM.
            2.0,                      // TODO: Robot B horizontal drift.
            {
                2.75,  // TODO: Robot B tracking wheel diameter.
                1.0,   // TODO: Robot B tracking wheel gear ratio.
                -1.0,  // TODO: Robot B vertical offset.
                -2.5,  // TODO: Robot B horizontal offset.
                false, // TODO: Robot B vertical wheel direction.
                false  // TODO: Robot B horizontal wheel direction.
            }
        },
        {todoLateralSettings(), todoAngularSettings()}
    };
    return config;
}

const RobotConfig& activeRobotConfig() {
    return selectedRobot() == RobotIdentity::RobotA ? robotAConfig() : robotBConfig();
}

} // namespace config
