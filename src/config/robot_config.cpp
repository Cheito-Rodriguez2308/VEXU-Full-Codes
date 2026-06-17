#include "config/robot_config.hpp"

namespace config {

static lemlib::ControllerSettings aonLateralSettings() {
    // Starting point from AON drivePID. Retune with LemLib on this codebase.
    return lemlib::ControllerSettings(8.0, 0.0, 28.0, 3.0, 1.0, 100.0, 3.0, 500.0, 8.0);
}

static lemlib::ControllerSettings aonAngularSettings() {
    // Starting point from AON turnPID behavior. Check before trusting auton.
    return lemlib::ControllerSettings(3.0, 0.0, 18.0, 3.0, 1.0, 100.0, 3.0, 500.0, 0.0);
}

const RobotConfig& smallRobotConfig() {
    static const RobotConfig config{
        RobotIdentity::SmallRobot,
        smallRobotPorts(),
        {
            pros::E_MOTOR_GEARSET_06,
            12.5,
            2.75,
            450.0,
            2.0,
            {
                2.0,
                1.0,
                1.125,
                1.572,
                false,
                false
            }
        },
        {aonLateralSettings(), aonAngularSettings()}
    };
    return config;
}

const RobotConfig& bigRobotConfig() {
    static const RobotConfig config{
        RobotIdentity::BigRobot,
        bigRobotPorts(),
        {
            pros::E_MOTOR_GEARSET_06,
            15.5,
            3.25,
            780.0,
            2.0,
            {
                2.0,
                1.0,
                1.572,
                1.572,
                false,
                false
            }
        },
        {aonLateralSettings(), aonAngularSettings()}
    };
    return config;
}

const RobotConfig& activeRobotConfig() {
    return selectedRobot() == RobotIdentity::BigRobot ? bigRobotConfig() : smallRobotConfig();
}

} // namespace config
