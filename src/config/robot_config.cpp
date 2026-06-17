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
            2.75,
            2.0,
            1.125,
            1.125,
            1.572,
            0.75,
            12.5,
            10.5,
            600.0,
            2500.0,
            200.0,
            600.0,
            10.0,
            120.0,
            true,
            1.0,
            1,
            0.0,
            3.250
        },
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
                -1.125,
                1.572,
                false,
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
            3.25,
            2.0,
            1.572,
            1.572,
            1.572,
            1.3,
            15.5,
            9.0,
            600.0,
            4991.46340024,
            500.0,
            600.0,
            10.0,
            45.0,
            true,
            1.0,
            1,
            0.0,
            3.250
        },
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
                -1.572,
                1.572,
                false,
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
