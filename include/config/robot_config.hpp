#pragma once

#include "config/ports.hpp"
#include "config/robot_identity.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"

namespace config {

struct TrackingConfig {
    double wheelDiameter;
    double gearRatio;
    double verticalOffset;
    double horizontalOffset;
    bool verticalReversed;
    bool horizontalReversed;
};

struct DriveConfig {
    pros::motor_gearset_e_t gearset;
    double trackWidth;
    double wheelDiameter;
    double drivetrainRpm;
    double horizontalDrift;
    TrackingConfig tracking;
};

struct PidConfig {
    lemlib::ControllerSettings lateral;
    lemlib::ControllerSettings angular;
};

struct RobotConfig {
    RobotIdentity identity;
    Ports ports;
    DriveConfig drive;
    PidConfig pid;
};

const RobotConfig& robotAConfig();
const RobotConfig& robotBConfig();
const RobotConfig& activeRobotConfig();

} // namespace config
