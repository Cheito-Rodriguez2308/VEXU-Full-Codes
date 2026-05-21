#pragma once

// Robot configuration describes measured hardware, not behavior.
// Students should replace TODO_PLACEHOLDER values here before real testing.

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
    // Lateral PID controls field-space translation using LemLib odometry.
    // Tune after wheel direction, track width, and horizontal drift are believable.
    lemlib::ControllerSettings lateral;
    // Angular PID controls robot heading using the IMU through LemLib.
    // Tune after the IMU calibrates reliably and the drivetrain turns symmetrically.
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
