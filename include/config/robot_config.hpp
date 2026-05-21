#pragma once

// Robot configuration describes measured hardware, not behavior.
// Replace TODO-marked measurements here before real robot testing.

#include "config/ports.hpp"
#include "config/robot_identity.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"

namespace config {

/**
 * @brief Tracking wheel geometry used by LemLib odometry.
 * Offsets are signed, so direction tests come before PID tuning.
 */
struct TrackingConfig {
    double wheelDiameter;
    double gearRatio;
    double verticalOffset;
    double horizontalOffset;
    bool verticalReversed;
    bool horizontalReversed;
};

/**
 * @brief Drivetrain measurements passed into LemLib.
 * Retune these after changing wheels, cartridges, or drive weight.
 */
struct DriveConfig {
    pros::motor_gearset_e_t gearset;
    double trackWidth;
    double wheelDiameter;
    double drivetrainRpm;
    double horizontalDrift;
    TrackingConfig tracking;
};

/**
 * @brief LemLib motion controllers for lateral and angular movement.
 */
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
