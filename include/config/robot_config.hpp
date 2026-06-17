#pragma once

// Robot configuration describes measured hardware, not behavior.
// Replace TODO-marked measurements here before real robot testing.

#include "config/ports.hpp"
#include "config/robot_identity.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"

namespace config {

// Tracking wheel geometry. Direction tests come before PID tuning.
struct TrackingConfig {
    double wheelDiameter;
    double gearRatio;
    double leftVerticalOffset;
    double rightVerticalOffset;
    double horizontalOffset;
    bool leftVerticalReversed;
    bool rightVerticalReversed;
    bool horizontalReversed;
};

// Drivetrain measurements passed into LemLib.
struct DriveConfig {
    pros::motor_gearset_e_t gearset;
    double trackWidth;
    double wheelDiameter;
    double drivetrainRpm;
    double horizontalDrift;
    TrackingConfig tracking;
};

// AON constants we keep around even when LemLib does not use them directly.
struct AonRobotConstants {
    double driveWheelDiameter;
    double trackingWheelDiameter;
    double distanceLeftTrackingWheelCenter;
    double distanceRightTrackingWheelCenter;
    double distanceBackTrackingWheelCenter;
    double motorToDriveRatio;
    double driveWidth;
    double driveLength;
    double maxRpm;
    double maxAccel;
    double maxDecel;
    double intakeVelocity;
    double sensitivity;
    double intakeActivationDistance;
    bool gyroEnabled;
    double gyroConfidence;
    int gyroFilterLength;
    double encoderConfidence;
    double offsetXEncoderMid;
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
    AonRobotConstants aon;
    DriveConfig drive;
    PidConfig pid;
};

const RobotConfig& smallRobotConfig();
const RobotConfig& bigRobotConfig();
const RobotConfig& activeRobotConfig();

} // namespace config
