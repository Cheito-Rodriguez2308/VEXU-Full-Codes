#pragma once

// Drivetrain owns the LemLib chassis and exposes team-readable motion wrappers.
// Other modules should call these methods instead of touching LemLib directly.

#include "config/robot_config.hpp"
#include "lemlib/api.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "util/logger.hpp"
#include <string>

namespace subsystems {

class Drivetrain {
  public:
    Drivetrain(const config::RobotConfig& config, util::Logger& logger);

    void initialize();
    void configureForMatch(bool opcontrol, double maxAccel);
    void update();
    void stop();
    void debug() const;

    lemlib::Chassis& chassis();
    const lemlib::Chassis& chassis() const;
    lemlib::Pose pose() const;
    config::DriveKind driveKind() const;

    void tank(int left, int right);
    void arcade(int throttle, int turn, float desaturateBias);
    void curvature(int throttle, int turn);
    void hDrive(int throttle, int turn, int strafe);

    void moveToPoint(const std::string& name,
                     double x,
                     double y,
                     int timeout,
                     lemlib::MoveToPointParams params = {},
                     bool async = false);
    void moveToPose(const std::string& name,
                    double x,
                    double y,
                    double theta,
                    int timeout,
                    lemlib::MoveToPoseParams params = {},
                    bool async = false);
    void turnToHeading(const std::string& name,
                       double heading,
                       int timeout,
                       lemlib::TurnToHeadingParams params = {},
                       bool async = false);
    void turnToPoint(const std::string& name,
                     double x,
                     double y,
                     int timeout,
                     lemlib::TurnToPointParams params = {},
                     bool async = false);
    void swingToHeading(const std::string& name,
                        double heading,
                        lemlib::DriveSide lockedSide,
                        int timeout,
                        lemlib::SwingToHeadingParams params = {},
                        bool async = false);
    void swingToPoint(const std::string& name,
                      double x,
                      double y,
                      lemlib::DriveSide lockedSide,
                      int timeout,
                      lemlib::SwingToPointParams params = {},
                      bool async = false);
    void followPath(const std::string& name,
                    const asset& path,
                    double lookahead,
                    int timeout,
                    bool forwards,
                    bool async = false);

    void cancelMotion();
    void cancelAllMotions();
    bool isInMotion() const;
    bool hasStrafeMotor() const;

    double verticalTrackingInches();
    double horizontalTrackingInches();
    bool imuReady() const;
    double hottestDriveMotorCelsius() const;

  private:
    void setTankVoltage(int left, int right);
    void setStrafeVoltage(int strafe);

    config::DriveKind kind;
    double configuredMaxAccel = 0.0;
    util::Logger& logger;
    bool hasLeftMotors;
    bool hasRightMotors;
    bool hasCenterMotors;
    pros::MotorGroup leftMotors;
    pros::MotorGroup rightMotors;
    pros::MotorGroup centerMotors;
    pros::Imu imu;
    pros::Rotation leftVerticalRotation;
    pros::Rotation rightVerticalRotation;
    pros::Rotation horizontalRotation;
    lemlib::TrackingWheel leftVerticalTrackingWheel;
    lemlib::TrackingWheel rightVerticalTrackingWheel;
    lemlib::TrackingWheel horizontalTrackingWheel;
    lemlib::Drivetrain drivetrain;
    lemlib::OdomSensors odomSensors;
    lemlib::ControllerSettings lateralController;
    lemlib::ControllerSettings angularController;
    lemlib::ExpoDriveCurve throttleCurve;
    lemlib::ExpoDriveCurve steerCurve;
    lemlib::Chassis lemlibChassis;
};

} // namespace subsystems
