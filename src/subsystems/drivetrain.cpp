#include "subsystems/drivetrain.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>

namespace subsystems {

static std::int8_t rotationPortWithDirection(std::int8_t port, bool reversed) {
    const std::int8_t absolutePort = port < 0 ? static_cast<std::int8_t>(-port) : port;
    return reversed ? static_cast<std::int8_t>(-absolutePort) : absolutePort;
}

static int joystickToVoltage(int input) {
    const int clipped = std::clamp(input, -127, 127);
    return clipped * 12000 / 127;
}

Drivetrain::Drivetrain(const config::RobotConfig& config, util::Logger& logger)
    : kind(config.drive.kind),
      logger(logger),
      hasLeftMotors(config.ports.motors.leftDrive.size() > 0),
      hasRightMotors(config.ports.motors.rightDrive.size() > 0),
      hasCenterMotors(config.ports.motors.centerDrive.size() > 0),
      leftMotors(config.ports.motors.leftDrive),
      rightMotors(config.ports.motors.rightDrive),
      centerMotors(config.ports.motors.centerDrive),
      imu(config.ports.sensors.imu),
      leftVerticalRotation(rotationPortWithDirection(config.ports.sensors.leftVerticalRotation,
                                                     config.drive.tracking.leftVerticalReversed)),
      rightVerticalRotation(rotationPortWithDirection(config.ports.sensors.rightVerticalRotation,
                                                      config.drive.tracking.rightVerticalReversed)),
      horizontalRotation(rotationPortWithDirection(config.ports.sensors.horizontalRotation,
                                                   config.drive.tracking.horizontalReversed)),
      leftVerticalTrackingWheel(&leftVerticalRotation,
                                config.drive.tracking.wheelDiameter,
                                config.drive.tracking.leftVerticalOffset,
                                config.drive.tracking.gearRatio),
      rightVerticalTrackingWheel(&rightVerticalRotation,
                                 config.drive.tracking.wheelDiameter,
                                 config.drive.tracking.rightVerticalOffset,
                                 config.drive.tracking.gearRatio),
      horizontalTrackingWheel(&horizontalRotation,
                              config.drive.tracking.wheelDiameter,
                              config.drive.tracking.horizontalOffset,
                              config.drive.tracking.gearRatio),
      drivetrain(&leftMotors,
                 &rightMotors,
                 config.drive.trackWidth,
                 config.drive.wheelDiameter,
                 config.drive.drivetrainRpm,
                 config.drive.horizontalDrift),
      odomSensors(&leftVerticalTrackingWheel, &rightVerticalTrackingWheel, &horizontalTrackingWheel, nullptr, &imu),
      lateralController(config.pid.lateral),
      angularController(config.pid.angular),
      throttleCurve(5, 12, 1.132), // AON-style driver curve starting point.
      steerCurve(5, 12, 1.132),    // AON-style driver curve starting point.
      lemlibChassis(drivetrain,
                    lateralController,
                    angularController,
                    odomSensors,
                    &throttleCurve,
                    &steerCurve) {}

void Drivetrain::initialize() {
    logger.info(kind == config::DriveKind::HDrive ? "Calibrating H-drive odom" : "Calibrating LemLib chassis");
    if (kind == config::DriveKind::HDrive) {
        logger.warn("BigRobot driver strafe works; LemLib auton still ignores the center motor");
    }
    lemlibChassis.calibrate();
    lemlibChassis.setPose(0, 0, 0); // Bench default; auton routines can reset pose before running.
}

void Drivetrain::configureForMatch(bool opcontrol, double maxAccel) {
    const pros::motor_brake_mode_e_t brakeMode = opcontrol ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_HOLD;
    configuredMaxAccel = maxAccel;

    if (hasLeftMotors) leftMotors.set_brake_mode_all(brakeMode);
    if (hasRightMotors) rightMotors.set_brake_mode_all(brakeMode);
    if (hasCenterMotors) centerMotors.set_brake_mode_all(brakeMode);

    if (hasLeftMotors) leftMotors.set_gearing_all(pros::E_MOTOR_GEARSET_06);
    if (hasRightMotors) rightMotors.set_gearing_all(pros::E_MOTOR_GEARSET_06);
    if (hasCenterMotors) centerMotors.set_gearing_all(pros::E_MOTOR_GEARSET_06);

    if (hasLeftMotors) leftMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    if (hasRightMotors) rightMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    if (hasCenterMotors) centerMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);

    if (hasLeftMotors) leftMotors.tare_position_all();
    if (hasRightMotors) rightMotors.tare_position_all();
    if (hasCenterMotors) centerMotors.tare_position_all();

    lemlibChassis.setBrakeMode(brakeMode);
    logger.info(opcontrol ? "Drive brake mode: brake" : "Drive brake mode: hold");
}

void Drivetrain::update() {}

void Drivetrain::stop() {
    if (hasLeftMotors) leftMotors.brake();
    if (hasRightMotors) rightMotors.brake();
    if (hasCenterMotors) centerMotors.brake();
}

void Drivetrain::debug() const {
    logger.pose("Drive", const_cast<Drivetrain*>(this)->lemlibChassis.getPose());
}

lemlib::Chassis& Drivetrain::chassis() {
    return lemlibChassis;
}

const lemlib::Chassis& Drivetrain::chassis() const {
    return lemlibChassis;
}

lemlib::Pose Drivetrain::pose() const {
    return const_cast<Drivetrain*>(this)->lemlibChassis.getPose();
}

config::DriveKind Drivetrain::driveKind() const {
    return kind;
}

void Drivetrain::tank(int left, int right) {
    if (kind == config::DriveKind::HDrive) {
        setTankVoltage(joystickToVoltage(left), joystickToVoltage(right));
        setStrafeVoltage(0);
        return;
    }
    lemlibChassis.tank(left, right);
}

void Drivetrain::arcade(int throttle, int turn, float desaturateBias) {
    if (kind == config::DriveKind::HDrive) {
        hDrive(throttle, turn, 0);
        return;
    }
    lemlibChassis.arcade(throttle, turn, false, desaturateBias);
}

void Drivetrain::curvature(int throttle, int turn) {
    if (kind == config::DriveKind::HDrive) {
        hDrive(throttle, turn, 0);
        return;
    }
    lemlibChassis.curvature(throttle, turn);
}

void Drivetrain::hDrive(int throttle, int turn, int strafe) {
    const int leftVoltage = joystickToVoltage(throttle + turn);
    const int rightVoltage = joystickToVoltage(throttle - turn);
    const int strafeVoltage = joystickToVoltage(strafe);

    setTankVoltage(leftVoltage, rightVoltage);
    setStrafeVoltage(strafeVoltage);
}

void Drivetrain::moveToPoint(const std::string& name,
                             double x,
                             double y,
                             int timeout,
                             lemlib::MoveToPointParams params,
                             bool async) {
    logger.autonStep(name.c_str());
    if (kind == config::DriveKind::HDrive) {
        logger.warn("H-drive moveToPoint uses LemLib left/right only; sideways motor is not part of this motion");
    }
    const std::uint32_t startMs = pros::millis();
    // moveToPoint is usually faster when final heading does not matter. earlyExitRange is in inches for chaining.
    lemlibChassis.moveToPoint(x, y, timeout, params, async);
    if (!async) {
        const lemlib::Pose finalPose = lemlibChassis.getPose();
        const double positionError = std::hypot(x - finalPose.x, y - finalPose.y);
        logger.pose("After moveToPoint", finalPose);
        logger.motionSample("moveToPoint", positionError, 0.0, 0.0, pros::millis() - startMs);
    }
}

void Drivetrain::moveToPose(const std::string& name,
                            double x,
                            double y,
                            double theta,
                            int timeout,
                            lemlib::MoveToPoseParams params,
                            bool async) {
    logger.autonStep(name.c_str());
    if (kind == config::DriveKind::HDrive) {
        logger.warn("H-drive moveToPose uses LemLib left/right only; strafe is driver-control only for now");
    }
    const std::uint32_t startMs = pros::millis();
    // moveToPose uses LemLib's boomerang-style controller. lead changes how wide the turn is.
    // horizontalDrift affects motion while turning; retune it after wheel changes.
    lemlibChassis.moveToPose(x, y, theta, timeout, params, async);
    if (!async) {
        const lemlib::Pose finalPose = lemlibChassis.getPose();
        const double positionError = std::hypot(x - finalPose.x, y - finalPose.y);
        logger.pose("After moveToPose", finalPose);
        logger.motionSample("moveToPose", positionError, theta - finalPose.theta, 0.0, pros::millis() - startMs);
    }
}

void Drivetrain::turnToHeading(const std::string& name,
                               double heading,
                               int timeout,
                               lemlib::TurnToHeadingParams params,
                               bool async) {
    logger.autonStep(name.c_str());
    const std::uint32_t startMs = pros::millis();
    // turnToHeading uses an absolute field heading. Timeout is a hard limit.
    lemlibChassis.turnToHeading(heading, timeout, params, async);
    if (!async) {
        const lemlib::Pose finalPose = lemlibChassis.getPose();
        logger.pose("After turnToHeading", finalPose);
        logger.motionSample("turnToHeading", 0.0, heading - finalPose.theta, 0.0, pros::millis() - startMs);
    }
}

void Drivetrain::turnToPoint(const std::string& name,
                             double x,
                             double y,
                             int timeout,
                             lemlib::TurnToPointParams params,
                             bool async) {
    logger.autonStep(name.c_str());
    // turnToPoint faces a Cartesian point; avoid points within about 10 inches unless the geometry is intentional.
    lemlibChassis.turnToPoint(x, y, timeout, params, async);
    if (!async) logger.pose("After turnToPoint", lemlibChassis.getPose());
}

void Drivetrain::swingToHeading(const std::string& name,
                                double heading,
                                lemlib::DriveSide lockedSide,
                                int timeout,
                                lemlib::SwingToHeadingParams params,
                                bool async) {
    logger.autonStep(name.c_str());
    // Swing turns lock one side of the drivetrain, so they need extra space and driver awareness.
    lemlibChassis.swingToHeading(heading, lockedSide, timeout, params, async);
    if (!async) logger.pose("After swingToHeading", lemlibChassis.getPose());
}

void Drivetrain::swingToPoint(const std::string& name,
                              double x,
                              double y,
                              lemlib::DriveSide lockedSide,
                              int timeout,
                              lemlib::SwingToPointParams params,
                              bool async) {
    logger.autonStep(name.c_str());
    lemlibChassis.swingToPoint(x, y, lockedSide, timeout, params, async);
    if (!async) logger.pose("After swingToPoint", lemlibChassis.getPose());
}

void Drivetrain::followPath(const std::string& name,
                            const asset& path,
                            double lookahead,
                            int timeout,
                            bool forwards,
                            bool async) {
    logger.autonStep(name.c_str());
    if (kind == config::DriveKind::HDrive) {
        logger.warn("H-drive followPath uses LemLib left/right only; center strafe motor is ignored");
    }
    lemlibChassis.follow(path, lookahead, timeout, forwards, async);
    if (!async) logger.pose("After follow", lemlibChassis.getPose());
}

void Drivetrain::cancelMotion() {
    lemlibChassis.cancelMotion();
}

void Drivetrain::cancelAllMotions() {
    lemlibChassis.cancelMotion();
}

bool Drivetrain::isInMotion() const {
    return lemlibChassis.isInMotion();
}

bool Drivetrain::hasStrafeMotor() const {
    return hasCenterMotors;
}

double Drivetrain::verticalTrackingInches() {
    return leftVerticalTrackingWheel.getDistanceTraveled();
}

double Drivetrain::horizontalTrackingInches() {
    return horizontalTrackingWheel.getDistanceTraveled();
}

bool Drivetrain::imuReady() const {
    return !imu.is_calibrating();
}

double Drivetrain::hottestDriveMotorCelsius() const {
    double hottest = 0.0;
    if (hasLeftMotors) {
        for (double temperature : leftMotors.get_temperature_all()) {
            if (temperature > hottest) hottest = temperature;
        }
    }
    if (hasRightMotors) {
        for (double temperature : rightMotors.get_temperature_all()) {
            if (temperature > hottest) hottest = temperature;
        }
    }
    if (hasCenterMotors) {
        for (double temperature : centerMotors.get_temperature_all()) {
            if (temperature > hottest) hottest = temperature;
        }
    }
    return hottest;
}

void Drivetrain::setTankVoltage(int left, int right) {
    if (hasLeftMotors) leftMotors.move_voltage(left);
    if (hasRightMotors) rightMotors.move_voltage(right);
}

void Drivetrain::setStrafeVoltage(int strafe) {
    if (hasCenterMotors) centerMotors.move_voltage(strafe);
}

} // namespace subsystems
