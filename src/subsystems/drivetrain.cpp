#include "subsystems/drivetrain.hpp"
#include "pros/rtos.hpp"
#include <cmath>

namespace subsystems {

static std::int8_t rotationPortWithDirection(std::int8_t port, bool reversed) {
    const std::int8_t absolutePort = port < 0 ? static_cast<std::int8_t>(-port) : port;
    return reversed ? static_cast<std::int8_t>(-absolutePort) : absolutePort;
}

Drivetrain::Drivetrain(const config::RobotConfig& config, util::Logger& logger)
    : logger(logger),
      leftMotors(config.ports.motors.leftDrive),
      rightMotors(config.ports.motors.rightDrive),
      imu(config.ports.sensors.imu),
      verticalRotation(rotationPortWithDirection(config.ports.sensors.verticalRotation, config.drive.tracking.verticalReversed)),
      horizontalRotation(rotationPortWithDirection(config.ports.sensors.horizontalRotation, config.drive.tracking.horizontalReversed)),
      verticalTrackingWheel(&verticalRotation, config.drive.tracking.wheelDiameter, config.drive.tracking.verticalOffset, config.drive.tracking.gearRatio),
      horizontalTrackingWheel(&horizontalRotation, config.drive.tracking.wheelDiameter, config.drive.tracking.horizontalOffset, config.drive.tracking.gearRatio),
      drivetrain(&leftMotors, &rightMotors, config.drive.trackWidth, config.drive.wheelDiameter, config.drive.drivetrainRpm, config.drive.horizontalDrift),
      odomSensors(&verticalTrackingWheel, nullptr, &horizontalTrackingWheel, nullptr, &imu),
      lateralController(config.pid.lateral),
      angularController(config.pid.angular),
      throttleCurve(5, 12, 1.132), // TODO_PLACEHOLDER driver curve values.
      steerCurve(5, 12, 1.132),    // TODO_PLACEHOLDER driver curve values.
      lemlibChassis(drivetrain, lateralController, angularController, odomSensors, &throttleCurve, &steerCurve) {}

void Drivetrain::initialize() {
    logger.info("Calibrating LemLib chassis with TODO_PLACEHOLDER geometry");
    lemlibChassis.calibrate();
    lemlibChassis.setPose(0, 0, 0); // TODO_PLACEHOLDER starting pose.
}

void Drivetrain::update() {}
void Drivetrain::stop() { leftMotors.brake(); rightMotors.brake(); }
void Drivetrain::debug() const { logger.pose("Drive", const_cast<Drivetrain*>(this)->lemlibChassis.getPose()); }
lemlib::Chassis& Drivetrain::chassis() { return lemlibChassis; }
const lemlib::Chassis& Drivetrain::chassis() const { return lemlibChassis; }
lemlib::Pose Drivetrain::pose() const { return const_cast<Drivetrain*>(this)->lemlibChassis.getPose(); }

void Drivetrain::tank(int left, int right) { lemlibChassis.tank(left, right); }
void Drivetrain::arcade(int throttle, int turn, float desaturateBias) { lemlibChassis.arcade(throttle, turn, false, desaturateBias); }
void Drivetrain::curvature(int throttle, int turn) { lemlibChassis.curvature(throttle, turn); }

void Drivetrain::moveToPoint(const std::string& name, double x, double y, int timeout, lemlib::MoveToPointParams params, bool async) {
    logger.autonStep(name.c_str());
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

void Drivetrain::moveToPose(const std::string& name, double x, double y, double theta, int timeout, lemlib::MoveToPoseParams params, bool async) {
    logger.autonStep(name.c_str());
    const std::uint32_t startMs = pros::millis();
    // moveToPose uses LemLib's boomerang-style controller. lead changes how wide the turn is.
    // horizontalDrift affects motion while turning; TODO_PLACEHOLDER values must be tuned.
    lemlibChassis.moveToPose(x, y, theta, timeout, params, async);
    if (!async) {
        const lemlib::Pose finalPose = lemlibChassis.getPose();
        const double positionError = std::hypot(x - finalPose.x, y - finalPose.y);
        logger.pose("After moveToPose", finalPose);
        logger.motionSample("moveToPose", positionError, theta - finalPose.theta, 0.0, pros::millis() - startMs);
    }
}

void Drivetrain::turnToHeading(const std::string& name, double heading, int timeout, lemlib::TurnToHeadingParams params, bool async) {
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

void Drivetrain::turnToPoint(const std::string& name, double x, double y, int timeout, lemlib::TurnToPointParams params, bool async) {
    logger.autonStep(name.c_str());
    // turnToPoint faces a Cartesian point; avoid points within about 10 inches unless the geometry is intentional.
    lemlibChassis.turnToPoint(x, y, timeout, params, async);
    if (!async) logger.pose("After turnToPoint", lemlibChassis.getPose());
}

void Drivetrain::swingToHeading(const std::string& name, double heading, lemlib::DriveSide lockedSide, int timeout, lemlib::SwingToHeadingParams params, bool async) {
    logger.autonStep(name.c_str());
    // Swing turns lock one side of the drivetrain, so they need extra space and driver awareness.
    lemlibChassis.swingToHeading(heading, lockedSide, timeout, params, async);
    if (!async) logger.pose("After swingToHeading", lemlibChassis.getPose());
}

void Drivetrain::swingToPoint(const std::string& name, double x, double y, lemlib::DriveSide lockedSide, int timeout, lemlib::SwingToPointParams params, bool async) {
    logger.autonStep(name.c_str());
    lemlibChassis.swingToPoint(x, y, lockedSide, timeout, params, async);
    if (!async) logger.pose("After swingToPoint", lemlibChassis.getPose());
}

void Drivetrain::followPath(const std::string& name, const asset& path, double lookahead, int timeout, bool forwards, bool async) {
    logger.autonStep(name.c_str());
    lemlibChassis.follow(path, lookahead, timeout, forwards, async);
    if (!async) logger.pose("After follow", lemlibChassis.getPose());
}

void Drivetrain::cancelMotion() { lemlibChassis.cancelMotion(); }
void Drivetrain::cancelAllMotions() { lemlibChassis.cancelMotion(); }
bool Drivetrain::isInMotion() const { return lemlibChassis.isInMotion(); }
double Drivetrain::verticalTrackingInches() { return verticalTrackingWheel.getDistanceTraveled(); }
double Drivetrain::horizontalTrackingInches() { return horizontalTrackingWheel.getDistanceTraveled(); }
bool Drivetrain::imuReady() const { return !imu.is_calibrating(); }

double Drivetrain::hottestDriveMotorCelsius() const {
    double hottest = 0.0;
    for (double temperature : leftMotors.get_temperature_all()) {
        if (temperature > hottest) hottest = temperature;
    }
    for (double temperature : rightMotors.get_temperature_all()) {
        if (temperature > hottest) hottest = temperature;
    }
    return hottest;
}

} // namespace subsystems
