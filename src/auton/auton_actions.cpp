#include "auton/auton_actions.hpp"
#include "config/constants.hpp"
#include "pros/rtos.hpp"
#include "util/timer.hpp"

namespace auton {

AutonActions::AutonActions(subsystems::Drivetrain& drivetrain, subsystems::Intake& intake,
                           subsystems::PinMechanism& pin, subsystems::CupMechanism& cup,
                           subsystems::ToggleMechanism& toggle, subsystems::Sensors& sensors,
                           util::Logger& logger)
    : drivetrain(drivetrain), intake(intake), pin(pin), cup(cup), toggle(toggle), sensors(sensors), logger(logger) {}

void AutonActions::driveToPoint(const char* name, double x, double y, int timeout,
                                lemlib::MoveToPointParams params) {
    drivetrain.moveToPoint(name, x, y, timeout, params);
}

void AutonActions::driveToPose(const char* name, double x, double y, double theta, int timeout,
                               lemlib::MoveToPoseParams params) {
    drivetrain.moveToPose(name, x, y, theta, timeout, params);
}

void AutonActions::turnToHeading(const char* name, double heading, int timeout,
                                 lemlib::TurnToHeadingParams params) {
    drivetrain.turnToHeading(name, heading, timeout, params);
}

void AutonActions::turnToPoint(const char* name, double x, double y, int timeout,
                               lemlib::TurnToPointParams params) {
    drivetrain.turnToPoint(name, x, y, timeout, params);
}

void AutonActions::swingToHeading(const char* name, double heading, lemlib::DriveSide side, int timeout,
                                  lemlib::SwingToHeadingParams params) {
    drivetrain.swingToHeading(name, heading, side, timeout, params);
}

void AutonActions::swingToPoint(const char* name, double x, double y, lemlib::DriveSide side, int timeout,
                                lemlib::SwingToPointParams params) {
    drivetrain.swingToPoint(name, x, y, side, timeout, params);
}

void AutonActions::followPath(const char* name, const asset& path, double lookahead, int timeout, bool forwards) {
    drivetrain.followPath(name, path, lookahead, timeout, forwards);
}

void AutonActions::scorePin() {
    logger.autonStep("scorePin");
    pin.setState(subsystems::PinMechanismState::Score);
    pros::delay(350); // TODO: mechanism timing.
    sensors.setHasPin(false);
}

void AutonActions::scoreCup() {
    logger.autonStep("scoreCup");
    cup.setState(subsystems::CupMechanismState::Stack);
    pros::delay(350); // TODO: mechanism timing.
    sensors.setHasCup(false);
}

void AutonActions::setToggle(subsystems::ToggleMechanismState state) {
    logger.autonStep("setToggle");
    toggle.setState(state);
    pros::delay(250); // TODO: timing.
}

void AutonActions::waitForSensor(const char* name, int timeoutMs) {
    logger.autonStep(name);
    util::Timer timer;
    timer.reset();
    while (!timer.done(timeoutMs)) {
        sensors.update();
        if (sensors.possession().hasPin || sensors.possession().hasCup) return;
        pros::delay(10);
    }
    logger.warn("waitForSensor timed out");
}

void AutonActions::cancelMotionIf(bool condition, const char* reason) {
    if (condition) {
        logger.warn(reason);
        drivetrain.cancelMotion();
    }
}

void AutonActions::chainAroundGoal() {
    // Use this only after single motions are predictable.
    // TODO: replace these coordinates with a tested goal-side lane.
    driveToPoint("chain entry", 18.0, 12.0, 1800, {.minSpeed = 45, .earlyExitRange = 8});
    driveToPose("chain exit", 28.0, 28.0, 90.0, 2200, {.minSpeed = 35, .earlyExitRange = 6});
}

void AutonActions::approachAndScore() {
    // TODO: replace with a scoring pose that leaves room for Robot B.
    driveToPose("approach scoring zone", 36.0, 14.0, 0.0, config::timing::defaultMotionTimeoutMs);
    scoreCup();
}

void AutonActions::intakeUntilDetected() {
    intake.setState(subsystems::IntakeState::IntakeCup);
    // TODO: tune maxSpeed so the intake has time to pull the object in.
    drivetrain.moveToPoint("intake lane", 24.0, 8.0, 2500, {.maxSpeed = 70}, true);
    waitForSensor("wait for possession", 1500);
    cancelMotionIf(sensors.possession().hasCup, "Possession detected; canceling intake motion");
    intake.setState(subsystems::IntakeState::Hold);
}

} // namespace auton
