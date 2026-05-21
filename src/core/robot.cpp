#include "core/robot.hpp"
#include "config/constants.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

namespace core {

// Constructor order matches the declaration order in robot.hpp.
// Dependencies are passed by reference so ownership remains in Robot.
Robot::Robot()
    : logger("VEXU"),
      controller(pros::E_CONTROLLER_MASTER),
      config(config::activeRobotConfig()),
      drivetrain(config, logger),
      intake(config.ports.motors.intake, logger),
      pin(config.ports.motors.pin, config.ports.adi.pinPiston, logger),
      cup(config.ports.motors.cup, config.ports.adi.cupPiston, logger),
      toggle(config.ports.motors.toggle, logger),
      sensors(config.ports.sensors.optical, config.ports.sensors.distance, logger),
      driverControl(controller, drivetrain, intake, pin, cup, toggle, sensors),
      autonActions(drivetrain, intake, pin, cup, toggle, sensors, logger),
      autonSelector(autonActions, controller, logger),
      driveTuning(drivetrain, logger),
      pidTuning(logger),
      sensorTuning(drivetrain, sensors, logger),
      tuningMenu(controller, driveTuning, pidTuning, sensorTuning, logger) {}

void Robot::initialize() {
    matchState.setMode(MatchMode::Initialize);
    pros::lcd::initialize();
    logger.info(std::string("Initializing ") + config::toString(config.identity));
    drivetrain.initialize();
    intake.initialize();
    pin.initialize();
    cup.initialize();
    toggle.initialize();
    sensors.initialize();
    driverControl.initialize();
    autonSelector.initialize();
    tuningMenu.initialize();
}

void Robot::disabled() {
    matchState.setMode(MatchMode::Disabled);
    drivetrain.stop();
    intake.stop();
    pin.stop();
    cup.stop();
    toggle.stop();
}

void Robot::competitionInitialize() {
    matchState.setMode(MatchMode::CompetitionInitialize);
    while (pros::competition::is_disabled()) {
        autonSelector.update();
        tuningMenu.update();
        updateDashboard();
        pros::delay(config::timing::dashboardLoopMs);
    }
}

void Robot::autonomous() {
    matchState.setMode(MatchMode::Autonomous);
    logger.info("VEX U autonomous scaffold: 30 seconds available");
    autonSelector.runSelected();
}

void Robot::opcontrol() {
    logger.info("VEX U driver control scaffold: 90 seconds available");

    while (true) {
        matchState.setMode(tuningMenu.enabled() ? MatchMode::Tuning : MatchMode::DriverControl);
        sensors.update();
        driverControl.update();
        intake.update(sensors.possession().hasPin, sensors.possession().hasCup, sensors.possession().manualOverride);
        pin.update();
        cup.update();
        toggle.update();
        tuningMenu.update();
        updateDashboard();
        pros::delay(config::timing::driverLoopMs);
    }
}

void Robot::updateDashboard() {
    const lemlib::Pose pose = drivetrain.pose();
    const double batteryVolts = pros::battery::get_voltage() / 1000.0;
    const double hottestDriveMotor = drivetrain.hottestDriveMotorCelsius();

    pros::lcd::print(0, "%s | %s", config::toString(config.identity), toString(matchState.mode()));
    pros::lcd::print(1, "Auton: %s", auton::toString(autonSelector.selected()));
    pros::lcd::print(2, "Pose x %.1f y %.1f h %.1f", pose.x, pose.y, pose.theta);
    pros::lcd::print(3, "IMU %s | Batt %.1fV", drivetrain.imuReady() ? "ready" : "cal", batteryVolts);
    pros::lcd::print(4, "Track V %.1f H %.1f M %.0fC", drivetrain.verticalTrackingInches(), drivetrain.horizontalTrackingInches(), hottestDriveMotor);
    pros::lcd::print(5, "Pin %s Cup %s", sensors.possession().hasPin ? "yes" : "no", sensors.possession().hasCup ? "yes" : "no");
    pros::lcd::print(6, "I:%s P:%s C:%s T:%s", subsystems::toString(intake.getState()), subsystems::toString(pin.getState()),
                     subsystems::toString(cup.getState()), subsystems::toString(toggle.getState()));

    if (!drivetrain.imuReady()) controller.set_text(0, 0, "IMU calibrating");
    if (batteryVolts < config::safety::lowBatteryVolts) controller.set_text(0, 0, "Battery low");
    if (hottestDriveMotor > config::safety::hotMotorCelsius) controller.set_text(0, 0, "Drive motor hot");
}

} // namespace core
