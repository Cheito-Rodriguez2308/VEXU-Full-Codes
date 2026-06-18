#include "core/robot.hpp"
#include "config/constants.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

namespace core {

// Constructor order matches robot.hpp.
// Passing references keeps hardware ownership in one place.
Robot::Robot()
    : logger("VEXU"),
      controller(pros::E_CONTROLLER_MASTER),
      config(config::activeRobotConfig()),
      drivetrain(config, logger),
      intake(config, logger),
      pin(config.ports.motors.pin, config.ports.adi.semPiston ? config.ports.adi.semPiston : config.ports.adi.arrowPiston, logger),
      cup(config.ports.motors.cup, config.ports.adi.brooksPiston, logger),
      toggle(config.ports.motors.toggle, logger),
      sensors(config.ports.intake.opticalSensor, config.ports.intake.distanceSensor, logger),
      driverControl(controller, drivetrain, intake, pin, cup, toggle, sensors),
      autonActions(drivetrain, intake, pin, cup, toggle, sensors, logger),
      autonSelector(autonActions, config, controller, logger),
      driveTuning(drivetrain, logger),
      pidTuning(logger),
      sensorTuning(drivetrain, sensors, logger),
      autoTuner(controller, drivetrain, config, logger),
      tuningMenu(controller, driveTuning, pidTuning, sensorTuning, autoTuner, logger) {}

void Robot::initialize() {
    matchState.setMode(MatchMode::Initialize);
    pros::lcd::initialize();
    logger.info(std::string("Initializing ") + config::toString(config.identity));
    drivetrain.initialize();
    intake.initialize();
    configureForMatch(true);
    pin.initialize();
    cup.initialize();
    toggle.initialize();
    sensors.initialize();
    driverControl.setProfile(control::profileForDriver(config.driver));
    driverControl.initialize();
    autonSelector.initialize();
    tuningMenu.initialize();
    startBackgroundTasks();
}

void Robot::disabled() {
    matchState.setMode(MatchMode::Disabled);
    stopRobotOutputs();
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
    logger.info("VEX U autonomous: 30 seconds available");
    configureForMatch(false);
    intake.startScan();
    autonSelector.runSelected();
}

void Robot::opcontrol() {
    logger.info("VEX U driver control: 90 seconds available");
    configureForMatch(true);
    intake.startScan();

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

void Robot::startBackgroundTasks() {
    if (intakeScanTask == nullptr) {
        intakeScanTask = pros::Task::create([this] {
            while (true) {
                intake.scanTaskStep();
                pros::delay(50);
            }
        }, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "intake scan");
    }

    if (intakeSortTask == nullptr) {
        intakeSortTask = pros::Task::create([this] {
            while (true) {
                intake.sortTaskStep();
                pros::delay(10);
            }
        }, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "intake sort");
    }

    if (autonomousSafetyTask == nullptr) {
        autonomousSafetyTask = pros::Task::create([this] {
            while (true) {
                autonomousSafetyStep();
                pros::delay(50);
            }
        }, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "auton safety");
    }

    logger.info("Background tasks started");
}

void Robot::autonomousSafetyStep() {
    if (matchState.mode() != MatchMode::Autonomous) return;
    if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) return;

    stopRobotOutputs();
    logger.warn("Auton safety stop");
}

void Robot::stopRobotOutputs() {
    // AON's old safety also stopped ORBIT. This codebase does not have an
    // orbit subsystem yet, so all owned outputs stop here.
    drivetrain.stop();
    intake.stopScan();
    intake.stopReleasing();
    intake.stop();
    pin.stop();
    cup.stop();
    toggle.stop();
}

void Robot::configureForMatch(bool opcontrol) {
    const double driveMaxAccel =
        config.identity == config::RobotIdentity::BigRobot ? config.aon.maxAccel * 0.4 : config.aon.maxAccel;

    drivetrain.configureForMatch(opcontrol, driveMaxAccel);
    intake.configureForMatch(opcontrol);
}

void Robot::updateDashboard() {
    const lemlib::Pose pose = drivetrain.pose();
    const double batteryVolts = pros::battery::get_voltage() / 1000.0;
    const double hottestDriveMotor = drivetrain.hottestDriveMotorCelsius();

    pros::lcd::print(0, "%s | %s", config::toString(config.identity), toString(matchState.mode()));
    pros::lcd::print(1, "Auton: %s", auton::toString(autonSelector.selected()));
    pros::lcd::print(2, "Pose x %.1f y %.1f h %.1f", pose.x, pose.y, pose.theta);
    pros::lcd::print(3, "IMU %s | Batt %.1fV", drivetrain.imuReady() ? "ready" : "cal", batteryVolts);

    pros::lcd::print(4,
                     "Track V %.1f H %.1f M %.0fC",
                     drivetrain.verticalTrackingInches(),
                     drivetrain.horizontalTrackingInches(),
                     hottestDriveMotor);

    pros::lcd::print(5,
                     "Pin %s Cup %s All %s",
                     sensors.possession().hasPin ? "yes" : "no",
                     sensors.possession().hasCup ? "yes" : "no",
                     subsystems::toString(intake.getAlliance()));
    pros::lcd::print(6,
                     "I:%s P:%s C:%s T:%s",
                     subsystems::toString(intake.getState()),
                     subsystems::toString(pin.getState()),
                     subsystems::toString(cup.getState()), subsystems::toString(toggle.getState()));
    if (config.drive.kind == config::DriveKind::HDrive) {
        pros::lcd::print(7, "H-drive auton: center strafe ignored");
    }

    if (!drivetrain.imuReady()) {
        controller.set_text(0, 0, "IMU calibrating");
    }
    if (batteryVolts < config::safety::lowBatteryVolts) {
        controller.set_text(0, 0, "Battery low");
    }
    if (hottestDriveMotor > config::safety::hotMotorCelsius) {
        controller.set_text(0, 0, "Drive motor hot");
    }
}

} // namespace core
