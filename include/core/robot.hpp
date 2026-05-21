#pragma once

// Robot owns the subsystems and match callbacks.
// Keeping one owner makes match debugging less confusing.

#include "auton/auton_actions.hpp"
#include "auton/auton_selector.hpp"
#include "config/robot_config.hpp"
#include "control/driver_control.hpp"
#include "core/match_state.hpp"
#include "subsystems/cup_mechanism.hpp"
#include "subsystems/drivetrain.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/pin_mechanism.hpp"
#include "subsystems/sensors.hpp"
#include "subsystems/toggle_mechanism.hpp"
#include "tuning/drive_tuning.hpp"
#include "tuning/pid_tuning.hpp"
#include "tuning/sensor_tuning.hpp"
#include "tuning/tuning_menu.hpp"
#include "util/logger.hpp"
#include "pros/misc.hpp"

namespace core {

class Robot {
  public:
    Robot();

    void initialize();
    void disabled();
    void competitionInitialize();
    void autonomous();
    void opcontrol();

  private:
    void updateDashboard();

    util::Logger logger;
    pros::Controller controller;
    const config::RobotConfig& config;
    MatchState matchState;
    subsystems::Drivetrain drivetrain;
    subsystems::Intake intake;
    subsystems::PinMechanism pin;
    subsystems::CupMechanism cup;
    subsystems::ToggleMechanism toggle;
    subsystems::Sensors sensors;
    control::DriverControl driverControl;
    auton::AutonActions autonActions;
    auton::AutonSelector autonSelector;
    tuning::DriveTuning driveTuning;
    tuning::PidTuning pidTuning;
    tuning::SensorTuning sensorTuning;
    tuning::TuningMenu tuningMenu;
};

} // namespace core
