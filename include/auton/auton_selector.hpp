#pragma once

#include "auton/auton_actions.hpp"
#include "config/robot_config.hpp"
#include "pros/misc.hpp"
#include "util/logger.hpp"

namespace auton {

enum class AutonRoutine {
    RedSafeAWP,
    BlueSafeAWP,
    RedGoalStackPriority,
    BlueGoalStackPriority,
    RedMidfieldPressure,
    BlueMidfieldPressure,
    SkillsRoute,
    AonRed1,
    AonRed2,
    AonBlue1,
    AonBlue2,
    AonSkills1,
    DoNothing
};

const char* toString(AutonRoutine routine);

class AutonSelector {
  public:
    AutonSelector(AutonActions& actions, const config::RobotConfig& config,
                  pros::Controller& controller, util::Logger& logger);

    void initialize();
    void update();
    void runSelected();
    void setSelected(AutonRoutine routine);
    AutonRoutine selected() const;

  private:
    AutonActions& actions;
    const config::RobotConfig& config;
    pros::Controller& controller;
    util::Logger& logger;
    AutonRoutine selectedRoutine = AutonRoutine::DoNothing;
};

} // namespace auton
