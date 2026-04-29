#pragma once

#include "auton/auton_actions.hpp"
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
    DoNothing
};

const char* toString(AutonRoutine routine);

class AutonSelector {
  public:
    AutonSelector(AutonActions& actions, pros::Controller& controller, util::Logger& logger);

    void initialize();
    void update();
    void runSelected();
    void setSelected(AutonRoutine routine);
    AutonRoutine selected() const;

  private:
    AutonActions& actions;
    pros::Controller& controller;
    util::Logger& logger;
    AutonRoutine selectedRoutine = AutonRoutine::DoNothing;
};

} // namespace auton
