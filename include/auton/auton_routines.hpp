#pragma once

namespace auton {

class AutonActions;

} // namespace auton

namespace config {
enum class RobotIdentity;
}

namespace auton {

void redSafeAWP(AutonActions& actions);
void blueSafeAWP(AutonActions& actions);
void redGoalStackPriority(AutonActions& actions);
void blueGoalStackPriority(AutonActions& actions);
void redMidfieldPressure(AutonActions& actions);
void blueMidfieldPressure(AutonActions& actions);
void skillsRoute(AutonActions& actions);
void aonRedRoutine1(AutonActions& actions, config::RobotIdentity identity);
void aonRedRoutine2(AutonActions& actions, config::RobotIdentity identity);
void aonBlueRoutine1(AutonActions& actions, config::RobotIdentity identity);
void aonBlueRoutine2(AutonActions& actions, config::RobotIdentity identity);
void aonSkillsRoutine1(AutonActions& actions, config::RobotIdentity identity);
void doNothing(AutonActions& actions);

} // namespace auton
