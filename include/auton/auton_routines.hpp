#pragma once

namespace auton {

class AutonActions;

void redSafeAWP(AutonActions& actions);
void blueSafeAWP(AutonActions& actions);
void redGoalStackPriority(AutonActions& actions);
void blueGoalStackPriority(AutonActions& actions);
void redMidfieldPressure(AutonActions& actions);
void blueMidfieldPressure(AutonActions& actions);
void skillsRoute(AutonActions& actions);
void doNothing(AutonActions& actions);

} // namespace auton
