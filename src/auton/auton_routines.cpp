#include "auton/auton_routines.hpp"
#include "auton/auton_actions.hpp"
#include "auton/paths.hpp"
#include "config/constants.hpp"

namespace auton {

void redSafeAWP(AutonActions& actions) {
    actions.driveToPoint("TODO_PLACEHOLDER red safe AWP leave line", 12.0, 6.0, 1500);
    actions.intakeUntilDetected();
    actions.driveToPose("TODO_PLACEHOLDER red safe AWP score", 30.0, 18.0, 45.0, 2500);
    actions.scoreCup();
}

void blueSafeAWP(AutonActions& actions) {
    actions.driveToPoint("TODO_PLACEHOLDER blue safe AWP leave line", 12.0, -6.0, 1500);
    actions.followPath("TODO_PLACEHOLDER blue safe path", paths_safe_awp_blue_txt, config::tuning::defaultLookahead, 3000, true);
    actions.scorePin();
}

void redGoalStackPriority(AutonActions& actions) {
    actions.driveToPose("TODO_PLACEHOLDER red goal approach", 20.0, 20.0, 0.0, 2500, {.lead = 0.4});
    actions.approachAndScore();
}

void blueGoalStackPriority(AutonActions& actions) {
    actions.driveToPose("TODO_PLACEHOLDER blue goal approach", 20.0, -20.0, 0.0, 2500, {.lead = 0.4});
    actions.approachAndScore();
}

void redMidfieldPressure(AutonActions& actions) {
    actions.turnToPoint("TODO_PLACEHOLDER face midfield", 48.0, 0.0, 1000);
    actions.chainAroundGoal();
}

void blueMidfieldPressure(AutonActions& actions) {
    actions.turnToHeading("TODO_PLACEHOLDER blue midfield heading", -45.0, 1000);
    actions.chainAroundGoal();
}

void skillsRoute(AutonActions& actions) {
    actions.followPath("TODO_PLACEHOLDER skills part 1", paths_skills_part_1_txt, config::tuning::defaultLookahead, 5000, true);
    actions.scoreCup();
    actions.followPath("TODO_PLACEHOLDER skills part 2", paths_skills_part_2_txt, config::tuning::defaultLookahead, 5000, true);
}

void doNothing(AutonActions&) {
    // Intentional safe option for field setup and first bring-up.
}

} // namespace auton
