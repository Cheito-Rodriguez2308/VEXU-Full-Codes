#include "auton/auton_routines.hpp"
#include "auton/auton_actions.hpp"
#include "auton/paths.hpp"
#include "config/constants.hpp"

namespace auton {

void redSafeAWP(AutonActions& actions) {
    // TODO: replace these poses after odometry is trusted on the real field.
    actions.driveToPoint("red safe leave line", 12.0, 6.0, 1500);
    actions.intakeUntilDetected();
    actions.driveToPose("red safe score", 30.0, 18.0, 45.0, 2500);
    actions.scoreCup();
}

void blueSafeAWP(AutonActions& actions) {
    // TODO: mirror and retest once Robot A/B starting poses are measured.
    actions.driveToPoint("blue safe leave line", 12.0, -6.0, 1500);
    actions.followPath("blue safe path", paths_safe_awp_blue_txt, config::tuning::defaultLookahead, 3000, true);
    actions.scorePin();
}

void redGoalStackPriority(AutonActions& actions) {
    // TODO: tune lead and timeout after the scoring mechanism is consistent.
    actions.driveToPose("red goal approach", 20.0, 20.0, 0.0, 2500, {.lead = 0.4});
    actions.approachAndScore();
}

void blueGoalStackPriority(AutonActions& actions) {
    // TODO: tune lead and timeout after the scoring mechanism is consistent.
    actions.driveToPose("blue goal approach", 20.0, -20.0, 0.0, 2500, {.lead = 0.4});
    actions.approachAndScore();
}

void redMidfieldPressure(AutonActions& actions) {
    // TODO: choose a real midfield target after field notes are done.
    actions.turnToPoint("face midfield", 48.0, 0.0, 1000);
    actions.chainAroundGoal();
}

void blueMidfieldPressure(AutonActions& actions) {
    // TODO: verify this heading against the blue starting tile.
    actions.turnToHeading("blue midfield heading", -45.0, 1000);
    actions.chainAroundGoal();
}

void skillsRoute(AutonActions& actions) {
    // TODO: split skills paths around reliable reset points.
    actions.followPath("skills part 1", paths_skills_part_1_txt, config::tuning::defaultLookahead, 5000, true);
    actions.scoreCup();
    actions.followPath("skills part 2", paths_skills_part_2_txt, config::tuning::defaultLookahead, 5000, true);
}

void doNothing(AutonActions&) {
    // Intentional safe option for field setup and first bring-up.
}

} // namespace auton
