#include "auton/auton_routines.hpp"
#include "auton/auton_actions.hpp"
#include "auton/paths.hpp"
#include "config/constants.hpp"

namespace auton {

void redSafeAWP(AutonActions& actions) {
    // Safer qual idea: leave line, grab one object, score without crossing traffic.
    // Still depends on preload not sliding during the first move.
    actions.driveToPoint("red safe leave line", 12.0, 6.0, 1500);
    actions.intakeUntilDetected();
    actions.driveToPose("red safe score", 30.0, 18.0, 45.0, 2500);
    actions.scoreCup();
}

void blueSafeAWP(AutonActions& actions) {
    // Mirrored version. Check this separately; blue-side turns usually expose
    // tracking wheel sign mistakes faster than red.
    actions.driveToPoint("blue safe leave line", 12.0, -6.0, 1500);
    actions.followPath("blue safe path", paths_safe_awp_blue_txt, config::tuning::defaultLookahead, 3000, true);
    actions.scorePin();
}

void redGoalStackPriority(AutonActions& actions) {
    // Faster idea for when the goal-side stack is worth the risk.
    // Timing breaks if the cup mechanism hesitates.
    actions.driveToPose("red goal approach", 20.0, 20.0, 0.0, 2500, {.lead = 0.4});
    actions.approachAndScore();
}

void blueGoalStackPriority(AutonActions& actions) {
    // Keep separate from red until both robots have measured starts.
    actions.driveToPose("blue goal approach", 20.0, -20.0, 0.0, 2500, {.lead = 0.4});
    actions.approachAndScore();
}

void redMidfieldPressure(AutonActions& actions) {
    // More aggressive. This should stay out of finals code until midfield traffic is mapped.
    actions.turnToPoint("face midfield", 48.0, 0.0, 1000);
    actions.chainAroundGoal();
}

void blueMidfieldPressure(AutonActions& actions) {
    // Probably needs a lower angular D if it snaps past the lane.
    actions.turnToHeading("blue midfield heading", -45.0, 1000);
    actions.chainAroundGoal();
}

void skillsRoute(AutonActions& actions) {
    // Split skills into chunks so a bad first path does not hide later problems.
    actions.followPath("skills part 1", paths_skills_part_1_txt, config::tuning::defaultLookahead, 5000, true);
    actions.scoreCup();
    actions.followPath("skills part 2", paths_skills_part_2_txt, config::tuning::defaultLookahead, 5000, true);
}

void doNothing(AutonActions&) {
    // Safe option for field setup and quick checks.
}

} // namespace auton
