#include "auton/auton_routines.hpp"
#include "auton/auton_actions.hpp"
#include "auton/paths.hpp"
#include "config/constants.hpp"
#include "config/robot_identity.hpp"
#include "pros/rtos.hpp"

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

void pathJerryIoTest(AutonActions& actions) {
    // Quick path.jerryio check. First line of static/paths/path.jerryio.txt is 0, 0, 20.
    // Keep this as a test routine until the path is retuned on the current robot.
    actions.setPose("path start pose", 0.0, 0.0, 20.0);
    actions.followPath("path.jerryio test", paths_path_jerryio_txt, config::tuning::defaultLookahead, 8000, true);
}

static void aonSmallMatchLoader(AutonActions& actions) {
    // AON small bot routine style: line up with the loader, use scan, then score.
    actions.driveToPoint("small loader align", 31.0, 0.0, 2500);
    actions.turnToHeading("small face loader", 86.0, 1500);
    actions.toggleCart();
    actions.startIntakeScan();
    actions.driveToPoint("small touch loader", 35.0, 0.0, 1200);
    pros::delay(3000);
    actions.stopIntakeScan();
    actions.driveToPoint("small back to goal", 18.0, 0.0, 1800);
    actions.turnToHeading("small face goal", 170.0, 1800);
    actions.scoreIntake(subsystems::IntakeScoreHeight::Top, 2500);
}

static void aonSmallPark(AutonActions& actions) {
    actions.driveToPoint("small park approach", -5.0, 0.0, 1200);
    actions.hDriveFor("small push park", 90, 0, 0, 800);
    actions.scoreCup();
}

static void aonBigMatchLoader(AutonActions& actions) {
    // AON big bot routine style: strafe to loader, drop cart, load, score long goal.
    actions.startIntakeScan();
    actions.hDriveFor("big strafe to loader", 0, 0, 80, 900);
    actions.toggleCart();
    actions.driveToPoint("big loader bump", 6.0, 0.0, 1200);
    actions.hDriveFor("big push loader", 70, 0, 0, 300);
    pros::delay(5000);
    actions.driveToPoint("big back from loader", -18.0, 0.0, 1800);
    actions.toggleCart();
    actions.setScoreHeight(subsystems::IntakeScoreHeight::Top);
    actions.startSortRelease();
    pros::delay(3000);
    actions.stopSortRelease();
    actions.stopIntakeScan();
}

static void aonBigPark(AutonActions& actions) {
    actions.hDriveFor("big park push", 110, 0, 0, 1200);
    actions.scoreCup();
}

void aonRedRoutine1(AutonActions& actions, config::RobotIdentity identity) {
    if (identity == config::RobotIdentity::BigRobot) {
        aonBigMatchLoader(actions);
    } else {
        aonSmallMatchLoader(actions);
    }
}

void aonRedRoutine2(AutonActions& actions, config::RobotIdentity identity) {
    if (identity == config::RobotIdentity::BigRobot) {
        aonBigPark(actions);
    } else {
        aonSmallPark(actions);
    }
}

void aonBlueRoutine1(AutonActions& actions, config::RobotIdentity identity) {
    aonRedRoutine1(actions, identity);
}

void aonBlueRoutine2(AutonActions& actions, config::RobotIdentity identity) {
    aonRedRoutine2(actions, identity);
}

void aonSkillsRoutine1(AutonActions& actions, config::RobotIdentity identity) {
    aonRedRoutine1(actions, identity);
    actions.turnToHeading("skills reset heading", 180.0, 1800);
    aonRedRoutine2(actions, identity);
}

void doNothing(AutonActions&) {
    // Safe option for field setup and quick checks.
}

} // namespace auton
