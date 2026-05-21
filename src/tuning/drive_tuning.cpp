#include "tuning/drive_tuning.hpp"
#include "auton/paths.hpp"
#include "config/constants.hpp"
#include "subsystems/drivetrain.hpp"
#include "util/logger.hpp"

namespace tuning {

DriveTuning::DriveTuning(subsystems::Drivetrain& drivetrain, util::Logger& logger) : drivetrain(drivetrain), logger(logger) {}
void DriveTuning::angular90() { drivetrain.turnToHeading("TODO_PLACEHOLDER angular 90 test", config::tuning::angularQuarterTurnDeg, config::tuning::angularTimeoutMs); }
void DriveTuning::angular180() { drivetrain.turnToHeading("TODO_PLACEHOLDER angular 180 test", config::tuning::angularHalfTurnDeg, config::tuning::angularTimeoutMs); }
void DriveTuning::forward24() { drivetrain.moveToPoint("TODO_PLACEHOLDER forward 24 test", 0, config::tuning::forwardShortInches, config::tuning::lateralTimeoutMs); }
void DriveTuning::forward48() { drivetrain.moveToPoint("TODO_PLACEHOLDER forward 48 test", 0, config::tuning::forwardLongInches, config::tuning::lateralTimeoutMs); }
void DriveTuning::moveToPointTest() { drivetrain.moveToPoint("TODO_PLACEHOLDER moveToPoint test", config::tuning::moveToPointTestX, config::tuning::moveToPointTestY, config::tuning::lateralTimeoutMs); }
void DriveTuning::moveToPoseTest() { drivetrain.moveToPose("TODO_PLACEHOLDER moveToPose test", config::tuning::moveToPointTestX, config::tuning::moveToPointTestY, config::tuning::moveToPoseHeadingDeg, config::tuning::lateralTimeoutMs); }
void DriveTuning::swingTurnTest() { drivetrain.swingToHeading("TODO_PLACEHOLDER swing test", config::tuning::swingTurnHeadingDeg, lemlib::DriveSide::LEFT, config::tuning::angularTimeoutMs); }
void DriveTuning::purePursuitTest() { drivetrain.followPath("TODO_PLACEHOLDER Pure Pursuit test", paths_safe_awp_red_txt, config::tuning::defaultLookahead, 4000, true); }
void DriveTuning::motionChainingTest() {
    logger.info("Testing motion chaining: minSpeed avoids full stop; earlyExitRange trades accuracy for flow");
    drivetrain.moveToPoint("TODO_PLACEHOLDER chain one", 12, 12, 1500, {.minSpeed = 40, .earlyExitRange = 6});
    drivetrain.moveToPose("TODO_PLACEHOLDER chain two", 24, 18, 90, 2200, {.minSpeed = 35, .earlyExitRange = 5});
}

} // namespace tuning
