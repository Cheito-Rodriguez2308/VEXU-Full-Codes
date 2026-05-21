#include "main.h"
#include "core/robot.hpp"

// PROS callback bridge. Keep match logic in Robot so this file stays boring.
static core::Robot robot;

void initialize() {
    robot.initialize();
}

void disabled() {
    robot.disabled();
}

void competition_initialize() {
    robot.competitionInitialize();
}

void autonomous() {
    robot.autonomous();
}

void opcontrol() {
    robot.opcontrol();
}
