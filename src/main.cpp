#include "main.h"
#include "core/robot.hpp"

// Entry point for the PROS competition template.
// Keep this file tiny: PROS owns callbacks, core::Robot owns robot behavior.
static core::Robot robot;

void initialize() { robot.initialize(); }
void disabled() { robot.disabled(); }
void competition_initialize() { robot.competitionInitialize(); }
void autonomous() { robot.autonomous(); }
void opcontrol() { robot.opcontrol(); }
