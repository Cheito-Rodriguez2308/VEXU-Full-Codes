#include "main.h"
#include "core/robot.hpp"

// main.cpp stays intentionally small. PROS owns these callbacks; our Robot class
// owns the actual robot architecture so students can reason about one object.
static core::Robot robot;

void initialize() { robot.initialize(); }
void disabled() { robot.disabled(); }
void competition_initialize() { robot.competitionInitialize(); }
void autonomous() { robot.autonomous(); }
void opcontrol() { robot.opcontrol(); }
