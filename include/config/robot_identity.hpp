#pragma once

#ifndef USING_BIG_ROBOT
#define USING_BIG_ROBOT false
#endif

namespace config {

enum class RobotIdentity {
    SmallRobot,
    BigRobot
};

const char* toString(RobotIdentity identity);
RobotIdentity selectedRobot();

} // namespace config
