#pragma once

namespace config {

enum class RobotIdentity {
    RobotA,
    RobotB
};

const char* toString(RobotIdentity identity);
RobotIdentity selectedRobot();

} // namespace config
