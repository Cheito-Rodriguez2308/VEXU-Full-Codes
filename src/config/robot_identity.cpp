#include "config/robot_identity.hpp"

namespace config {

const char* toString(RobotIdentity identity) {
    return identity == RobotIdentity::BigRobot ? "BigRobot" : "SmallRobot";
}

RobotIdentity selectedRobot() {
    return USING_BIG_ROBOT ? RobotIdentity::BigRobot : RobotIdentity::SmallRobot;
}

} // namespace config
