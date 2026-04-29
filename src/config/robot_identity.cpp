#include "config/robot_identity.hpp"

namespace config {

const char* toString(RobotIdentity identity) {
    return identity == RobotIdentity::RobotA ? "Robot A" : "Robot B";
}

RobotIdentity selectedRobot() {
    // TODO_PLACEHOLDER portfolio default. Change this on each brain or replace with an SD-card/team selector.
    return RobotIdentity::RobotA;
}

} // namespace config
