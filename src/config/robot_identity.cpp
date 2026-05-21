#include "config/robot_identity.hpp"

namespace config {

const char* toString(RobotIdentity identity) {
    return identity == RobotIdentity::RobotA ? "Robot A" : "Robot B";
}

RobotIdentity selectedRobot() {
    // TODO: Set this per brain, or replace it with an SD-card/team selector.
    return RobotIdentity::RobotA;
}

} // namespace config
