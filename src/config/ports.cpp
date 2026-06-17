#include "config/ports.hpp"
#include "config/robot_identity.hpp"

namespace config {

const Ports& smallRobotPorts() {
    static const Ports ports{
        {{11, -12, 13, -14}, {1, -2, 3, -4}, {}, {-9, -6, 7, -8}, 0, 0, 0},
        {16, 19, -18, 5, 17, 20},
        {'H', 'B', 'A'}
    };
    return ports;
}

const Ports& bigRobotPorts() {
    static const Ports ports{
        {{12, -13, -18, 19}, {-1, 2, 3, -4}, {-15}, {20, -11, -10, 17}, 0, 0, 0},
        {14, 5, -6, 7, 16, 9},
        {'G', 'H', 'D'}
    };
    return ports;
}

const Ports& activePorts() {
    return selectedRobot() == RobotIdentity::BigRobot ? bigRobotPorts() : smallRobotPorts();
}

} // namespace config
