#include "config/ports.hpp"
#include "config/robot_identity.hpp"

namespace config {

const Ports& smallRobotPorts() {
    static const Ports ports{
        {{11, -12, 13, -14}, {1, -2, 3, -4}, {}, 0, 0, 0},
        {{-9}, {-6}, {7}, {-8}, 'H', 'B', 'A', 20, 17, 0, 0},
        {16, 19, -18, 5},
        {'C', 'G', 0}
    };
    return ports;
}

const Ports& bigRobotPorts() {
    static const Ports ports{
        {{12, -13, -18, 19}, {-1, 2, 3, -4}, {-15}, 0, 0, 0},
        {{}, {20, -11, -10}, {17}, {}, 0, 'H', 0, 9, 16, 'F', 'E'},
        {14, 5, -6, 7},
        {0, 'D', 'G'}
    };
    return ports;
}

const Ports& activePorts() {
    return selectedRobot() == RobotIdentity::BigRobot ? bigRobotPorts() : smallRobotPorts();
}

} // namespace config
