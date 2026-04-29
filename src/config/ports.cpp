#include "config/ports.hpp"
#include "config/robot_identity.hpp"

namespace config {

const Ports& robotAPorts() {
    static const Ports ports{
        {{1, -2, 3}, {-4, 5, -6}, 7, 8, 9, 10}, // TODO_PLACEHOLDER fake motor ports; negative ports mean reversed.
        {11, 12, 13, 14, 15},                   // TODO_PLACEHOLDER fake IMU/rotation/optical/distance ports.
        {'A', 'B', 'C'}                         // TODO_PLACEHOLDER fake ADI pneumatic ports.
    };
    return ports;
}

const Ports& robotBPorts() {
    static const Ports ports{
        {{-1, 2, -3}, {4, -5, 6}, 7, 8, 9, 10}, // TODO_PLACEHOLDER fake Robot B motor ports.
        {11, 12, 13, 14, 15},                   // TODO_PLACEHOLDER fake Robot B sensor ports.
        {'A', 'B', 'C'}                         // TODO_PLACEHOLDER fake Robot B ADI ports.
    };
    return ports;
}

const Ports& activePorts() {
    return selectedRobot() == RobotIdentity::RobotA ? robotAPorts() : robotBPorts();
}

} // namespace config
