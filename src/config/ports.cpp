#include "config/ports.hpp"
#include "config/robot_identity.hpp"

namespace config {

const Ports& robotAPorts() {
    static const Ports ports{
        {{1, -2, 3}, {-4, 5, -6}, 7, 8, 9, 10}, // TODO: sample motor ports; negative ports mean reversed.
        {11, 12, 13, 14, 15},                   // TODO: sample IMU/rotation/optical/distance ports.
        {'A', 'B', 'C'}                         // TODO: sample ADI pneumatic ports.
    };
    return ports;
}

const Ports& robotBPorts() {
    static const Ports ports{
        {{-1, 2, -3}, {4, -5, 6}, 7, 8, 9, 10}, // TODO: sample Robot B motor ports.
        {11, 12, 13, 14, 15},                   // TODO: sample Robot B sensor ports.
        {'A', 'B', 'C'}                         // TODO: sample Robot B ADI ports.
    };
    return ports;
}

const Ports& activePorts() {
    return selectedRobot() == RobotIdentity::RobotA ? robotAPorts() : robotBPorts();
}

} // namespace config
