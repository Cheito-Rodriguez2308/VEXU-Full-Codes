#pragma once

#include <cstdint>
#include <initializer_list>

namespace config {

struct MotorPorts {
    std::initializer_list<std::int8_t> leftDrive;
    std::initializer_list<std::int8_t> rightDrive;
    std::initializer_list<std::int8_t> centerDrive;
    std::initializer_list<std::int8_t> intake;
    std::int8_t pin;
    std::int8_t cup;
    std::int8_t toggle;
};

struct SensorPorts {
    std::int8_t imu;
    std::int8_t leftVerticalRotation;
    std::int8_t rightVerticalRotation;
    std::int8_t horizontalRotation;
    std::int8_t optical;
    std::int8_t distance;
};

struct AdiPorts {
    char pinPiston;
    char cupPiston;
    char clampPiston;
};

struct Ports {
    MotorPorts motors;
    SensorPorts sensors;
    AdiPorts adi;
};

const Ports& smallRobotPorts();
const Ports& bigRobotPorts();
const Ports& activePorts();

} // namespace config
