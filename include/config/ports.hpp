#pragma once

#include <cstdint>
#include <initializer_list>

namespace config {

struct MotorPorts {
    std::initializer_list<std::int8_t> leftDrive;
    std::initializer_list<std::int8_t> rightDrive;
    std::initializer_list<std::int8_t> centerDrive;
    std::int8_t pin;
    std::int8_t cup;
    std::int8_t toggle;
};

struct IntakePorts {
    std::initializer_list<std::int8_t> corridor;
    std::initializer_list<std::int8_t> elevator;
    std::initializer_list<std::int8_t> judge;
    std::initializer_list<std::int8_t> scorer;
    char scorerPiston;
    char cartPiston;
    char trapdoorPiston;
    std::int8_t distanceSensor;
    std::int8_t opticalSensor;
    char acceptSensor;
    char rejectSensor;
};

struct SensorPorts {
    std::int8_t imu;
    std::int8_t leftVerticalRotation;
    std::int8_t rightVerticalRotation;
    std::int8_t horizontalRotation;
};

struct AdiPorts {
    char arrowPiston;
    char brooksPiston;
    char semPiston;
};

struct Ports {
    MotorPorts motors;
    IntakePorts intake;
    SensorPorts sensors;
    AdiPorts adi;
};

const Ports& smallRobotPorts();
const Ports& bigRobotPorts();
const Ports& activePorts();

} // namespace config
