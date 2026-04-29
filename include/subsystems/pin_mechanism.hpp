#pragma once

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "util/logger.hpp"

namespace subsystems {

enum class PinMechanismState {
    Idle,
    Grab,
    Hold,
    Score,
    Release
};

const char* toString(PinMechanismState state);

class PinMechanism {
  public:
    PinMechanism(std::int8_t motorPort, char pistonPort, util::Logger& logger);
    void initialize();
    void update();
    void stop();
    void setState(PinMechanismState nextState);
    PinMechanismState getState() const;
    void debug() const;

  private:
    pros::Motor motor;
    pros::adi::DigitalOut piston;
    util::Logger& logger;
    PinMechanismState state = PinMechanismState::Idle;
};

} // namespace subsystems
