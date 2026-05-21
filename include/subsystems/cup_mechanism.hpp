#pragma once

// CupMechanism stays small so we can map each state to a real mechanism action
// during testing.

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "util/logger.hpp"

namespace subsystems {

enum class CupMechanismState {
    Idle,
    Grab,
    Hold,
    Stack,
    Release
};

const char* toString(CupMechanismState state);

class CupMechanism {
  public:
    CupMechanism(std::int8_t motorPort, char pistonPort, util::Logger& logger);
    void initialize();
    void update();
    void stop();
    void setState(CupMechanismState nextState);
    CupMechanismState getState() const;
    void debug() const;

  private:
    pros::Motor motor;
    pros::adi::DigitalOut piston;
    util::Logger& logger;
    CupMechanismState state = CupMechanismState::Idle;
};

} // namespace subsystems
