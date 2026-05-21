#pragma once

// Intake is a state machine. Driver control and autonomous request states;
// this class decides the motor behavior for each state.

#include "pros/motors.hpp"
#include "util/logger.hpp"

namespace subsystems {

enum class IntakeState {
    Off,
    IntakePin,
    IntakeCup,
    Outtake,
    Hold
};

const char* toString(IntakeState state);

class Intake {
  public:
    Intake(std::int8_t port, util::Logger& logger);

    void initialize();
    void update(bool hasPin, bool hasCup, bool manualOverride);
    void stop();
    void setState(IntakeState nextState);
    IntakeState getState() const;
    void debug() const;

  private:
    pros::Motor motor;
    util::Logger& logger;
    IntakeState state = IntakeState::Off;
};

} // namespace subsystems
