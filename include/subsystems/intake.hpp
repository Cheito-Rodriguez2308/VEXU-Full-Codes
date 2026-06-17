#pragma once

// Intake is a state machine. Driver control and autonomous request states;
// this class decides the motor behavior for each state.

#include "pros/motor_group.hpp"
#include "util/logger.hpp"
#include <initializer_list>

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
    Intake(std::initializer_list<std::int8_t> ports, util::Logger& logger);

    void initialize();
    void update(bool hasPin, bool hasCup, bool manualOverride);
    void stop();
    void setState(IntakeState nextState);
    IntakeState getState() const;
    void debug() const;

  private:
    pros::MotorGroup motors;
    util::Logger& logger;
    IntakeState state = IntakeState::Off;
};

} // namespace subsystems
