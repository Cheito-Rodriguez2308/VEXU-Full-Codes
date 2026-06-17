#pragma once

// Intake is a state machine. Driver control and autonomous request states;
// this class decides the motor behavior for each state.

#include "config/ports.hpp"
#include "pros/motor_group.hpp"
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
    Intake(const config::IntakePorts& ports, util::Logger& logger);

    void initialize();
    void update(bool hasPin, bool hasCup, bool manualOverride);
    void stop();
    void setState(IntakeState nextState);
    IntakeState getState() const;
    void debug() const;

  private:
    void moveAll(int voltage);
    void moveStorePath(int voltage);

    pros::MotorGroup corridorMotors;
    pros::MotorGroup elevatorMotors;
    pros::MotorGroup judgeMotors;
    pros::MotorGroup scorerMotors;
    util::Logger& logger;
    IntakeState state = IntakeState::Off;
};

} // namespace subsystems
