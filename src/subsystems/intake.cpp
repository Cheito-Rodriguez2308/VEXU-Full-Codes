#include "subsystems/intake.hpp"
#include "config/constants.hpp"

namespace subsystems {

const char* toString(IntakeState state) {
    switch (state) {
    case IntakeState::Off: return "Off";
    case IntakeState::IntakePin: return "Intake Pin";
    case IntakeState::IntakeCup: return "Intake Cup";
    case IntakeState::Outtake: return "Outtake";
    case IntakeState::Hold: return "Hold";
    }
    return "Unknown";
}

Intake::Intake(std::int8_t port, util::Logger& logger) : motor(port), logger(logger) {}

void Intake::initialize() {
    stop();
    logger.subsystemState("Intake", toString(state));
}

void Intake::update(bool hasPin, bool hasCup, bool manualOverride) {
    if (!manualOverride && ((state == IntakeState::IntakePin && hasPin) || (state == IntakeState::IntakeCup && hasCup))) {
        logger.warn("Possession guard blocked intake request");
        state = IntakeState::Hold;
    }

    switch (state) {
    case IntakeState::Off: motor.move_voltage(0); break;
    case IntakeState::IntakePin: motor.move_voltage(config::mechanism::intakeVoltage); break;
    case IntakeState::IntakeCup: motor.move_voltage(config::mechanism::intakeVoltage); break;
    case IntakeState::Outtake: motor.move_voltage(config::mechanism::outtakeVoltage); break;
    case IntakeState::Hold: motor.move_voltage(config::mechanism::holdVoltage); break;
    }
}

void Intake::stop() {
    state = IntakeState::Off;
    motor.brake();
}

void Intake::setState(IntakeState nextState) {
    if (state != nextState) logger.subsystemState("Intake", toString(nextState));
    state = nextState;
}

IntakeState Intake::getState() const { return state; }
void Intake::debug() const { logger.subsystemState("Intake", toString(state)); }

} // namespace subsystems
