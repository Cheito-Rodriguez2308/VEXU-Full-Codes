#include "subsystems/toggle_mechanism.hpp"
#include "config/constants.hpp"

namespace subsystems {

const char* toString(ToggleMechanismState state) {
    switch (state) {
    case ToggleMechanismState::Idle: return "Idle";
    case ToggleMechanismState::SetRed: return "Set Red";
    case ToggleMechanismState::SetBlue: return "Set Blue";
    case ToggleMechanismState::SetYellow: return "Set Yellow";
    case ToggleMechanismState::Retract: return "Retract";
    }
    return "Unknown";
}

ToggleMechanism::ToggleMechanism(std::int8_t motorPort, util::Logger& logger) : motor(motorPort), logger(logger) {}

void ToggleMechanism::initialize() { stop(); }

void ToggleMechanism::update() {
    switch (state) {
    case ToggleMechanismState::Idle: motor.move_voltage(0); break;
    case ToggleMechanismState::SetRed: motor.move_voltage(config::mechanism::toggleVoltage); break;
    case ToggleMechanismState::SetBlue: motor.move_voltage(config::mechanism::toggleVoltage); break;
    case ToggleMechanismState::SetYellow: motor.move_voltage(config::mechanism::toggleVoltage); break;
    case ToggleMechanismState::Retract: motor.move_voltage(-config::mechanism::toggleVoltage); break;
    }
}

void ToggleMechanism::stop() { state = ToggleMechanismState::Idle; motor.brake(); }
void ToggleMechanism::setState(ToggleMechanismState nextState) { if (state != nextState) logger.subsystemState("Toggle", toString(nextState)); state = nextState; }
ToggleMechanismState ToggleMechanism::getState() const { return state; }
void ToggleMechanism::debug() const { logger.subsystemState("Toggle", toString(state)); }

} // namespace subsystems
