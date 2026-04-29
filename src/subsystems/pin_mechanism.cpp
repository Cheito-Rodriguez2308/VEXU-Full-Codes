#include "subsystems/pin_mechanism.hpp"
#include "config/constants.hpp"

namespace subsystems {

const char* toString(PinMechanismState state) {
    switch (state) {
    case PinMechanismState::Idle: return "Idle";
    case PinMechanismState::Grab: return "Grab";
    case PinMechanismState::Hold: return "Hold";
    case PinMechanismState::Score: return "Score";
    case PinMechanismState::Release: return "Release";
    }
    return "Unknown";
}

PinMechanism::PinMechanism(std::int8_t motorPort, char pistonPort, util::Logger& logger)
    : motor(motorPort), piston(pistonPort), logger(logger) {}

void PinMechanism::initialize() {
    piston.set_value(false);
    stop();
}

void PinMechanism::update() {
    switch (state) {
    case PinMechanismState::Idle: motor.move_voltage(0); break;
    case PinMechanismState::Grab: piston.set_value(true); motor.move_voltage(config::mechanism::grabVoltage); break;
    case PinMechanismState::Hold: piston.set_value(true); motor.move_voltage(config::mechanism::holdVoltage); break;
    case PinMechanismState::Score: piston.set_value(true); motor.move_voltage(config::mechanism::scoreVoltage); break;
    case PinMechanismState::Release: piston.set_value(false); motor.move_voltage(config::mechanism::releaseVoltage); break;
    }
}

void PinMechanism::stop() { state = PinMechanismState::Idle; motor.brake(); }
void PinMechanism::setState(PinMechanismState nextState) { if (state != nextState) logger.subsystemState("Pin", toString(nextState)); state = nextState; }
PinMechanismState PinMechanism::getState() const { return state; }
void PinMechanism::debug() const { logger.subsystemState("Pin", toString(state)); }

} // namespace subsystems
