#include "subsystems/cup_mechanism.hpp"
#include "config/constants.hpp"

namespace subsystems {

const char* toString(CupMechanismState state) {
    switch (state) {
    case CupMechanismState::Idle: return "Idle";
    case CupMechanismState::Grab: return "Grab";
    case CupMechanismState::Hold: return "Hold";
    case CupMechanismState::Stack: return "Stack";
    case CupMechanismState::Release: return "Release";
    }
    return "Unknown";
}

CupMechanism::CupMechanism(std::int8_t motorPort, char pistonPort, util::Logger& logger)
    : motor(motorPort), piston(pistonPort), logger(logger) {}

void CupMechanism::initialize() {
    piston.set_value(false);
    stop();
}

void CupMechanism::update() {
    switch (state) {
    case CupMechanismState::Idle: motor.move_voltage(0); break;
    case CupMechanismState::Grab: piston.set_value(true); motor.move_voltage(config::mechanism::grabVoltage); break;
    case CupMechanismState::Hold: piston.set_value(true); motor.move_voltage(config::mechanism::holdVoltage); break;
    case CupMechanismState::Stack: piston.set_value(true); motor.move_voltage(config::mechanism::scoreVoltage); break;
    case CupMechanismState::Release: piston.set_value(false); motor.move_voltage(config::mechanism::releaseVoltage); break;
    }
}

void CupMechanism::stop() { state = CupMechanismState::Idle; motor.brake(); }
void CupMechanism::setState(CupMechanismState nextState) { if (state != nextState) logger.subsystemState("Cup", toString(nextState)); state = nextState; }
CupMechanismState CupMechanism::getState() const { return state; }
void CupMechanism::debug() const { logger.subsystemState("Cup", toString(state)); }

} // namespace subsystems
