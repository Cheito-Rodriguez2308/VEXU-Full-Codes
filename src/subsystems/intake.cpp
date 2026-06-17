#include "subsystems/intake.hpp"
#include "config/constants.hpp"

namespace subsystems {

const char* toString(IntakeState state) {
    switch (state) {
    case IntakeState::Off:
        return "Off";
    case IntakeState::IntakePin:
        return "Intake Pin";
    case IntakeState::IntakeCup:
        return "Intake Cup";
    case IntakeState::Outtake:
        return "Outtake";
    case IntakeState::Hold:
        return "Hold";
    }
    return "Unknown";
}

Intake::Intake(const config::IntakePorts& ports, util::Logger& logger)
    : corridorMotors(ports.corridor),
      elevatorMotors(ports.elevator),
      judgeMotors(ports.judge),
      scorerMotors(ports.scorer),
      logger(logger) {}

void Intake::initialize() {
    stop();
    logger.subsystemState("Intake", toString(state));
}

void Intake::update(bool hasPin, bool hasCup, bool manualOverride) {
    const bool alreadyHasRequestedObject =
        (state == IntakeState::IntakePin && hasPin) ||
        (state == IntakeState::IntakeCup && hasCup);

    if (!manualOverride && alreadyHasRequestedObject) {
        logger.warn("Possession guard blocked intake request");
        state = IntakeState::Hold;
    }

    switch (state) {
    case IntakeState::Off:
        moveAll(0);
        break;
    case IntakeState::IntakePin:
        moveStorePath(config::mechanism::intakeVoltage);
        break;
    case IntakeState::IntakeCup:
        moveStorePath(config::mechanism::intakeVoltage);
        break;
    case IntakeState::Outtake:
        moveAll(config::mechanism::outtakeVoltage);
        break;
    case IntakeState::Hold:
        elevatorMotors.move_voltage(config::mechanism::holdVoltage);
        corridorMotors.move_voltage(0);
        judgeMotors.move_voltage(0);
        scorerMotors.move_voltage(0);
        break;
    }
}

void Intake::stop() {
    state = IntakeState::Off;
    moveAll(0);
    corridorMotors.brake();
    elevatorMotors.brake();
    judgeMotors.brake();
    scorerMotors.brake();
}

void Intake::setState(IntakeState nextState) {
    if (state != nextState) {
        logger.subsystemState("Intake", toString(nextState));
    }
    state = nextState;
}

IntakeState Intake::getState() const {
    return state;
}

void Intake::debug() const {
    logger.subsystemState("Intake", toString(state));
}

void Intake::moveAll(int voltage) {
    corridorMotors.move_voltage(voltage);
    elevatorMotors.move_voltage(voltage);
    judgeMotors.move_voltage(voltage);
    scorerMotors.move_voltage(voltage);
}

void Intake::moveStorePath(int voltage) {
    // AON's robots do not share one intake layout. These are the motors that
    // usually move objects inward without forcing scorer/judge behavior.
    corridorMotors.move_voltage(voltage);
    elevatorMotors.move_voltage(voltage);
    judgeMotors.move_voltage(0);
    scorerMotors.move_voltage(0);
}

} // namespace subsystems
