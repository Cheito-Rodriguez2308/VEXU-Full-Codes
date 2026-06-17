#include "subsystems/intake.hpp"
#include "pros/rtos.hpp"

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

Intake::Intake(const config::RobotConfig& config, util::Logger& logger)
    : robotIdentity(config.identity),
      intakeVelocity(static_cast<int>(config.aon.intakeVelocity)),
      activationDistance(static_cast<int>(config.aon.intakeActivationDistance)),
      corridorMotors(config.ports.intake.corridor),
      elevatorMotors(config.ports.intake.elevator),
      judgeMotors(config.ports.intake.judge),
      scorerMotors(config.ports.intake.scorer),
      scorerPiston(config.ports.intake.scorerPiston),
      cartPiston(config.ports.intake.cartPiston),
      trapdoorPiston(config.ports.intake.trapdoorPiston),
      acceptSensor(config.ports.intake.acceptSensor),
      rejectSensor(config.ports.intake.rejectSensor),
      distanceSensor(config.ports.intake.distanceSensor),
      opticalSensor(config.ports.intake.opticalSensor),
      logger(logger) {}

void Intake::initialize() {
    opticalSensor.set_led_pwm(50);
    setPiston(scorerPiston, false);
    setPiston(cartPiston, false);
    setPiston(trapdoorPiston, false);
    scanning = robotIdentity == config::RobotIdentity::SmallRobot;
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

    updateScan();
    updateSort();

    if (robotIdentity == config::RobotIdentity::BigRobot) {
        updateBigRobot();
    } else {
        updateSmallRobot();
    }
}

void Intake::stop() {
    state = IntakeState::Off;
    releasing = false;
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

void Intake::setScoreHeight(IntakeScoreHeight height) {
    scoreHeight = height;
}

void Intake::toggleCart() {
    cartDown = !cartDown;
    setPiston(cartPiston, cartDown);
}

void Intake::toggleScorerHeight() {
    scorerRaised = !scorerRaised;
    setPiston(scorerPiston, scorerRaised);
}

void Intake::toggleTrapdoor() {
    trapdoorOpen = !trapdoorOpen;
    setPiston(trapdoorPiston, trapdoorOpen);
}

void Intake::extendLever() {
    leverExtended = true;
    scorerMotors.move_relative(150, intakeVelocity);
}

void Intake::resetLever() {
    leverExtended = false;
    scorerMotors.move_absolute(0, intakeVelocity);
}

void Intake::startScan() {
    scanning = true;
}

void Intake::stopScan() {
    scanning = false;
}

void Intake::startReleasing() {
    releasing = true;
    sortState = IntakeSortState::Idle;
}

void Intake::stopReleasing() {
    releasing = false;
    sortState = IntakeSortState::Idle;
    judgeMotors.move_velocity(0);
}

void Intake::debug() const {
    logger.subsystemState("Intake", toString(state));
}

void Intake::updateSmallRobot() {
    switch (state) {
    case IntakeState::Off:
        moveAll(0);
        break;
    case IntakeState::IntakePin:
    case IntakeState::IntakeCup:
        moveSmallStore(intakeVelocity);
        break;
    case IntakeState::Outtake:
        moveSmallReject(intakeVelocity);
        break;
    case IntakeState::Hold:
        corridorMotors.move_velocity(0);
        elevatorMotors.move_velocity(0);
        judgeMotors.move_velocity(0);
        scorerMotors.move_velocity(0);
        break;
    }
}

void Intake::updateBigRobot() {
    switch (state) {
    case IntakeState::Off:
        moveAll(0);
        break;
    case IntakeState::IntakePin:
    case IntakeState::IntakeCup:
        moveBigStore(intakeVelocity);
        break;
    case IntakeState::Outtake:
        moveBigScore(scoreHeight);
        break;
    case IntakeState::Hold:
        elevatorMotors.move_velocity(0);
        judgeMotors.move_velocity(0);
        break;
    }
}

void Intake::updateScan() {
    if (!scanning || !objectDetected()) return;

    // AON used scan as a helper to keep objects moving once detected.
    if (robotIdentity == config::RobotIdentity::BigRobot) {
        moveBigStore(intakeVelocity);
    } else {
        moveSmallStore(intakeVelocity);
    }
}

void Intake::updateSort() {
    if (robotIdentity != config::RobotIdentity::BigRobot || !releasing) return;

    const std::uint32_t now = pros::millis();
    switch (sortState) {
    case IntakeSortState::Idle:
        if (seesRed() || seesBlue()) {
            sortState = IntakeSortState::Kickback;
            sortUntilMs = now + 250;
            elevatorMotors.move_velocity(-intakeVelocity);
            judgeMotors.move_velocity(0);
        }
        break;
    case IntakeSortState::Kickback:
        if (now >= sortUntilMs) {
            sortState = IntakeSortState::Routing;
            elevatorMotors.move_velocity(intakeVelocity * 2 / 3);
            judgeMotors.move_velocity(scoreHeight == IntakeScoreHeight::Top ? intakeVelocity : -intakeVelocity);
        }
        break;
    case IntakeSortState::Routing:
        if (acceptSensor.get_value() || rejectSensor.get_value()) {
            sortState = IntakeSortState::Settling;
            sortUntilMs = now + 120;
            judgeMotors.move_velocity(0);
        }
        break;
    case IntakeSortState::Settling:
        if (now >= sortUntilMs) {
            sortState = IntakeSortState::Idle;
        }
        break;
    }
}

void Intake::moveAll(int rpm) {
    corridorMotors.move_velocity(rpm);
    elevatorMotors.move_velocity(rpm);
    judgeMotors.move_velocity(rpm);
    scorerMotors.move_velocity(rpm);
}

void Intake::moveStorePath(int rpm) {
    if (robotIdentity == config::RobotIdentity::BigRobot) {
        moveBigStore(rpm);
    } else {
        moveSmallStore(rpm);
    }
}

void Intake::moveSmallStore(int rpm) {
    corridorMotors.move_velocity(rpm);
    elevatorMotors.move_velocity(rpm);
    judgeMotors.move_velocity(rpm);
    scorerMotors.move_velocity(0);
}

void Intake::moveSmallReject(int rpm) {
    corridorMotors.move_velocity(rpm);
    elevatorMotors.move_velocity(rpm);
    judgeMotors.move_velocity(-rpm);
    scorerMotors.move_velocity(0);
}

void Intake::moveBigStore(int rpm) {
    elevatorMotors.move_velocity(rpm);
    judgeMotors.move_velocity(0);
}

void Intake::moveBigScore(IntakeScoreHeight height) {
    if (height == IntakeScoreHeight::Top) {
        elevatorMotors.move_velocity(intakeVelocity);
        judgeMotors.move_velocity(intakeVelocity);
    } else if (height == IntakeScoreHeight::Middle) {
        elevatorMotors.move_velocity(intakeVelocity);
        judgeMotors.move_velocity(-intakeVelocity);
    } else {
        elevatorMotors.move_velocity(-intakeVelocity);
        judgeMotors.move_velocity(-intakeVelocity);
    }
}

void Intake::setPiston(pros::adi::DigitalOut& piston, bool value) {
    piston.set_value(value);
}

bool Intake::objectDetected() {
    const int distance = distanceSensor.get();
    return distance > 0 && distance <= activationDistance;
}

bool Intake::seesRed() {
    const double hue = opticalSensor.get_hue();
    return (hue >= 356.0 && hue <= 359.0) || (hue >= 1.0 && hue <= 25.0);
}

bool Intake::seesBlue() {
    const double hue = opticalSensor.get_hue();
    return hue >= 170.0 && hue <= 230.0;
}

} // namespace subsystems
