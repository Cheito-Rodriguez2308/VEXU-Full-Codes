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

const char* toString(IntakeAlliance alliance) {
    switch (alliance) {
    case IntakeAlliance::Red:
        return "Red";
    case IntakeAlliance::Blue:
        return "Blue";
    }
    return "Unknown";
}

Intake::Intake(const config::RobotConfig& config, util::Logger& logger)
    : robotIdentity(config.identity),
      intakeVelocity(static_cast<int>(config.aon.intakeVelocity)),
      activationDistance(static_cast<int>(config.aon.intakeActivationDistance)),
      hasCorridorMotors(config.ports.intake.corridor.size() > 0),
      hasElevatorMotors(config.ports.intake.elevator.size() > 0),
      hasJudgeMotors(config.ports.intake.judge.size() > 0),
      hasScorerMotors(config.ports.intake.scorer.size() > 0),
      corridorMotors(config.ports.intake.corridor),
      elevatorMotors(config.ports.intake.elevator),
      judgeMotors(config.ports.intake.judge),
      scorerMotors(config.ports.intake.scorer),
      scorerPiston(config.ports.intake.scorerPiston == 0
                       ? nullptr
                       : std::make_unique<pros::adi::DigitalOut>(config.ports.intake.scorerPiston)),
      cartPiston(config.ports.intake.cartPiston == 0
                     ? nullptr
                     : std::make_unique<pros::adi::DigitalOut>(config.ports.intake.cartPiston)),
      trapdoorPiston(config.ports.intake.trapdoorPiston == 0
                         ? nullptr
                         : std::make_unique<pros::adi::DigitalOut>(config.ports.intake.trapdoorPiston)),
      acceptSensor(config.ports.intake.acceptSensor == 0
                       ? nullptr
                       : std::make_unique<pros::adi::DigitalIn>(config.ports.intake.acceptSensor)),
      rejectSensor(config.ports.intake.rejectSensor == 0
                       ? nullptr
                       : std::make_unique<pros::adi::DigitalIn>(config.ports.intake.rejectSensor)),
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

void Intake::configureForMatch(bool) {
    const bool bigRobot = robotIdentity == config::RobotIdentity::BigRobot;
    const pros::motor_brake_mode_e_t brakeMode = bigRobot ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_COAST;

    configureIfPresent(corridorMotors, hasCorridorMotors, brakeMode);
    configureIfPresent(elevatorMotors, hasElevatorMotors, brakeMode);
    configureIfPresent(judgeMotors, hasJudgeMotors, brakeMode);
    configureIfPresent(scorerMotors, hasScorerMotors, brakeMode);

    if (!bigRobot) {
        stopScan();
    }

    logger.info(bigRobot ? "Intake brake mode: brake" : "Intake brake mode: coast");
}

void Intake::update(bool hasPin, bool hasCup, bool manualOverride) {
    const bool alreadyHasRequestedObject =
        (state == IntakeState::IntakePin && hasPin) ||
        (state == IntakeState::IntakeCup && hasCup);

    if (!manualOverride && alreadyHasRequestedObject) {
        logger.warn("Possession guard blocked intake request");
        state = IntakeState::Hold;
    }

    if (robotIdentity == config::RobotIdentity::BigRobot) {
        updateBigRobot();
    } else {
        updateSmallRobot();
    }
}

void Intake::stop() {
    state = IntakeState::Off;
    releasing = false;
    scanStoreRequested = false;
    sortDecision = IntakeSortDecision::None;
    moveAll(0);
    brakeIfPresent(corridorMotors, hasCorridorMotors);
    brakeIfPresent(elevatorMotors, hasElevatorMotors);
    brakeIfPresent(judgeMotors, hasJudgeMotors);
    brakeIfPresent(scorerMotors, hasScorerMotors);
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

void Intake::setAlliance(IntakeAlliance nextAlliance) {
    if (alliance != nextAlliance) {
        logger.info(std::string("Intake alliance: ") + toString(nextAlliance));
    }
    alliance = nextAlliance;
}

IntakeAlliance Intake::getAlliance() const {
    return alliance;
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
    if (hasScorerMotors) scorerMotors.move_relative(150, intakeVelocity);
}

void Intake::resetLever() {
    leverExtended = false;
    if (hasScorerMotors) scorerMotors.move_absolute(0, intakeVelocity);
}

void Intake::startScan() {
    scanning = true;
}

void Intake::stopScan() {
    scanning = false;
    scanStoreRequested = false;
}

void Intake::startReleasing() {
    releasing = true;
    sortState = IntakeSortState::Idle;
    sortDecision = IntakeSortDecision::None;
}

void Intake::stopReleasing() {
    releasing = false;
    sortState = IntakeSortState::Idle;
    sortDecision = IntakeSortDecision::None;
    moveIfPresent(judgeMotors, hasJudgeMotors, 0);
}

void Intake::scanTaskStep() {
    scanStoreRequested = scanning && objectDetected();
}

void Intake::sortTaskStep() {
    if (robotIdentity != config::RobotIdentity::BigRobot || !releasing) return;

    const std::uint32_t now = pros::millis();
    switch (sortState) {
    case IntakeSortState::Idle:
        sortDecision = sortDecisionFromOptical();
        if (sortDecision != IntakeSortDecision::None) {
            sortState = IntakeSortState::Kickback;
            sortUntilMs = now + 250;
        }
        break;
    case IntakeSortState::Kickback:
        if (now >= sortUntilMs) {
            sortState = IntakeSortState::Routing;
        }
        break;
    case IntakeSortState::Routing: {
        const bool accepted = acceptSensor && acceptSensor->get_value();
        const bool rejected = rejectSensor && rejectSensor->get_value();
        if (accepted || rejected) {
            sortState = IntakeSortState::Settling;
            sortUntilMs = now + 120;
        }
        break;
    }
    case IntakeSortState::Settling:
        if (now >= sortUntilMs) {
            sortState = IntakeSortState::Idle;
            sortDecision = IntakeSortDecision::None;
        }
        break;
    }
}

void Intake::debug() const {
    logger.subsystemState("Intake", toString(state));
}

void Intake::updateSmallRobot() {
    if (scanning && scanStoreRequested) {
        moveSmallStore(intakeVelocity);
        return;
    }

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
        moveAll(0);
        break;
    }
}

void Intake::updateBigRobot() {
    if (releasing) {
        applyBigSortOutputs();
        return;
    }

    if (scanning && scanStoreRequested) {
        moveBigStore(intakeVelocity);
        return;
    }

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
        moveIfPresent(elevatorMotors, hasElevatorMotors, 0);
        moveIfPresent(judgeMotors, hasJudgeMotors, 0);
        break;
    }
}

void Intake::applyBigSortOutputs() {
    switch (sortState) {
    case IntakeSortState::Idle:
        moveBigStore(intakeVelocity);
        break;
    case IntakeSortState::Kickback:
        moveIfPresent(elevatorMotors, hasElevatorMotors, -intakeVelocity);
        moveIfPresent(judgeMotors, hasJudgeMotors, 0);
        break;
    case IntakeSortState::Routing: {
        const bool accept = sortDecision == IntakeSortDecision::Accept;
        const int judgeRpm = accept
                                 ? (scoreHeight == IntakeScoreHeight::Top ? intakeVelocity : -intakeVelocity)
                                 : -intakeVelocity;
        moveIfPresent(elevatorMotors, hasElevatorMotors, intakeVelocity * 2 / 3);
        moveIfPresent(judgeMotors, hasJudgeMotors, judgeRpm);
        break;
    }
    case IntakeSortState::Settling:
        moveIfPresent(elevatorMotors, hasElevatorMotors, intakeVelocity / 2);
        moveIfPresent(judgeMotors, hasJudgeMotors, 0);
        break;
    }
}

void Intake::moveAll(int rpm) {
    moveIfPresent(corridorMotors, hasCorridorMotors, rpm);
    moveIfPresent(elevatorMotors, hasElevatorMotors, rpm);
    moveIfPresent(judgeMotors, hasJudgeMotors, rpm);
    moveIfPresent(scorerMotors, hasScorerMotors, rpm);
}

void Intake::moveStorePath(int rpm) {
    if (robotIdentity == config::RobotIdentity::BigRobot) {
        moveBigStore(rpm);
    } else {
        moveSmallStore(rpm);
    }
}

void Intake::moveSmallStore(int rpm) {
    moveIfPresent(corridorMotors, hasCorridorMotors, rpm);
    moveIfPresent(elevatorMotors, hasElevatorMotors, rpm);
    moveIfPresent(judgeMotors, hasJudgeMotors, rpm);
    moveIfPresent(scorerMotors, hasScorerMotors, 0);
}

void Intake::moveSmallReject(int rpm) {
    moveIfPresent(corridorMotors, hasCorridorMotors, rpm);
    moveIfPresent(elevatorMotors, hasElevatorMotors, rpm);
    moveIfPresent(judgeMotors, hasJudgeMotors, -rpm);
    moveIfPresent(scorerMotors, hasScorerMotors, 0);
}

void Intake::moveBigStore(int rpm) {
    moveIfPresent(elevatorMotors, hasElevatorMotors, rpm);
    moveIfPresent(judgeMotors, hasJudgeMotors, 0);
}

void Intake::moveBigScore(IntakeScoreHeight height) {
    if (height == IntakeScoreHeight::Top) {
        moveIfPresent(elevatorMotors, hasElevatorMotors, intakeVelocity);
        moveIfPresent(judgeMotors, hasJudgeMotors, intakeVelocity);
    } else if (height == IntakeScoreHeight::Middle) {
        moveIfPresent(elevatorMotors, hasElevatorMotors, intakeVelocity);
        moveIfPresent(judgeMotors, hasJudgeMotors, -intakeVelocity);
    } else {
        moveIfPresent(elevatorMotors, hasElevatorMotors, -intakeVelocity);
        moveIfPresent(judgeMotors, hasJudgeMotors, -intakeVelocity);
    }
}

void Intake::setPiston(std::unique_ptr<pros::adi::DigitalOut>& piston, bool value) {
    if (piston) piston->set_value(value);
}

void Intake::moveIfPresent(pros::MotorGroup& motors, bool present, int rpm) {
    if (present) motors.move_velocity(rpm);
}

void Intake::brakeIfPresent(pros::MotorGroup& motors, bool present) {
    if (present) motors.brake();
}

void Intake::configureIfPresent(pros::MotorGroup& motors,
                                bool present,
                                pros::motor_brake_mode_e_t brakeMode) {
    if (!present) return;

    motors.set_brake_mode_all(brakeMode);
    motors.set_gearing_all(pros::E_MOTOR_GEARSET_06);
    motors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    motors.tare_position_all();
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

IntakeSortDecision Intake::sortDecisionFromOptical() {
    const bool red = seesRed();
    const bool blue = seesBlue();
    if (!red && !blue) return IntakeSortDecision::None;

    const bool allianceColor = (alliance == IntakeAlliance::Red && red) ||
                               (alliance == IntakeAlliance::Blue && blue);
    return allianceColor ? IntakeSortDecision::Accept : IntakeSortDecision::Reject;
}

} // namespace subsystems
