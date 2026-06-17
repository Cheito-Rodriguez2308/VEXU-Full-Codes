#pragma once

// Intake is a state machine. Driver control and autonomous request states;
// this class decides the motor behavior for each state.

#include "config/robot_config.hpp"
#include "config/robot_identity.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "util/logger.hpp"
#include <memory>

namespace subsystems {

enum class IntakeState {
    Off,
    IntakePin,
    IntakeCup,
    Outtake,
    Hold
};

enum class IntakeScoreHeight {
    Top,
    Middle,
    Bottom
};

enum class IntakeSortState {
    Idle,
    Kickback,
    Routing,
    Settling
};

const char* toString(IntakeState state);

class Intake {
  public:
    Intake(const config::RobotConfig& config, util::Logger& logger);

    void initialize();
    void configureForMatch(bool opcontrol);
    void update(bool hasPin, bool hasCup, bool manualOverride);
    void stop();
    void setState(IntakeState nextState);
    IntakeState getState() const;
    void setScoreHeight(IntakeScoreHeight height);
    void toggleCart();
    void toggleScorerHeight();
    void toggleTrapdoor();
    void extendLever();
    void resetLever();
    void startScan();
    void stopScan();
    void startReleasing();
    void stopReleasing();
    void scanTaskStep();
    void sortTaskStep();
    void debug() const;

  private:
    void updateSmallRobot();
    void updateBigRobot();
    void moveAll(int rpm);
    void moveStorePath(int rpm);
    void moveSmallStore(int rpm);
    void moveSmallReject(int rpm);
    void moveBigStore(int rpm);
    void moveBigScore(IntakeScoreHeight height);
    void setPiston(std::unique_ptr<pros::adi::DigitalOut>& piston, bool value);
    bool objectDetected();
    bool seesRed();
    bool seesBlue();

    config::RobotIdentity robotIdentity;
    int intakeVelocity;
    int activationDistance;
    pros::MotorGroup corridorMotors;
    pros::MotorGroup elevatorMotors;
    pros::MotorGroup judgeMotors;
    pros::MotorGroup scorerMotors;
    std::unique_ptr<pros::adi::DigitalOut> scorerPiston;
    std::unique_ptr<pros::adi::DigitalOut> cartPiston;
    std::unique_ptr<pros::adi::DigitalOut> trapdoorPiston;
    std::unique_ptr<pros::adi::DigitalIn> acceptSensor;
    std::unique_ptr<pros::adi::DigitalIn> rejectSensor;
    pros::Distance distanceSensor;
    pros::Optical opticalSensor;
    util::Logger& logger;
    IntakeState state = IntakeState::Off;
    IntakeScoreHeight scoreHeight = IntakeScoreHeight::Top;
    IntakeSortState sortState = IntakeSortState::Idle;
    bool scanning = false;
    bool releasing = false;
    bool cartDown = false;
    bool scorerRaised = false;
    bool trapdoorOpen = false;
    bool leverExtended = false;
    std::uint32_t sortUntilMs = 0;
};

} // namespace subsystems
