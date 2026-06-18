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

enum class IntakeAlliance {
    Red,
    Blue
};

enum class IntakeSortDecision {
    None,
    Accept,
    Reject
};

const char* toString(IntakeState state);
const char* toString(IntakeAlliance alliance);

class Intake {
  public:
    Intake(const config::RobotConfig& config, util::Logger& logger);

    void initialize();
    void configureForMatch(bool opcontrol);
    void update(bool hasPin, bool hasCup, bool manualOverride);
    void stop();
    void setState(IntakeState nextState);
    IntakeState getState() const;
    void setAlliance(IntakeAlliance nextAlliance);
    IntakeAlliance getAlliance() const;
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
    void applyBigSortOutputs();
    void moveAll(int rpm);
    void moveStorePath(int rpm);
    void moveSmallStore(int rpm);
    void moveSmallReject(int rpm);
    void moveBigStore(int rpm);
    void moveBigScore(IntakeScoreHeight height);
    void setPiston(std::unique_ptr<pros::adi::DigitalOut>& piston, bool value);
    void moveIfPresent(pros::MotorGroup& motors, bool present, int rpm);
    void brakeIfPresent(pros::MotorGroup& motors, bool present);
    void configureIfPresent(pros::MotorGroup& motors,
                            bool present,
                            pros::motor_brake_mode_e_t brakeMode);
    bool objectDetected();
    bool seesRed();
    bool seesBlue();
    IntakeSortDecision sortDecisionFromOptical();

    config::RobotIdentity robotIdentity;
    int intakeVelocity;
    int activationDistance;
    bool hasCorridorMotors;
    bool hasElevatorMotors;
    bool hasJudgeMotors;
    bool hasScorerMotors;
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
    IntakeAlliance alliance = IntakeAlliance::Red;
    IntakeScoreHeight scoreHeight = IntakeScoreHeight::Top;
    IntakeSortState sortState = IntakeSortState::Idle;
    IntakeSortDecision sortDecision = IntakeSortDecision::None;
    bool scanning = false;
    bool scanStoreRequested = false;
    bool releasing = false;
    bool cartDown = false;
    bool scorerRaised = false;
    bool trapdoorOpen = false;
    bool leverExtended = false;
    std::uint32_t sortUntilMs = 0;
};

} // namespace subsystems
