#pragma once

#include "lemlib/api.hpp"
#include "subsystems/drivetrain.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/pin_mechanism.hpp"
#include "subsystems/cup_mechanism.hpp"
#include "subsystems/toggle_mechanism.hpp"
#include "subsystems/sensors.hpp"
#include "util/logger.hpp"

namespace auton {

class AutonActions {
  public:
    AutonActions(subsystems::Drivetrain& drivetrain, subsystems::Intake& intake,
                 subsystems::PinMechanism& pin, subsystems::CupMechanism& cup,
                 subsystems::ToggleMechanism& toggle, subsystems::Sensors& sensors,
                 util::Logger& logger);

    void driveToPoint(const char* name, double x, double y, int timeout, lemlib::MoveToPointParams params = {});
    void driveToPose(const char* name, double x, double y, double theta, int timeout, lemlib::MoveToPoseParams params = {});
    void turnToHeading(const char* name, double heading, int timeout, lemlib::TurnToHeadingParams params = {});
    void turnToPoint(const char* name, double x, double y, int timeout, lemlib::TurnToPointParams params = {});
    void swingToHeading(const char* name, double heading, lemlib::DriveSide side, int timeout, lemlib::SwingToHeadingParams params = {});
    void swingToPoint(const char* name, double x, double y, lemlib::DriveSide side, int timeout, lemlib::SwingToPointParams params = {});
    void followPath(const char* name, const asset& path, double lookahead, int timeout, bool forwards);
    void scorePin();
    void scoreCup();
    void setToggle(subsystems::ToggleMechanismState state);
    void waitForSensor(const char* name, int timeoutMs);
    void cancelMotionIf(bool condition, const char* reason);

    void chainAroundGoal();
    void approachAndScore();
    void intakeUntilDetected();

  private:
    subsystems::Drivetrain& drivetrain;
    subsystems::Intake& intake;
    subsystems::PinMechanism& pin;
    subsystems::CupMechanism& cup;
    subsystems::ToggleMechanism& toggle;
    subsystems::Sensors& sensors;
    util::Logger& logger;
};

} // namespace auton
