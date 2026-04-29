#pragma once

#include "control/driver_profiles.hpp"
#include "pros/misc.hpp"

namespace subsystems {
class Drivetrain;
class Intake;
class PinMechanism;
class CupMechanism;
class ToggleMechanism;
class Sensors;
}

namespace control {

class DriverControl {
  public:
    DriverControl(pros::Controller& controller, subsystems::Drivetrain& drivetrain, subsystems::Intake& intake,
                  subsystems::PinMechanism& pin, subsystems::CupMechanism& cup,
                  subsystems::ToggleMechanism& toggle, subsystems::Sensors& sensors);

    void initialize();
    void update();
    void setProfile(const DriverProfile& nextProfile);
    const DriverProfile& getProfile() const;

  private:
    void updateDrive();
    void updateSubsystemRequests();

    pros::Controller& controller;
    subsystems::Drivetrain& drivetrain;
    subsystems::Intake& intake;
    subsystems::PinMechanism& pin;
    subsystems::CupMechanism& cup;
    subsystems::ToggleMechanism& toggle;
    subsystems::Sensors& sensors;
    DriverProfile profile;
};

} // namespace control
