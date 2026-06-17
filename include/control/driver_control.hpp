#pragma once

// DriverControl translates controller intent into subsystem state requests.
// It avoids direct mechanism voltages so drivers can change behavior safely.

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
    void updateDefaultRequests();
    void updateKevinRequests();
    void updateFabianRequests();

    pros::Controller& controller;
    subsystems::Drivetrain& drivetrain;
    subsystems::Intake& intake;
    subsystems::PinMechanism& pin;
    subsystems::CupMechanism& cup;
    subsystems::ToggleMechanism& toggle;
    subsystems::Sensors& sensors;
    DriverProfile profile;
    std::uint32_t lastR1PressMs = 0;
    std::uint32_t lastR2PressMs = 0;
    bool kevinMergeCorridorAndElevator = true;
    bool fabianSortEnabled = true;
};

} // namespace control
