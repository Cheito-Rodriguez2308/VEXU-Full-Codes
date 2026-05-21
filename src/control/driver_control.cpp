#include "control/driver_control.hpp"
#include "control/input_curve.hpp"
#include "subsystems/cup_mechanism.hpp"
#include "subsystems/drivetrain.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/pin_mechanism.hpp"
#include "subsystems/sensors.hpp"
#include "subsystems/toggle_mechanism.hpp"

namespace control {

DriverControl::DriverControl(pros::Controller& controller, subsystems::Drivetrain& drivetrain,
                             subsystems::Intake& intake, subsystems::PinMechanism& pin,
                             subsystems::CupMechanism& cup, subsystems::ToggleMechanism& toggle,
                             subsystems::Sensors& sensors)
    : controller(controller), drivetrain(drivetrain), intake(intake), pin(pin), cup(cup), toggle(toggle),
      sensors(sensors), profile(defaultDriverProfile()) {}

void DriverControl::initialize() {
    controller.set_text(0, 0, "Driver ready");
}

void DriverControl::update() {
    updateDrive();
    updateSubsystemRequests();
}

void DriverControl::setProfile(const DriverProfile& nextProfile) {
    profile = nextProfile;
}

const DriverProfile& DriverControl::getProfile() const {
    return profile;
}

void DriverControl::updateDrive() {
    InputCurve curve(profile);
    const bool precision = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    const int leftY = curve.limit(curve.throttle(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)), precision);
    const int rightY = curve.limit(curve.throttle(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)), precision);
    const int leftX = curve.limit(curve.steer(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)), precision);
    const int rightX = curve.limit(curve.steer(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), precision);

    switch (profile.mode) {
    case DriveMode::Tank:
        drivetrain.tank(leftY, rightY);
        break;
    case DriveMode::SingleStickArcade:
        drivetrain.arcade(leftY, leftX, profile.desaturateBias);
        break;
    case DriveMode::DoubleStickArcade:
        drivetrain.arcade(leftY, rightX, profile.desaturateBias);
        break;
    case DriveMode::Curvature:
        drivetrain.curvature(leftY, rightX);
        break;
    }
}

void DriverControl::updateSubsystemRequests() {
    // Buttons request states; subsystems decide motor outputs. This keeps driver control readable.
    const bool manualOverride = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    sensors.setManualOverride(manualOverride);

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake.setState(subsystems::IntakeState::IntakePin);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        intake.setState(subsystems::IntakeState::IntakeCup);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        intake.setState(subsystems::IntakeState::Outtake);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        intake.setState(subsystems::IntakeState::Off);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        pin.setState(subsystems::PinMechanismState::Grab);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        pin.setState(subsystems::PinMechanismState::Release);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        cup.setState(subsystems::CupMechanismState::Stack);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        cup.setState(subsystems::CupMechanismState::Release);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        toggle.setState(subsystems::ToggleMechanismState::SetRed);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        toggle.setState(subsystems::ToggleMechanismState::Retract);
    }
}

} // namespace control
