#include "control/driver_control.hpp"
#include "control/input_curve.hpp"
#include "subsystems/cup_mechanism.hpp"
#include "subsystems/drivetrain.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/pin_mechanism.hpp"
#include "subsystems/sensors.hpp"
#include "subsystems/toggle_mechanism.hpp"
#include "pros/rtos.hpp"

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

    if (drivetrain.driveKind() == config::DriveKind::HDrive) {
        // BigRobot uses the left stick for forward/strafe and the right stick for turn.
        drivetrain.hDrive(leftY, rightX, leftX);
        return;
    }

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
    const bool manualOverride = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    sensors.setManualOverride(manualOverride);

    switch (profile.driver) {
    case config::DriverName::Kevin:
        updateKevinRequests();
        break;
    case config::DriverName::Fabian:
        updateFabianRequests();
        break;
    case config::DriverName::Default:
        updateDefaultRequests();
        break;
    }
}

void DriverControl::updateDefaultRequests() {
    // Buttons request states; subsystems decide motor outputs. This keeps driver control readable.
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

void DriverControl::updateKevinRequests() {
    const std::uint32_t now = pros::millis();

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        if (now - lastR2PressMs < 250) {
            kevinMergeCorridorAndElevator = !kevinMergeCorridorAndElevator;
        }
        lastR2PressMs = now;
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        intake.setState(kevinMergeCorridorAndElevator ? subsystems::IntakeState::IntakeCup
                                                      : subsystems::IntakeState::IntakePin);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        intake.setState(subsystems::IntakeState::Outtake);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        intake.setScoreHeight(subsystems::IntakeScoreHeight::Bottom);
        intake.setState(subsystems::IntakeState::Outtake);
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        intake.setState(subsystems::IntakeState::Off);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        if (now - lastR1PressMs < 250) {
            intake.resetLever();
        } else {
            intake.extendLever();
        }
        lastR1PressMs = now;
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        intake.toggleScorerHeight();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        intake.toggleCart();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        intake.toggleTrapdoor();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        cup.setState(subsystems::CupMechanismState::Stack);
    }

    pin.setState(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)
                     ? subsystems::PinMechanismState::Release
                     : subsystems::PinMechanismState::Grab);
}

void DriverControl::updateFabianRequests() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        intake.setState(subsystems::IntakeState::IntakeCup);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        intake.setScoreHeight(subsystems::IntakeScoreHeight::Bottom);
        intake.setState(subsystems::IntakeState::Outtake);
    } else if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
               !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        intake.setState(subsystems::IntakeState::Off);
    }

    if (fabianSortEnabled) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.setScoreHeight(subsystems::IntakeScoreHeight::Top);
            intake.startReleasing();
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.setScoreHeight(subsystems::IntakeScoreHeight::Middle);
            intake.startReleasing();
        } else if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
                   !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.stopReleasing();
        }
    } else {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.setScoreHeight(subsystems::IntakeScoreHeight::Top);
            intake.setState(subsystems::IntakeState::Outtake);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.setScoreHeight(subsystems::IntakeScoreHeight::Middle);
            intake.setState(subsystems::IntakeState::Outtake);
        }
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        cup.setState(subsystems::CupMechanismState::Stack);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        pin.setState(subsystems::PinMechanismState::Grab);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        intake.toggleCart();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        fabianSortEnabled = !fabianSortEnabled;
        if (!fabianSortEnabled) intake.stopReleasing();
    }
}

} // namespace control
