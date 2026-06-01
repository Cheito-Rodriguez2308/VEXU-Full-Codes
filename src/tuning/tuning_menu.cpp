#include "tuning/tuning_menu.hpp"

namespace tuning {

TuningMenu::TuningMenu(pros::Controller& controller,
                       DriveTuning& driveTuning,
                       PidTuning& pidTuning,
                       SensorTuning& sensorTuning,
                       AutoTuner& autoTuner,
                       util::Logger& logger)
    : controller(controller), driveTuning(driveTuning), pidTuning(pidTuning), sensorTuning(sensorTuning),
      autoTuner(autoTuner),
      logger(logger) {}

void TuningMenu::initialize() {
    logger.info("Tuning menu ready; disabled by default");
}

void TuningMenu::update() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) &&
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        setEnabled(!tuningEnabled);
    }
    if (!tuningEnabled) {
        return;
    }

    // R1 keeps AutoTuner harder to start by accident.
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            autoTuner.runAngularTest();
            return;
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            autoTuner.runLateralTest();
            return;
        }
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        driveTuning.angular90();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        driveTuning.forward24();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        driveTuning.moveToPoseTest();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        pidTuning.explainWorkflow();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        sensorTuning.trackingWheelDirectionTest();
    }
}

bool TuningMenu::enabled() const {
    return tuningEnabled;
}

void TuningMenu::setEnabled(bool value) {
    tuningEnabled = value;
    logger.info(tuningEnabled ? "Tuning enabled" : "Tuning disabled");
}

} // namespace tuning
