#include "auton/auton_selector.hpp"
#include "auton/auton_routines.hpp"

namespace auton {

const char* toString(AutonRoutine routine) {
    switch (routine) {
    case AutonRoutine::RedSafeAWP:
        return "Red Safe AWP";
    case AutonRoutine::BlueSafeAWP:
        return "Blue Safe AWP";
    case AutonRoutine::RedGoalStackPriority:
        return "Red Goal Stack";
    case AutonRoutine::BlueGoalStackPriority:
        return "Blue Goal Stack";
    case AutonRoutine::RedMidfieldPressure:
        return "Red Midfield";
    case AutonRoutine::BlueMidfieldPressure:
        return "Blue Midfield";
    case AutonRoutine::SkillsRoute:
        return "Skills";
    case AutonRoutine::DoNothing:
        return "Do Nothing";
    }
    return "Unknown";
}

AutonSelector::AutonSelector(AutonActions& actions, pros::Controller& controller, util::Logger& logger)
    : actions(actions), controller(controller), logger(logger) {}

void AutonSelector::initialize() {
    selectedRoutine = AutonRoutine::DoNothing;
    controller.set_text(1, 0, "Auton: Do Nothing");
}

void AutonSelector::update() {
    // TODO: replace this with a brain-screen selector when routine testing gets serious.
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        setSelected(AutonRoutine::RedSafeAWP);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        setSelected(AutonRoutine::BlueSafeAWP);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        setSelected(AutonRoutine::SkillsRoute);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        setSelected(AutonRoutine::DoNothing);
    }
}

void AutonSelector::runSelected() {
    logger.info(std::string("Running auton: ") + toString(selectedRoutine));

    switch (selectedRoutine) {
    case AutonRoutine::RedSafeAWP:
        redSafeAWP(actions);
        break;
    case AutonRoutine::BlueSafeAWP:
        blueSafeAWP(actions);
        break;
    case AutonRoutine::RedGoalStackPriority:
        redGoalStackPriority(actions);
        break;
    case AutonRoutine::BlueGoalStackPriority:
        blueGoalStackPriority(actions);
        break;
    case AutonRoutine::RedMidfieldPressure:
        redMidfieldPressure(actions);
        break;
    case AutonRoutine::BlueMidfieldPressure:
        blueMidfieldPressure(actions);
        break;
    case AutonRoutine::SkillsRoute:
        skillsRoute(actions);
        break;
    case AutonRoutine::DoNothing:
        doNothing(actions);
        break;
    }
}

void AutonSelector::setSelected(AutonRoutine routine) {
    selectedRoutine = routine;
    logger.info(std::string("Selected auton: ") + toString(routine));
}

AutonRoutine AutonSelector::selected() const {
    return selectedRoutine;
}

} // namespace auton
