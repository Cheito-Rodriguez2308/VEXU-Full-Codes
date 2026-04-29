#include "tuning/sensor_tuning.hpp"
#include "subsystems/drivetrain.hpp"
#include "subsystems/sensors.hpp"
#include "util/logger.hpp"

namespace tuning {

SensorTuning::SensorTuning(subsystems::Drivetrain& drivetrain, subsystems::Sensors& sensors, util::Logger& logger)
    : drivetrain(drivetrain), sensors(sensors), logger(logger) {}

void SensorTuning::trackingWheelDirectionTest() {
    logger.info("Push forward: vertical wheel should increase. Push right: horizontal wheel should increase.");
    logger.info("TODO_PLACEHOLDER check offsets: vertical sign is left/right, horizontal sign is front/back");
}

void SensorTuning::mechanismSensorTest() {
    sensors.debug();
    logger.info("TODO_PLACEHOLDER verify optical/distance thresholds before trusting possession guards");
}

} // namespace tuning
