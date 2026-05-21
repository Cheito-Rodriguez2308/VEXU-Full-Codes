#include "subsystems/sensors.hpp"
#include "config/constants.hpp"

namespace subsystems {

Sensors::Sensors(std::int8_t opticalPort, std::int8_t distancePort, util::Logger& logger)
    : optical(opticalPort), distance(distancePort), logger(logger) {}

void Sensors::initialize() {
    optical.set_led_pwm(config::sensors::opticalLedPercent);
    logger.info("Sensors initialized with TODO_PLACEHOLDER ports");
}

void Sensors::update() {
    // TODO_PLACEHOLDER possession estimation. Replace with real optical/distance/rotation conditions.
    if (!possessionState.manualOverride) {
        const int objectDistance = distance.get();
        if (objectDistance > 0 && objectDistance < config::sensors::possessionDistanceMm) possessionState.hasCup = true;
    }
}

void Sensors::stop() {}
void Sensors::debug() const { logger.info(possessionState.hasPin ? "hasPin=true" : "hasPin=false"); }
PossessionState& Sensors::possession() { return possessionState; }
const PossessionState& Sensors::possession() const { return possessionState; }
void Sensors::setHasPin(bool value) { possessionState.hasPin = value; }
void Sensors::setHasCup(bool value) { possessionState.hasCup = value; }
void Sensors::setManualOverride(bool value) { possessionState.manualOverride = value; }

} // namespace subsystems
