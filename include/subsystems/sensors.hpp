#pragma once

// Sensors centralizes possession estimates and sensor debug helpers.
// Keep thresholds in config/constants.hpp so tuning is easy to find.

#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "util/logger.hpp"

namespace subsystems {

struct PossessionState {
    bool hasPin = false;
    bool hasCup = false;
    bool manualOverride = false;
};

class Sensors {
  public:
    Sensors(std::int8_t opticalPort, std::int8_t distancePort, util::Logger& logger);

    void initialize();
    void update();
    void stop();
    void debug() const;

    PossessionState& possession();
    const PossessionState& possession() const;
    void setHasPin(bool value);
    void setHasCup(bool value);
    void setManualOverride(bool value);

  private:
    pros::Optical optical;
    pros::Distance distance;
    util::Logger& logger;
    PossessionState possessionState;
};

} // namespace subsystems
