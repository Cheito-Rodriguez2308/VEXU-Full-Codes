#pragma once

#include "pros/misc.hpp"
#include "tuning/drive_tuning.hpp"
#include "tuning/pid_tuning.hpp"
#include "tuning/sensor_tuning.hpp"
#include "util/logger.hpp"

namespace tuning {

class TuningMenu {
  public:
    TuningMenu(pros::Controller& controller, DriveTuning& driveTuning, PidTuning& pidTuning,
               SensorTuning& sensorTuning, util::Logger& logger);
    void initialize();
    void update();
    bool enabled() const;
    void setEnabled(bool value);

  private:
    pros::Controller& controller;
    DriveTuning& driveTuning;
    PidTuning& pidTuning;
    SensorTuning& sensorTuning;
    util::Logger& logger;
    bool tuningEnabled = false;
};

} // namespace tuning
