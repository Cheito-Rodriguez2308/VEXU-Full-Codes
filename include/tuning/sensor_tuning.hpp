#pragma once

namespace subsystems { class Drivetrain; class Sensors; }
namespace util { class Logger; }

namespace tuning {

class SensorTuning {
  public:
    SensorTuning(subsystems::Drivetrain& drivetrain, subsystems::Sensors& sensors, util::Logger& logger);
    void trackingWheelDirectionTest();
    void mechanismSensorTest();

  private:
    subsystems::Drivetrain& drivetrain;
    subsystems::Sensors& sensors;
    util::Logger& logger;
};

} // namespace tuning
