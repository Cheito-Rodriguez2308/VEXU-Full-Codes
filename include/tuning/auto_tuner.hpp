#pragma once

#include "config/robot_config.hpp"
#include "pros/misc.hpp"
#include "subsystems/drivetrain.hpp"
#include "util/logger.hpp"

namespace tuning {

enum class AutoTuneTest {
    Angular,
    Lateral
};

struct AutoTuneGains {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
};

struct AutoTuneResult {
    AutoTuneTest test = AutoTuneTest::Angular;
    AutoTuneGains gains;
    double finalError = 0.0;
    double overshoot = 0.0;
    double settleTime = 0.0;
    int oscillationCount = 0;
    double score = 0.0;
    bool canceled = false;
};

class AutoTuner {
  public:
    AutoTuner(pros::Controller& controller,
              subsystems::Drivetrain& drivetrain,
              const config::RobotConfig& robotConfig,
              util::Logger& logger);

    void runAngularTest();
    void runLateralTest();
    const AutoTuneResult& bestAngular() const;
    const AutoTuneResult& bestLateral() const;

  private:
    AutoTuneResult runSingleTest(AutoTuneTest test, const AutoTuneGains& gains);
    void applyOutput(AutoTuneTest test, double output);
    void stopAndCoolDown();
    void printResult(const char* label, const AutoTuneResult& result) const;
    AutoTuneGains candidateFromBase(const lemlib::ControllerSettings& settings,
                                    double pScale,
                                    double dScale) const;

    pros::Controller& controller;
    subsystems::Drivetrain& drivetrain;
    const config::RobotConfig& robotConfig;
    util::Logger& logger;

    AutoTuneResult bestAngularResult;
    AutoTuneResult bestLateralResult;
};

} // namespace tuning
