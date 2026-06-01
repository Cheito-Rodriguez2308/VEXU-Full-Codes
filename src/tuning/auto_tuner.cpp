#include "tuning/auto_tuner.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace tuning {
namespace {

constexpr double angularTargetDeg = 90.0;
constexpr double lateralTargetIn = 24.0;
constexpr int angularTimeoutMs = 2500;
constexpr int lateralTimeoutMs = 3000;
constexpr int sampleMs = 20;
constexpr int coolDownMs = 1000;
// Keep this low. AutoTuner runs on the floor, not on blocks.
constexpr int maxDriveMilliVolts = 5000;
constexpr int maxDriveOutput = maxDriveMilliVolts * 127 / 12000;
constexpr double angularSettleRangeDeg = 2.0;
constexpr double lateralSettleRangeIn = 1.0;
constexpr int settleHoldMs = 250;

double clamp(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(value, maxValue));
}

double signOf(double value) {
    if (value > 0.0) {
        return 1.0;
    }
    if (value < 0.0) {
        return -1.0;
    }
    return 0.0;
}

} // namespace

AutoTuner::AutoTuner(pros::Controller& controller,
                     subsystems::Drivetrain& drivetrain,
                     const config::RobotConfig& robotConfig,
                     util::Logger& logger)
    : controller(controller),
      drivetrain(drivetrain),
      robotConfig(robotConfig),
      logger(logger) {
    bestAngularResult.score = 999999.0;
    bestLateralResult.test = AutoTuneTest::Lateral;
    bestLateralResult.score = 999999.0;
}

void AutoTuner::runAngularTest() {
    logger.info("AutoTuner angular: X cancels");

    for (double pScale : {0.75, 1.0, 1.25}) {
        for (double dScale : {0.75, 1.0, 1.25}) {
            const AutoTuneGains gains = candidateFromBase(robotConfig.pid.angular, pScale, dScale);
            const AutoTuneResult result = runSingleTest(AutoTuneTest::Angular, gains);
            printResult("Angular", result);

            if (!result.canceled && result.score < bestAngularResult.score) {
                bestAngularResult = result;
            }
            if (result.canceled) {
                printResult("Angular best", bestAngularResult);
                return;
            }
            stopAndCoolDown();
        }
    }

    printResult("Angular best", bestAngularResult);
}

void AutoTuner::runLateralTest() {
    logger.info("AutoTuner lateral: X cancels");

    for (double pScale : {0.75, 1.0, 1.25}) {
        for (double dScale : {0.75, 1.0, 1.25}) {
            const AutoTuneGains gains = candidateFromBase(robotConfig.pid.lateral, pScale, dScale);
            const AutoTuneResult result = runSingleTest(AutoTuneTest::Lateral, gains);
            printResult("Lateral", result);

            if (!result.canceled && result.score < bestLateralResult.score) {
                bestLateralResult = result;
            }
            if (result.canceled) {
                printResult("Lateral best", bestLateralResult);
                return;
            }
            stopAndCoolDown();
        }
    }

    printResult("Lateral best", bestLateralResult);
}

const AutoTuneResult& AutoTuner::bestAngular() const {
    return bestAngularResult;
}

const AutoTuneResult& AutoTuner::bestLateral() const {
    return bestLateralResult;
}

AutoTuneResult AutoTuner::runSingleTest(AutoTuneTest test, const AutoTuneGains& gains) {
    AutoTuneResult result;
    result.test = test;
    result.gains = gains;

    drivetrain.cancelAllMotions();
    drivetrain.chassis().setPose(0, 0, 0);
    pros::delay(250);

    const double target = test == AutoTuneTest::Angular ? angularTargetDeg : lateralTargetIn;
    const double settleRange = test == AutoTuneTest::Angular ? angularSettleRangeDeg : lateralSettleRangeIn;
    const int timeout = test == AutoTuneTest::Angular ? angularTimeoutMs : lateralTimeoutMs;
    const std::uint32_t startMs = pros::millis();

    double previousError = target;
    double previousSign = signOf(previousError);
    double integral = 0.0;
    double maxPastTarget = 0.0;
    std::uint32_t settleStartMs = 0;

    while (pros::millis() - startMs < static_cast<std::uint32_t>(timeout)) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            result.canceled = true;
            break;
        }

        const lemlib::Pose pose = drivetrain.pose();
        const double measurement = test == AutoTuneTest::Angular ? pose.theta : pose.y;
        const double error = target - measurement;

        integral += error * (sampleMs / 1000.0);
        integral = clamp(integral, -50.0, 50.0);
        const double derivative = (error - previousError) / (sampleMs / 1000.0);
        double output = gains.kP * error + gains.kI * integral + gains.kD * derivative;
        output = clamp(output, -maxDriveOutput, maxDriveOutput);

        applyOutput(test, output);

        const double pastTarget = std::abs(measurement) - std::abs(target);
        if (pastTarget > maxPastTarget) {
            maxPastTarget = pastTarget;
        }

        const double currentSign = signOf(error);
        if (currentSign != 0.0 && previousSign != 0.0 && currentSign != previousSign) {
            result.oscillationCount++;
        }
        if (currentSign != 0.0) {
            previousSign = currentSign;
        }

        if (std::abs(error) < settleRange) {
            if (settleStartMs == 0) {
                settleStartMs = pros::millis();
            }
            if (pros::millis() - settleStartMs >= settleHoldMs) {
                break;
            }
        } else {
            settleStartMs = 0;
        }

        previousError = error;
        logger.motionSample(test == AutoTuneTest::Angular ? "Auto angular" : "Auto lateral",
                            std::abs(error),
                            test == AutoTuneTest::Angular ? error : 0.0,
                            output,
                            pros::millis() - startMs);
        pros::delay(sampleMs);
    }

    drivetrain.stop();

    const lemlib::Pose finalPose = drivetrain.pose();
    const double finalMeasurement = test == AutoTuneTest::Angular ? finalPose.theta : finalPose.y;
    result.finalError = std::abs(target - finalMeasurement);
    result.overshoot = maxPastTarget;
    result.settleTime = (pros::millis() - startMs) / 1000.0;
    result.score = result.finalError * 3.0 +
                   result.overshoot * 2.0 +
                   result.settleTime +
                   result.oscillationCount * 5.0;

    return result;
}

void AutoTuner::applyOutput(AutoTuneTest test, double output) {
    const int driveOutput = static_cast<int>(output);

    if (test == AutoTuneTest::Angular) {
        drivetrain.tank(-driveOutput, driveOutput);
        return;
    }

    drivetrain.tank(driveOutput, driveOutput);
}

void AutoTuner::stopAndCoolDown() {
    drivetrain.stop();
    pros::delay(coolDownMs);
}

void AutoTuner::printResult(const char* label, const AutoTuneResult& result) const {
    char line[96];
    std::snprintf(line,
                  sizeof(line),
                  "%s P%.2f I%.2f D%.2f score %.1f",
                  label,
                  result.gains.kP,
                  result.gains.kI,
                  result.gains.kD,
                  result.score);
    logger.info(line);

    std::snprintf(line,
                  sizeof(line),
                  "err %.2f over %.2f set %.1fs osc %d",
                  result.finalError,
                  result.overshoot,
                  result.settleTime,
                  result.oscillationCount);
    logger.info(line);

    pros::lcd::print(0, "%s", label);
    pros::lcd::print(1, "kP %.2f kI %.2f", result.gains.kP, result.gains.kI);
    pros::lcd::print(2, "kD %.2f score %.1f", result.gains.kD, result.score);
    pros::lcd::print(3, "err %.2f over %.2f", result.finalError, result.overshoot);
    pros::lcd::print(4, "settle %.1fs osc %d", result.settleTime, result.oscillationCount);
    pros::lcd::print(5, "Copy by hand; not saved");
}

AutoTuneGains AutoTuner::candidateFromBase(const lemlib::ControllerSettings& settings,
                                           double pScale,
                                           double dScale) const {
    return {
        settings.kP * pScale,
        settings.kI,
        settings.kD * dScale
    };
}

} // namespace tuning
