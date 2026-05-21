#pragma once

namespace config {

/**
 * @brief Loop timing used by PROS tasks.
 * Change these only if updates feel slow or the brain screen gets noisy.
 */
namespace timing {
constexpr int driverLoopMs = 20;
constexpr int dashboardLoopMs = 100;
constexpr int defaultMotionTimeoutMs = 2500;
}

/**
 * @brief Warnings shown to drivers during testing.
 * Retest these thresholds once the robot has its final drive motors.
 */
namespace safety {
constexpr double hotMotorCelsius = 55.0; // TODO: motor temperature warning.
constexpr double lowBatteryVolts = 11.8; // TODO: battery warning threshold.
}

/**
 * @brief Sensor thresholds used for possession estimates.
 * These should be measured with real game objects before matches.
 */
namespace sensors {
constexpr int opticalLedPercent = 50; // TODO: optical brightness for bench testing.
constexpr int possessionDistanceMm = 80; // TODO: distance threshold for possession estimates.
}

/**
 * @brief Normal mechanism voltages.
 * Keep these conservative until the mechanisms are tested under load.
 */
namespace mechanism {
constexpr int intakeVoltage = 9000; // TODO: intake voltage.
constexpr int outtakeVoltage = -9000; // TODO: outtake voltage.
constexpr int holdVoltage = 1500; // TODO: possession hold voltage.
constexpr int grabVoltage = 6000; // TODO: manipulator grab voltage.
constexpr int scoreVoltage = 8000; // TODO: manipulator score voltage.
constexpr int releaseVoltage = -4000; // TODO: manipulator release voltage.
constexpr int toggleVoltage = 5000; // TODO: toggle mechanism voltage.
}

/**
 * @brief Repeatable tuning targets.
 * These are simple tests, not autonomous route coordinates.
 */
namespace tuning {
constexpr double defaultLookahead = 12.0; // TODO: Pure Pursuit lookahead.
constexpr int mechanismTestVoltage = 6000; // TODO: safe bench-test voltage.
constexpr double angularQuarterTurnDeg = 90.0; // TODO: angular PID test target.
constexpr double angularHalfTurnDeg = 180.0; // TODO: angular PID test target.
constexpr double forwardShortInches = 24.0; // TODO: lateral PID short test.
constexpr double forwardLongInches = 48.0; // TODO: lateral PID long test.
constexpr double moveToPointTestX = 24.0; // TODO: motion test coordinate.
constexpr double moveToPointTestY = 24.0; // TODO: motion test coordinate.
constexpr double moveToPoseHeadingDeg = 90.0; // TODO: final heading test.
constexpr double swingTurnHeadingDeg = 45.0; // TODO: swing turn target.
constexpr int angularTimeoutMs = 2500; // TODO: angular test timeout.
constexpr int lateralTimeoutMs = 3500; // TODO: lateral test timeout.
}

} // namespace config
