#pragma once

namespace config {

// Loop timing. Driver loop matches the usual 20 ms control rhythm.
namespace timing {
constexpr int driverLoopMs = 20;
constexpr int dashboardLoopMs = 100;
constexpr int defaultMotionTimeoutMs = 2500;
}

// Driver warnings. Retest these once the final drive is installed.
namespace safety {
constexpr double hotMotorCelsius = 55.0; // TODO: warning threshold.
constexpr double lowBatteryVolts = 11.8; // TODO: adjust after battery testing.
}

// Possession estimates. Do not trust these until tested with real objects.
namespace sensors {
constexpr int opticalLedPercent = 50; // TODO: bench-test brightness.
constexpr int possessionDistanceMm = 80; // TODO: cup/pin distance threshold.
}

// Mechanism voltages. Keep conservative until the intake has a full-load test.
namespace mechanism {
constexpr int intakeVoltage = 9000; // TODO: intake voltage.
constexpr int outtakeVoltage = -9000; // TODO: outtake voltage.
constexpr int holdVoltage = 1500; // TODO: possession hold voltage.
constexpr int grabVoltage = 6000; // TODO: manipulator grab voltage.
constexpr int scoreVoltage = 8000; // TODO: manipulator score voltage.
constexpr int releaseVoltage = -4000; // TODO: manipulator release voltage.
constexpr int toggleVoltage = 5000; // TODO: toggle mechanism voltage.
}

// Tuning targets. These are test moves, not route coordinates.
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
