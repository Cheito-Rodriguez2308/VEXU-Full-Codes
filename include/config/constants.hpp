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
constexpr double hotMotorCelsius = 55.0; // Warning threshold; adjust after repeated runs.
constexpr double lowBatteryVolts = 11.8; // Conservative alert point for older batteries.
}

// Possession estimates. Do not trust these until tested with real objects.
namespace sensors {
constexpr int opticalLedPercent = 50; // Bench-test brightness under field lighting.
constexpr int possessionDistanceMm = 80; // Retest once the sensor bracket is final.
}

// Mechanism voltages. Keep conservative until the intake has a full-load test.
namespace mechanism {
constexpr int intakeVoltage = 9000; // Kept for voltage-based helper code.
constexpr int outtakeVoltage = -9000; // Kept for voltage-based helper code.
constexpr int holdVoltage = 1500; // Gentle hold for mechanisms that still use motors.
constexpr int grabVoltage = 6000; // Conservative manipulator grab voltage.
constexpr int scoreVoltage = 8000; // Conservative manipulator score voltage.
constexpr int releaseVoltage = -4000; // Reverse voltage for motorized releases.
constexpr int toggleVoltage = 5000; // Toggle helper voltage for motorized versions.
}

// Tuning targets. These are test moves, not route coordinates.
namespace tuning {
constexpr double defaultLookahead = 12.0; // Starting lookahead for path tests.
constexpr int mechanismTestVoltage = 6000; // Safe bench-test voltage.
constexpr double angularQuarterTurnDeg = 90.0; // Angular PID test target.
constexpr double angularHalfTurnDeg = 180.0; // Angular PID test target.
constexpr double forwardShortInches = 24.0; // Lateral PID short test.
constexpr double forwardLongInches = 48.0; // Lateral PID long test.
constexpr double moveToPointTestX = 24.0; // Motion test coordinate.
constexpr double moveToPointTestY = 24.0; // Motion test coordinate.
constexpr double moveToPoseHeadingDeg = 90.0; // Final heading test.
constexpr double swingTurnHeadingDeg = 45.0; // Swing turn target.
constexpr int angularTimeoutMs = 2500; // Angular test timeout.
constexpr int lateralTimeoutMs = 3500; // Lateral test timeout.
}

} // namespace config
