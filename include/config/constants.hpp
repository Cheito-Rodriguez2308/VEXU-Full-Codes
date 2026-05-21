#pragma once

namespace config {

namespace timing {
constexpr int driverLoopMs = 20;
constexpr int dashboardLoopMs = 100;
constexpr int defaultMotionTimeoutMs = 2500;
}

namespace safety {
constexpr double hotMotorCelsius = 55.0; // TODO_PLACEHOLDER motor temperature warning.
constexpr double lowBatteryVolts = 11.8; // TODO_PLACEHOLDER battery warning threshold.
}

namespace sensors {
constexpr int opticalLedPercent = 50; // TODO_PLACEHOLDER optical brightness for bench testing.
constexpr int possessionDistanceMm = 80; // TODO_PLACEHOLDER distance threshold for possession estimates.
}

namespace mechanism {
constexpr int intakeVoltage = 9000; // TODO_PLACEHOLDER intake voltage.
constexpr int outtakeVoltage = -9000; // TODO_PLACEHOLDER outtake voltage.
constexpr int holdVoltage = 1500; // TODO_PLACEHOLDER possession hold voltage.
constexpr int grabVoltage = 6000; // TODO_PLACEHOLDER manipulator grab voltage.
constexpr int scoreVoltage = 8000; // TODO_PLACEHOLDER manipulator score voltage.
constexpr int releaseVoltage = -4000; // TODO_PLACEHOLDER manipulator release voltage.
constexpr int toggleVoltage = 5000; // TODO_PLACEHOLDER toggle mechanism voltage.
}

namespace tuning {
constexpr double defaultLookahead = 12.0; // TODO_PLACEHOLDER Pure Pursuit lookahead.
constexpr int mechanismTestVoltage = 6000; // TODO_PLACEHOLDER safe bench-test voltage.
constexpr double angularQuarterTurnDeg = 90.0; // TODO_PLACEHOLDER angular PID test target.
constexpr double angularHalfTurnDeg = 180.0; // TODO_PLACEHOLDER angular PID test target.
constexpr double forwardShortInches = 24.0; // TODO_PLACEHOLDER lateral PID short test.
constexpr double forwardLongInches = 48.0; // TODO_PLACEHOLDER lateral PID long test.
constexpr double moveToPointTestX = 24.0; // TODO_PLACEHOLDER motion test coordinate.
constexpr double moveToPointTestY = 24.0; // TODO_PLACEHOLDER motion test coordinate.
constexpr double moveToPoseHeadingDeg = 90.0; // TODO_PLACEHOLDER final heading test.
constexpr double swingTurnHeadingDeg = 45.0; // TODO_PLACEHOLDER swing turn target.
constexpr int angularTimeoutMs = 2500; // TODO_PLACEHOLDER angular test timeout.
constexpr int lateralTimeoutMs = 3500; // TODO_PLACEHOLDER lateral test timeout.
}

} // namespace config
