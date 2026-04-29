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
}

} // namespace config
