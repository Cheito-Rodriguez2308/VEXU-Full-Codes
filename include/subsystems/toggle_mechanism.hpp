#pragma once

// ToggleMechanism models color/field-object toggling as named states.
// State names can change after we validate the real mechanism.

#include "pros/motors.hpp"
#include "util/logger.hpp"
#include <memory>

namespace subsystems {

enum class ToggleMechanismState {
    Idle,
    SetRed,
    SetBlue,
    SetYellow,
    Retract
};

const char* toString(ToggleMechanismState state);

class ToggleMechanism {
  public:
    ToggleMechanism(std::int8_t motorPort, util::Logger& logger);
    void initialize();
    void update();
    void stop();
    void setState(ToggleMechanismState nextState);
    ToggleMechanismState getState() const;
    void debug() const;

  private:
    std::unique_ptr<pros::Motor> motor;
    util::Logger& logger;
    ToggleMechanismState state = ToggleMechanismState::Idle;
};

} // namespace subsystems
