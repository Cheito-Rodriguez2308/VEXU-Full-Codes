#include "core/match_state.hpp"

namespace core {

const char* toString(MatchMode mode) {
    switch (mode) {
    case MatchMode::Disabled: return "Disabled";
    case MatchMode::Initialize: return "Initialize";
    case MatchMode::CompetitionInitialize: return "Competition Init";
    case MatchMode::Autonomous: return "Autonomous 30s";
    case MatchMode::DriverControl: return "Driver 90s";
    case MatchMode::Tuning: return "Tuning";
    }
    return "Unknown";
}

void MatchState::setMode(MatchMode mode) {
    currentMode = mode;
}

MatchMode MatchState::mode() const {
    return currentMode;
}

} // namespace core
