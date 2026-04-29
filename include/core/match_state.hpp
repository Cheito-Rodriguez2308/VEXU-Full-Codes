#pragma once

namespace core {

enum class MatchMode {
    Disabled,
    Initialize,
    CompetitionInitialize,
    Autonomous,
    DriverControl,
    Tuning
};

const char* toString(MatchMode mode);

class MatchState {
  public:
    void setMode(MatchMode mode);
    MatchMode mode() const;

  private:
    MatchMode currentMode = MatchMode::Initialize;
};

} // namespace core
