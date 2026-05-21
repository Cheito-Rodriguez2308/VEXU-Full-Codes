#pragma once

namespace util {
class Logger;
}

namespace tuning {

class PidTuning {
  public:
    explicit PidTuning(util::Logger& logger);
    void explainWorkflow() const;

  private:
    util::Logger& logger;
};

} // namespace tuning
