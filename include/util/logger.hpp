#pragma once

#include "lemlib/pose.hpp"
#include <string>

namespace util {

class Logger {
  public:
    explicit Logger(const char* name = "Robot");

    void info(const std::string& message) const;
    void warn(const std::string& message) const;
    void error(const std::string& message) const;
    void pose(const char* label, const lemlib::Pose& pose) const;
    void subsystemState(const char* subsystem, const char* state) const;
    void autonStep(const char* stepName) const;
    void motionSample(const char* label,
                      double positionError,
                      double headingError,
                      double velocity,
                      double settleTimeMs) const;

  private:
    void write(const char* level, const std::string& message) const;

    const char* name;
};

} // namespace util
