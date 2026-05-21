#include "util/logger.hpp"
#include "pros/llemu.hpp"
#include <cstdio>

namespace util {

Logger::Logger(const char* name) : name(name) {}

void Logger::info(const std::string& message) const { write("INFO", message); }
void Logger::warn(const std::string& message) const { write("WARN", message); }
void Logger::error(const std::string& message) const { write("ERROR", message); }

void Logger::pose(const char* label, const lemlib::Pose& poseValue) const {
    char buffer[96];
    std::snprintf(buffer, sizeof(buffer), "%s pose x=%.1f y=%.1f h=%.1f", label, poseValue.x, poseValue.y, poseValue.theta);
    info(buffer);
}

void Logger::subsystemState(const char* subsystem, const char* state) const {
    info(std::string(subsystem) + " -> " + state);
}

void Logger::autonStep(const char* stepName) const {
    info(std::string("Auton step: ") + stepName);
}

void Logger::motionSample(const char* label, double positionError, double headingError, double velocity, double settleTimeMs) const {
    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "%s err=%.2f hErr=%.2f vel=%.1f settle=%.0fms", label, positionError, headingError,
                  velocity, settleTimeMs);
    info(buffer);
}

void Logger::write(const char* level, const std::string& message) const {
    std::printf("[%s][%s] %s\n", name, level, message.c_str());
    pros::lcd::print(7, "%s: %.32s", level, message.c_str());
}

} // namespace util
