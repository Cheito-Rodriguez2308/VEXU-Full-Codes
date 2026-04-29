#include "util/timer.hpp"
#include "pros/rtos.hpp"

namespace util {

void Timer::reset() {
    startMs = pros::millis();
}

std::uint32_t Timer::elapsedMs() const {
    return pros::millis() - startMs;
}

bool Timer::done(std::uint32_t timeoutMs) const {
    return elapsedMs() >= timeoutMs;
}

} // namespace util
