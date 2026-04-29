#pragma once

#include <cstdint>

namespace util {

class Timer {
  public:
    void reset();
    std::uint32_t elapsedMs() const;
    bool done(std::uint32_t timeoutMs) const;

  private:
    std::uint32_t startMs = 0;
};

} // namespace util
