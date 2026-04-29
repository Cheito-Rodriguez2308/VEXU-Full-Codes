#pragma once

#include "control/driver_profiles.hpp"

namespace control {

class InputCurve {
  public:
    explicit InputCurve(const DriverProfile& profile);

    int throttle(int raw) const;
    int steer(int raw) const;
    int limit(int raw, bool precision) const;

  private:
    DriverProfile profile;
};

} // namespace control
