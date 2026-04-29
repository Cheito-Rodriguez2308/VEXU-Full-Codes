#include "control/input_curve.hpp"
#include "lemlib/driveCurve.hpp"
#include "util/math_utils.hpp"

namespace control {

InputCurve::InputCurve(const DriverProfile& profile) : profile(profile) {}

int InputCurve::throttle(int raw) const {
    lemlib::ExpoDriveCurve curve(profile.deadband, profile.throttleMinOutput, profile.throttleCurveGain);
    return static_cast<int>(curve.curve(raw));
}

int InputCurve::steer(int raw) const {
    lemlib::ExpoDriveCurve curve(profile.deadband, profile.steerMinOutput, profile.steerCurveGain);
    return static_cast<int>(curve.curve(raw));
}

int InputCurve::limit(int raw, bool precision) const {
    const double speedScale = precision ? profile.precisionSpeedScale : 1.0;
    return util::scaleSpeed(raw, speedScale * profile.maxSpeed / 127.0);
}

} // namespace control
