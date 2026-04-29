#include "tuning/pid_tuning.hpp"
#include "util/logger.hpp"

namespace tuning {

PidTuning::PidTuning(util::Logger& logger) : logger(logger) {}

void PidTuning::explainWorkflow() const {
    logger.info("PID: tune kP and kD first; keep kI off unless needed");
    logger.info("Lateral slew reduces slipping/tipping; timeouts prevent stuck auton");
    logger.info("Small/large error ranges decide when LemLib exits a motion");
}

} // namespace tuning
