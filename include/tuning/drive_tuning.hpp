#pragma once

// DriveTuning contains repeatable tests, not autonomous routes.
// Run one test at a time on blocks first, then on the field.

namespace subsystems {
class Drivetrain;
}

namespace util {
class Logger;
}

namespace tuning {

class DriveTuning {
  public:
    DriveTuning(subsystems::Drivetrain& drivetrain, util::Logger& logger);
    void angular90();
    void angular180();
    void forward24();
    void forward48();
    void moveToPointTest();
    void moveToPoseTest();
    void swingTurnTest();
    void purePursuitTest();
    void motionChainingTest();

  private:
    subsystems::Drivetrain& drivetrain;
    util::Logger& logger;
};

} // namespace tuning
