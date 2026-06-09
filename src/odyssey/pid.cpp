#include <cmath>

#include "odyssey/pid.hpp"
#include "odyssey/util.hpp"

namespace odyssey {

PID::PID(float kP, float kI, float kD, float windupRange, bool signFlipReset)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange),
      signFlipReset(signFlipReset) {}

float PID::update(float error) {
    integral += error;
    // kill the integral when the error crosses zero to prevent oscillation
    if (signFlipReset && sgn(error) != sgn(prevError)) integral = 0;
    // anti-windup: only integrate near the target
    if (windupRange != 0 && std::fabs(error) > windupRange) integral = 0;

    const float derivative = error - prevError;
    prevError = error;

    return error * kP + integral * kI + derivative * kD;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}

} // namespace odyssey
