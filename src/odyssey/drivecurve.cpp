#include <cmath>

#include "odyssey/drivecurve.hpp"
#include "odyssey/util.hpp"

namespace odyssey {

ExpoDriveCurve::ExpoDriveCurve(float deadband, float minOutput, float curve)
    : deadband(deadband),
      minOutput(minOutput),
      curveGain(curve) {}

float ExpoDriveCurve::curve(float input) {
    // ignore input inside the deadzone
    if (std::fabs(input) <= deadband) return 0;
    // g(x) = |x| - deadband, remapped so g spans (0, 127 - deadband]
    const float g = std::fabs(input) - deadband;
    const float g127 = 127.0f - deadband;
    // exponential curve, normalized so full deflection still gives 127
    const float i = std::pow(curveGain, g - 127.0f) * g;
    const float i127 = std::pow(curveGain, g127 - 127.0f) * g127;
    return (127.0f - minOutput) / 127.0f * i * 127.0f / i127 * sgn(input) +
           minOutput * sgn(input);
}

} // namespace odyssey
