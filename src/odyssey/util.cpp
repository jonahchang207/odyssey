#include <cmath>

#include "odyssey/util.hpp"

namespace odyssey {

float radToDeg(float rad) { return rad * 180.0f / static_cast<float>(M_PI); }

float degToRad(float deg) { return deg * static_cast<float>(M_PI) / 180.0f; }

float sanitizeAngle(float angle, bool radians) {
    const float max = radians ? 2.0f * static_cast<float>(M_PI) : 360.0f;
    angle = std::fmod(angle, max);
    if (angle < 0) angle += max;
    return angle;
}

float angleError(float target, float position, bool radians, AngularDirection direction) {
    const float max = radians ? 2.0f * static_cast<float>(M_PI) : 360.0f;
    const float rawError = sanitizeAngle(target, radians) - sanitizeAngle(position, radians);
    switch (direction) {
        case AngularDirection::CW_CLOCKWISE:
            // force a positive (clockwise, in the compass frame) error
            return rawError < 0 ? rawError + max : rawError;
        case AngularDirection::CCW_COUNTERCLOCKWISE:
            // force a negative (counterclockwise, in the compass frame) error
            return rawError > 0 ? rawError - max : rawError;
        default:
            // shortest path
            return std::remainder(rawError, max);
    }
}

float avg(const std::vector<float>& values) {
    if (values.empty()) return 0;
    float sum = 0;
    for (const float value : values) sum += value;
    return sum / values.size();
}

float ema(float current, float previous, float smooth) {
    return current * smooth + previous * (1 - smooth);
}

float slew(float target, float current, float maxChange) {
    if (maxChange == 0) return target;
    float change = target - current;
    if (change > maxChange) change = maxChange;
    else if (change < -maxChange) change = -maxChange;
    return current + change;
}

float getCurvature(Pose pose, Pose other) {
    const float d = pose.distance(other);
    if (d == 0) return 0;
    // angle from the robot's heading to the chord, CCW positive
    const float beta = angleError(pose.angle(other), pose.theta, true);
    // curvature of the arc tangent to the heading through the other point
    return 2.0f * std::sin(beta) / d;
}

} // namespace odyssey
