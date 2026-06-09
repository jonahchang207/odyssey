#include <cmath>

#include "odyssey/pose.hpp"

namespace odyssey {

Pose::Pose(float x, float y, float theta)
    : x(x),
      y(y),
      theta(theta) {}

Pose Pose::operator+(const Pose& other) const { return Pose(x + other.x, y + other.y, theta); }

Pose Pose::operator-(const Pose& other) const { return Pose(x - other.x, y - other.y, theta); }

float Pose::operator*(const Pose& other) const { return x * other.x + y * other.y; }

Pose Pose::operator*(float scalar) const { return Pose(x * scalar, y * scalar, theta); }

Pose Pose::operator/(float scalar) const { return Pose(x / scalar, y / scalar, theta); }

Pose Pose::lerp(Pose other, float t) const {
    return Pose(x + (other.x - x) * t, y + (other.y - y) * t, theta);
}

float Pose::distance(Pose other) const { return std::hypot(other.x - x, other.y - y); }

float Pose::angle(Pose other) const { return std::atan2(other.y - y, other.x - x); }

Pose Pose::rotate(float angle) const {
    const float cosA = std::cos(angle);
    const float sinA = std::sin(angle);
    return Pose(x * cosA - y * sinA, x * sinA + y * cosA, theta);
}

} // namespace odyssey
