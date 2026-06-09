#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "odyssey/chassis.hpp"
#include "pros/rtos.hpp"

// ----------------------------------------------------------------------------
// Pure pursuit path following.
//
// Every tick the robot finds the point where a circle of radius `lookahead`
// centered on the robot intersects the path, then steers along the arc that
// passes through that point. Speed comes from the waypoint nearest the robot,
// so paths can slow the robot down for tight sections.
// ----------------------------------------------------------------------------

namespace odyssey {

/**
 * @brief index of the path point closest to the robot, never moving backwards
 * along the path so loops and self-crossing paths work
 */
static int findClosest(const Pose& pose, const std::vector<Waypoint>& path, int startIndex) {
    int closest = startIndex;
    float minDist = pose.distance(path.at(startIndex));
    for (int i = startIndex + 1; i < static_cast<int>(path.size()); i++) {
        const float dist = pose.distance(path.at(i));
        if (dist < minDist) {
            minDist = dist;
            closest = i;
        }
    }
    return closest;
}

/**
 * @brief where the lookahead circle intersects the segment p1 -> p2
 *
 * @return t in [0, 1] along the segment (the intersection furthest along
 *         the path), or -1 if there is no intersection
 */
static float circleIntersect(const Pose& p1, const Pose& p2, const Pose& center, float radius) {
    const Pose d = p2 - p1;
    const Pose f = p1 - center;
    const float a = d * d;
    const float b = 2.0f * (f * d);
    const float c = f * f - radius * radius;
    float discriminant = b * b - 4.0f * a * c;

    if (discriminant >= 0 && a != 0) {
        discriminant = std::sqrt(discriminant);
        const float t1 = (-b - discriminant) / (2.0f * a);
        const float t2 = (-b + discriminant) / (2.0f * a);
        // prefer the intersection furthest along the segment
        if (t2 >= 0 && t2 <= 1) return t2;
        if (t1 >= 0 && t1 <= 1) return t1;
    }
    return -1;
}

void Chassis::follow(const std::vector<Waypoint>& path, float lookahead, int timeout,
                     bool forwards, bool async) {
    if (path.size() < 2) return;

    requestMotionStart();
    if (!motionRunning) return;
    if (async) {
        pros::Task task([=, this]() { follow(path, lookahead, timeout, forwards, false); });
        endMotion();
        pros::delay(10);
        return;
    }

    const int startTime = pros::millis();
    Pose lastPose = getPose();
    Waypoint lookaheadPoint = path.at(0);
    int lookaheadIndex = 0;
    int closestIndex = 0;
    distTraveled = 0;

    while (pros::millis() - startTime < timeout && motionRunning) {
        Pose pose = getPose(true); // math radians
        if (!forwards) pose.theta += M_PI;
        distTraveled += pose.distance(lastPose);
        lastPose = getPose(true);

        // end the motion once the nearest path point is the final one
        closestIndex = findClosest(pose, path, closestIndex);
        if (closestIndex == static_cast<int>(path.size()) - 1) break;

        // search forward along the path for the furthest lookahead
        // intersection. If the circle misses (sharp corner), keep the
        // previous lookahead point so the robot keeps moving
        for (int i = lookaheadIndex; i < static_cast<int>(path.size()) - 1; i++) {
            const float t = circleIntersect(path.at(i), path.at(i + 1), pose, lookahead);
            if (t != -1) {
                lookaheadPoint =
                    Waypoint(path.at(i).x + (path.at(i + 1).x - path.at(i).x) * t,
                             path.at(i).y + (path.at(i + 1).y - path.at(i).y) * t,
                             path.at(i + 1).speed);
                lookaheadIndex = i;
            }
        }

        // signed curvature of the arc to the lookahead point (CCW positive)
        const float curvature = getCurvature(pose, lookaheadPoint);

        // speed comes from the path, so deceleration is baked into the file
        float targetVel = path.at(closestIndex).speed;
        if (!forwards) targetVel = -targetVel;

        // arc kinematics: v_left/right = v * (2 -/+ curvature * trackWidth) / 2
        float leftPower = targetVel * (2.0f - curvature * drivetrain.trackWidth) / 2.0f;
        float rightPower = targetVel * (2.0f + curvature * drivetrain.trackWidth) / 2.0f;
        const float ratio = std::fmax(std::fabs(leftPower), std::fabs(rightPower)) / 127.0f;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }
        drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);
        pros::delay(10);
    }

    stopDrive();
    distTraveled = -1;
    endMotion();
}

void Chassis::follow(const std::string& fileName, float lookahead, int timeout, bool forwards,
                     bool async) {
    // read a path.jerryio-format file from the micro SD card
    std::vector<Waypoint> path;
    std::ifstream file("/usd/" + fileName);
    if (!file) {
        printf("odyssey: could not open path file /usd/%s\n", fileName.c_str());
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line == "endData" || line == "endData\r") break;
        float x, y, speed;
        if (std::sscanf(line.c_str(), "%f, %f, %f", &x, &y, &speed) == 3) {
            path.push_back(Waypoint(x, y, speed));
        }
    }

    if (path.size() < 2) {
        printf("odyssey: path file /usd/%s has fewer than 2 points\n", fileName.c_str());
        return;
    }
    follow(path, lookahead, timeout, forwards, async);
}

} // namespace odyssey
