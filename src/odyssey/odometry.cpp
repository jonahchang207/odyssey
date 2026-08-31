#include <cmath>
#include <vector>

#include "odyssey/odometry.hpp"
#include "odyssey/util.hpp"
#include "pros/rtos.hpp"

// ----------------------------------------------------------------------------
// Arc-based odometry, as described in the 5225A "Introduction to Position
// Tracking" document and used (in spirit) by LemLib.
//
// Internal conventions (see docs/explanation/odometry-math.md):
//  - theta is in standard math radians: 0 = +x, counterclockwise positive
//  - vertical wheel offsets:   negative = left of center, positive = right
//  - horizontal wheel offsets: negative = behind center,  positive = in front
//
// Each 10ms tick:
//  1. read sensor deltas
//  2. find the heading change (IMU preferred, wheel difference as fallback)
//  3. remove the rotation component from each wheel delta to get the robot's
//     local translation, modeled as an arc chord
//  4. rotate the local translation into the field frame at the average
//     heading of the tick and accumulate
// ----------------------------------------------------------------------------

namespace odyssey {

static OdomSensors odomSensors = {nullptr, nullptr, nullptr, nullptr, nullptr};
// default pose: origin, facing compass 0 (+y), which is pi/2 in math radians
static Pose odomPose(0, 0, M_PI_2);
static std::uint32_t resetGeneration = 0;

static float prevVertical1 = 0;
static float prevVertical2 = 0;
static float prevHorizontal1 = 0;
static float prevHorizontal2 = 0;
static float prevImuRotation = 0;

static pros::Task* trackingTask = nullptr;
static pros::Mutex poseMutex;

void setSensors(OdomSensors sensors) { odomSensors = sensors; }

Pose getPose(bool radians) {
    poseMutex.take(TIMEOUT_MAX);
    const Pose pose = odomPose;
    poseMutex.give();
    if (radians) return pose;
    return Pose(pose.x, pose.y, 90.0f - radToDeg(pose.theta));
}

OdometryState getOdometryState() {
    poseMutex.take(TIMEOUT_MAX);
    const OdometryState state{odomPose, resetGeneration};
    poseMutex.give();
    return state;
}

void setPose(Pose pose, bool radians) {
    const float theta = radians ? pose.theta : degToRad(90.0f - pose.theta);
    poseMutex.take(TIMEOUT_MAX);
    odomPose = Pose(pose.x, pose.y, theta);
    resetGeneration++;
    poseMutex.give();
}

/**
 * @brief Re-read all sensors so the next update starts from a clean baseline
 */
static void resetBaselines() {
    prevVertical1 = odomSensors.vertical1 ? odomSensors.vertical1->getDistanceTraveled() : 0;
    prevVertical2 = odomSensors.vertical2 ? odomSensors.vertical2->getDistanceTraveled() : 0;
    prevHorizontal1 = odomSensors.horizontal1 ? odomSensors.horizontal1->getDistanceTraveled() : 0;
    prevHorizontal2 = odomSensors.horizontal2 ? odomSensors.horizontal2->getDistanceTraveled() : 0;
    if (odomSensors.imu) {
        const double rotation = odomSensors.imu->get_rotation();
        prevImuRotation = std::isfinite(rotation) ? rotation : 0;
    }
}

void update() {
    // 1. read sensors and compute deltas
    const float vertical1 = odomSensors.vertical1 ? odomSensors.vertical1->getDistanceTraveled() : 0;
    const float vertical2 = odomSensors.vertical2 ? odomSensors.vertical2->getDistanceTraveled() : 0;
    const float horizontal1 =
        odomSensors.horizontal1 ? odomSensors.horizontal1->getDistanceTraveled() : 0;
    const float horizontal2 =
        odomSensors.horizontal2 ? odomSensors.horizontal2->getDistanceTraveled() : 0;

    const float deltaVertical1 = vertical1 - prevVertical1;
    const float deltaVertical2 = vertical2 - prevVertical2;
    const float deltaHorizontal1 = horizontal1 - prevHorizontal1;
    const float deltaHorizontal2 = horizontal2 - prevHorizontal2;
    prevVertical1 = vertical1;
    prevVertical2 = vertical2;
    prevHorizontal1 = horizontal1;
    prevHorizontal2 = horizontal2;

    // 2. heading change. IMU is preferred; if it is missing or returns an
    // error this tick, fall back to the difference between parallel wheels
    float deltaTheta = 0;
    bool headingFound = false;
    if (odomSensors.imu) {
        const double imuRotation = odomSensors.imu->get_rotation();
        if (std::isfinite(imuRotation)) {
            // IMU rotation is clockwise positive; internal theta is CCW positive
            deltaTheta = -degToRad(imuRotation - prevImuRotation);
            prevImuRotation = imuRotation;
            headingFound = true;
        }
    }
    if (!headingFound && odomSensors.vertical1 && odomSensors.vertical2) {
        const float offsetDiff =
            odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset();
        if (offsetDiff != 0) {
            deltaTheta = (deltaVertical1 - deltaVertical2) / offsetDiff;
            headingFound = true;
        }
    }
    if (!headingFound && odomSensors.horizontal1 && odomSensors.horizontal2) {
        const float offsetDiff =
            odomSensors.horizontal1->getOffset() - odomSensors.horizontal2->getOffset();
        if (offsetDiff != 0) {
            deltaTheta = (deltaHorizontal2 - deltaHorizontal1) / offsetDiff;
            headingFound = true;
        }
    }
    // if no heading source exists at all, deltaTheta stays 0 and tracking is
    // translation-only

    // 3. local translation arcs. A wheel's reading includes the distance it
    // swept while the robot rotated, so subtract the rotation component:
    //   forward arc  = deltaVertical   - offset * deltaTheta
    //   rightward arc = deltaHorizontal + offset * deltaTheta
    std::vector<float> forwardEstimates;
    if (odomSensors.vertical1)
        forwardEstimates.push_back(deltaVertical1 -
                                   odomSensors.vertical1->getOffset() * deltaTheta);
    if (odomSensors.vertical2)
        forwardEstimates.push_back(deltaVertical2 -
                                   odomSensors.vertical2->getOffset() * deltaTheta);
    const float forwardArc = avg(forwardEstimates);

    std::vector<float> rightwardEstimates;
    if (odomSensors.horizontal1)
        rightwardEstimates.push_back(deltaHorizontal1 +
                                     odomSensors.horizontal1->getOffset() * deltaTheta);
    if (odomSensors.horizontal2)
        rightwardEstimates.push_back(deltaHorizontal2 +
                                     odomSensors.horizontal2->getOffset() * deltaTheta);
    const float rightwardArc = avg(rightwardEstimates);

    // convert the arcs to straight-line chords
    float localY; // forward
    float localX; // rightward
    if (deltaTheta == 0) {
        localY = forwardArc;
        localX = rightwardArc;
    } else {
        const float chordFactor = 2.0f * std::sin(deltaTheta / 2.0f);
        localY = chordFactor * (forwardArc / deltaTheta);
        localX = chordFactor * (rightwardArc / deltaTheta);
    }

    // 4. rotate the local chord into the field frame at the tick's average
    // heading and accumulate
    poseMutex.take(TIMEOUT_MAX);
    const float avgTheta = odomPose.theta + deltaTheta / 2.0f;
    odomPose.x += localY * std::cos(avgTheta) + localX * std::sin(avgTheta);
    odomPose.y += localY * std::sin(avgTheta) - localX * std::cos(avgTheta);
    odomPose.theta += deltaTheta;
    poseMutex.give();
}

void initOdometry() {
    resetBaselines();
    if (trackingTask == nullptr) {
        trackingTask = new pros::Task([]() {
            while (true) {
                update();
                pros::delay(10);
            }
        });
    }
}

} // namespace odyssey
