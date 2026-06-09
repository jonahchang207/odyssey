#include <algorithm>
#include <cmath>

#include "odyssey/chassis.hpp"
#include "pros/rtos.hpp"

// ----------------------------------------------------------------------------
// PID motions: turns, swings, moveToPoint and moveToPose (boomerang).
//
// Sign conventions:
//  - turn/swing functions work in the compass frame (degrees, clockwise
//    positive), so a positive angular output drives the left side forward
//  - drive motions work in the math frame (radians, counterclockwise
//    positive), so a positive angular output drives the right side faster
// ----------------------------------------------------------------------------

namespace odyssey {

// distance from the target where moveToPoint/moveToPose switch into the
// "close" state: heading correction is relaxed and speed is no longer
// allowed to increase, which prevents spinning near the target
static constexpr float CLOSE_RANGE = 7.5; // inches
// minimum speed cap while close, so the robot can still settle
static constexpr float CLOSE_MIN_CAP = 60;

void Chassis::turnToHeading(float theta, int timeout, TurnToHeadingParams params, bool async) {
    requestMotionStart();
    if (!motionRunning) return;
    // re-call this function in a new task if the motion is async
    if (async) {
        pros::Task task([=, this]() { turnToHeading(theta, timeout, params, false); });
        endMotion();
        pros::delay(10);
        return;
    }

    angularPID.reset();
    angularSmallExit.reset();
    angularLargeExit.reset();

    const float targetTheta = sanitizeAngle(theta, false);
    const float startTheta = getPose().theta;
    const int startTime = pros::millis();
    AngularDirection direction = params.direction;
    distTraveled = 0;

    while (pros::millis() - startTime < timeout && !angularSmallExit.getExit() &&
           !angularLargeExit.getExit() && motionRunning) {
        const float currentTheta = getPose().theta; // compass degrees
        distTraveled = std::fabs(angleError(currentTheta, startTheta, false));

        float error = angleError(targetTheta, currentTheta, false, direction);
        // once a forced-direction turn gets near the target, release the
        // direction lock so an overshoot doesn't cause a full extra rotation
        if (direction != AngularDirection::AUTO && std::fabs(error) < 45) {
            direction = AngularDirection::AUTO;
        }

        angularSmallExit.update(error);
        angularLargeExit.update(error);

        float power = angularPID.update(error);
        power = std::clamp(power, -params.maxSpeed, params.maxSpeed);
        if (params.minSpeed != 0 && std::fabs(power) < params.minSpeed)
            power = params.minSpeed * sgn(power);
        if (params.minSpeed != 0 && std::fabs(error) < params.earlyExitRange) break;

        // positive error = clockwise = left forward, right backward
        drivetrain.leftMotors->move(power);
        drivetrain.rightMotors->move(-power);
        pros::delay(10);
    }

    stopDrive();
    distTraveled = -1;
    endMotion();
}

void Chassis::turnToPoint(float x, float y, int timeout, TurnToPointParams params, bool async) {
    requestMotionStart();
    if (!motionRunning) return;
    if (async) {
        pros::Task task([=, this]() { turnToPoint(x, y, timeout, params, false); });
        endMotion();
        pros::delay(10);
        return;
    }

    angularPID.reset();
    angularSmallExit.reset();
    angularLargeExit.reset();

    const Pose target(x, y);
    const float startTheta = getPose().theta;
    const int startTime = pros::millis();
    AngularDirection direction = params.direction;
    distTraveled = 0;

    while (pros::millis() - startTime < timeout && !angularSmallExit.getExit() &&
           !angularLargeExit.getExit() && motionRunning) {
        const Pose pose = getPose(true); // math radians
        distTraveled = std::fabs(angleError(getPose().theta, startTheta, false));

        // recompute the target heading every tick in case the robot drifts
        float targetTheta = 90.0f - radToDeg(pose.angle(target));
        if (!params.forwards) targetTheta += 180;

        float error = angleError(targetTheta, getPose().theta, false, direction);
        if (direction != AngularDirection::AUTO && std::fabs(error) < 45) {
            direction = AngularDirection::AUTO;
        }

        angularSmallExit.update(error);
        angularLargeExit.update(error);

        float power = angularPID.update(error);
        power = std::clamp(power, -params.maxSpeed, params.maxSpeed);
        if (params.minSpeed != 0 && std::fabs(power) < params.minSpeed)
            power = params.minSpeed * sgn(power);
        if (params.minSpeed != 0 && std::fabs(error) < params.earlyExitRange) break;

        drivetrain.leftMotors->move(power);
        drivetrain.rightMotors->move(-power);
        pros::delay(10);
    }

    stopDrive();
    distTraveled = -1;
    endMotion();
}

void Chassis::swingToHeading(float theta, DriveSide lockedSide, int timeout,
                             SwingToHeadingParams params, bool async) {
    requestMotionStart();
    if (!motionRunning) return;
    if (async) {
        pros::Task task([=, this]() { swingToHeading(theta, lockedSide, timeout, params, false); });
        endMotion();
        pros::delay(10);
        return;
    }

    angularPID.reset();
    angularSmallExit.reset();
    angularLargeExit.reset();

    const float targetTheta = sanitizeAngle(theta, false);
    const float startTheta = getPose().theta;
    const int startTime = pros::millis();
    AngularDirection direction = params.direction;
    distTraveled = 0;

    // hold the locked side still while the other side swings
    const pros::v5::MotorBrake prevMode = lockedSide == DriveSide::LEFT
                                              ? drivetrain.leftMotors->get_brake_mode()
                                              : drivetrain.rightMotors->get_brake_mode();
    if (lockedSide == DriveSide::LEFT)
        drivetrain.leftMotors->set_brake_mode_all(pros::v5::MotorBrake::hold);
    else drivetrain.rightMotors->set_brake_mode_all(pros::v5::MotorBrake::hold);

    while (pros::millis() - startTime < timeout && !angularSmallExit.getExit() &&
           !angularLargeExit.getExit() && motionRunning) {
        const float currentTheta = getPose().theta;
        distTraveled = std::fabs(angleError(currentTheta, startTheta, false));

        float error = angleError(targetTheta, currentTheta, false, direction);
        if (direction != AngularDirection::AUTO && std::fabs(error) < 45) {
            direction = AngularDirection::AUTO;
        }

        angularSmallExit.update(error);
        angularLargeExit.update(error);

        float power = angularPID.update(error);
        power = std::clamp(power, -params.maxSpeed, params.maxSpeed);
        if (params.minSpeed != 0 && std::fabs(power) < params.minSpeed)
            power = params.minSpeed * sgn(power);
        if (params.minSpeed != 0 && std::fabs(error) < params.earlyExitRange) break;

        // positive error = clockwise. With the left side locked, clockwise
        // means the right side drives backward, and vice versa
        if (lockedSide == DriveSide::LEFT) {
            drivetrain.leftMotors->brake();
            drivetrain.rightMotors->move(-power);
        } else {
            drivetrain.rightMotors->brake();
            drivetrain.leftMotors->move(power);
        }
        pros::delay(10);
    }

    stopDrive();
    if (lockedSide == DriveSide::LEFT) drivetrain.leftMotors->set_brake_mode_all(prevMode);
    else drivetrain.rightMotors->set_brake_mode_all(prevMode);
    distTraveled = -1;
    endMotion();
}

void Chassis::moveToPoint(float x, float y, int timeout, MoveToPointParams params, bool async) {
    requestMotionStart();
    if (!motionRunning) return;
    if (async) {
        pros::Task task([=, this]() { moveToPoint(x, y, timeout, params, false); });
        endMotion();
        pros::delay(10);
        return;
    }

    lateralPID.reset();
    angularPID.reset();
    lateralSmallExit.reset();
    lateralLargeExit.reset();

    const Pose target(x, y);
    const int startTime = pros::millis();
    Pose lastPose = getPose();
    float maxSpeed = params.maxSpeed;
    float prevLateralOut = 0;
    bool close = false;
    distTraveled = 0;

    while (pros::millis() - startTime < timeout && !lateralSmallExit.getExit() &&
           !lateralLargeExit.getExit() && motionRunning) {
        const Pose pose = getPose(true); // math radians
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        const float distToTarget = pose.distance(target);
        if (!close && distToTarget < CLOSE_RANGE) {
            close = true;
            maxSpeed = std::fmax(std::fabs(prevLateralOut), CLOSE_MIN_CAP);
        }

        // heading error to the target, CCW positive. Flip the robot's
        // heading when driving backwards
        const float headingTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularError = angleError(pose.angle(target), headingTheta, true);

        // project the distance onto the robot's heading so driving past the
        // target produces a negative (corrective) error
        float lateralError = distToTarget * std::cos(angularError);
        if (!params.forwards) lateralError = -lateralError;

        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        float lateralOut = lateralPID.update(lateralError);
        lateralOut = std::clamp(lateralOut, -maxSpeed, maxSpeed);
        lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);
        if (!close && params.minSpeed != 0 && std::fabs(lateralOut) < params.minSpeed)
            lateralOut = params.minSpeed * sgn(lateralOut);
        prevLateralOut = lateralOut;

        if (params.minSpeed != 0 && distToTarget < params.earlyExitRange) break;

        // stop correcting heading when close so the robot doesn't spin on
        // top of the target point
        const float angularOut = close ? 0 : angularPID.update(radToDeg(angularError));

        // math frame: positive (CCW) angular error -> right side faster
        float leftPower = lateralOut - angularOut;
        float rightPower = lateralOut + angularOut;
        const float ratio = std::fmax(std::fabs(leftPower), std::fabs(rightPower)) / maxSpeed;
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

void Chassis::moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams params,
                         bool async) {
    requestMotionStart();
    if (!motionRunning) return;
    if (async) {
        pros::Task task([=, this]() { moveToPose(x, y, theta, timeout, params, false); });
        endMotion();
        pros::delay(10);
        return;
    }

    lateralPID.reset();
    angularPID.reset();
    lateralSmallExit.reset();
    lateralLargeExit.reset();

    // target heading in math radians. When driving backwards, flip both the
    // robot's heading and the target heading by 180 degrees so the math is
    // identical to the forwards case, then negate the output at the end
    float targetTheta = degToRad(90.0f - theta);
    if (!params.forwards) targetTheta += M_PI;
    const Pose target(x, y, targetTheta);

    const int startTime = pros::millis();
    Pose lastPose = getPose();
    float maxSpeed = params.maxSpeed;
    float prevLateralOut = 0;
    bool close = false;
    distTraveled = 0;

    while (pros::millis() - startTime < timeout && !lateralSmallExit.getExit() &&
           !lateralLargeExit.getExit() && motionRunning) {
        Pose pose = getPose(true);
        if (!params.forwards) pose.theta += M_PI;
        distTraveled += pose.distance(lastPose);
        lastPose = getPose(true);

        const float distToTarget = pose.distance(target);
        if (!close && distToTarget < CLOSE_RANGE) {
            close = true;
            maxSpeed = std::fmax(std::fabs(prevLateralOut), CLOSE_MIN_CAP);
        }

        // the carrot point sits behind the target along the target heading,
        // pulling the robot into an arc that arrives at the right angle.
        // Once close, chase the target itself
        const Pose carrot =
            close ? target
                  : target - Pose(std::cos(targetTheta), std::sin(targetTheta)) *
                                 (params.lead * distToTarget);

        // far away: aim at the carrot. Close: settle into the target heading
        const float angularError = close ? angleError(targetTheta, pose.theta, true)
                                         : angleError(pose.angle(carrot), pose.theta, true);
        const float lateralError =
            pose.distance(carrot) * std::cos(angleError(pose.angle(carrot), pose.theta, true));

        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        float lateralOut = lateralPID.update(lateralError);
        lateralOut = std::clamp(lateralOut, -maxSpeed, maxSpeed);
        lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

        // limit speed through tight arcs so the drivetrain doesn't drift
        if (!close) {
            const float curvature = std::fabs(getCurvature(pose, carrot));
            if (curvature != 0) {
                const float radius = 1.0f / curvature;
                const float maxSlipSpeed =
                    std::sqrt(drivetrain.horizontalDrift * radius * 9.8f);
                lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
            }
        }
        if (!close && params.minSpeed != 0 && std::fabs(lateralOut) < params.minSpeed)
            lateralOut = params.minSpeed * sgn(lateralOut);
        prevLateralOut = lateralOut;

        if (params.minSpeed != 0 && distToTarget < params.earlyExitRange) break;

        float angularOut = angularPID.update(radToDeg(angularError));
        angularOut = std::clamp(angularOut, -maxSpeed, maxSpeed);

        // undo the 180 degree flip for backwards motion
        const float lateralCmd = params.forwards ? lateralOut : -lateralOut;

        float leftPower = lateralCmd - angularOut;
        float rightPower = lateralCmd + angularOut;
        const float ratio = std::fmax(std::fabs(leftPower), std::fabs(rightPower)) / maxSpeed;
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

} // namespace odyssey
