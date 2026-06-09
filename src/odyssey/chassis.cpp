#include <cmath>

#include "odyssey/chassis.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace odyssey {

// linear curves used when the user doesn't supply their own
static ExpoDriveCurve defaultThrottleCurve(0, 0, 1);
static ExpoDriveCurve defaultSteerCurve(0, 0, 1);

Chassis::Chassis(Drivetrain drivetrain, ControllerSettings lateralSettings,
                 ControllerSettings angularSettings, OdomSensors sensors,
                 DriveCurve* throttleCurve, DriveCurve* steerCurve)
    : drivetrain(drivetrain),
      lateralSettings(lateralSettings),
      angularSettings(angularSettings),
      sensors(sensors),
      lateralPID(lateralSettings.kP, lateralSettings.kI, lateralSettings.kD,
                 lateralSettings.windupRange, true),
      angularPID(angularSettings.kP, angularSettings.kI, angularSettings.kD,
                 angularSettings.windupRange, true),
      lateralSmallExit(lateralSettings.smallError, lateralSettings.smallErrorTimeout),
      lateralLargeExit(lateralSettings.largeError, lateralSettings.largeErrorTimeout),
      angularSmallExit(angularSettings.smallError, angularSettings.smallErrorTimeout),
      angularLargeExit(angularSettings.largeError, angularSettings.largeErrorTimeout),
      throttleCurve(throttleCurve ? throttleCurve : &defaultThrottleCurve),
      steerCurve(steerCurve ? steerCurve : &defaultSteerCurve) {
    if (this->drivetrain.horizontalDrift == 0) this->drivetrain.horizontalDrift = 2;
}

void Chassis::calibrate(bool calibrateImu) {
    if (calibrated) return;
    calibrated = true;

    // fall back to drive motor encoders if no vertical tracking wheels exist.
    // intentionally leaked: these live for the lifetime of the program
    if (sensors.vertical1 == nullptr && sensors.vertical2 == nullptr) {
        sensors.vertical1 = new TrackingWheel(drivetrain.leftMotors, drivetrain.wheelDiameter,
                                              -drivetrain.trackWidth / 2, drivetrain.rpm);
        sensors.vertical2 = new TrackingWheel(drivetrain.rightMotors, drivetrain.wheelDiameter,
                                              drivetrain.trackWidth / 2, drivetrain.rpm);
    }

    // calibrate the IMU, retrying up to 3 times
    if (sensors.imu != nullptr && calibrateImu) {
        int attempt = 1;
        bool success = false;
        while (attempt <= 3 && !success) {
            sensors.imu->reset(true); // blocks until calibration finishes
            success = std::isfinite(sensors.imu->get_rotation());
            if (!success) attempt++;
        }
        if (!success) {
            // warn the driver and fall back to wheel-only heading
            sensors.imu = nullptr;
            pros::Controller master(pros::E_CONTROLLER_MASTER);
            master.rumble("---");
        }
    }

    // zero everything and start the tracking task
    if (sensors.vertical1) sensors.vertical1->reset();
    if (sensors.vertical2) sensors.vertical2->reset();
    if (sensors.horizontal1) sensors.horizontal1->reset();
    if (sensors.horizontal2) sensors.horizontal2->reset();
    drivetrain.leftMotors->tare_position_all();
    drivetrain.rightMotors->tare_position_all();

    setSensors(sensors);
    odyssey::setPose(Pose(0, 0, 0), false);
    initOdometry();
}

void Chassis::setPose(float x, float y, float theta, bool radians) {
    odyssey::setPose(Pose(x, y, theta), radians);
}

void Chassis::setPose(Pose pose, bool radians) { odyssey::setPose(pose, radians); }

Pose Chassis::getPose(bool radians) const { return odyssey::getPose(radians); }

// ----------------------------------------------------------------------------
// motion management
// ----------------------------------------------------------------------------

void Chassis::requestMotionStart() {
    if (isInMotion()) motionQueued = true; // queue until the current motion ends
    else motionRunning = true;

    // blocks until the previous motion gives the mutex back
    motionMutex.take(TIMEOUT_MAX);

    motionRunning = true;
    motionQueued = false;
}

void Chassis::endMotion() {
    // if another motion is queued, it keeps the "running" flag alive
    motionRunning = motionQueued;
    motionMutex.give();
}

void Chassis::cancelMotion() {
    motionRunning = false;
    pros::delay(10); // give the motion loop a tick to notice and exit
}

void Chassis::cancelAllMotions() {
    motionRunning = false;
    motionQueued = false;
    pros::delay(10);
}

bool Chassis::isInMotion() const { return motionRunning; }

void Chassis::waitUntil(float dist) {
    // wait until the motion passes dist, or until it ends
    do pros::delay(10);
    while (distTraveled <= dist && distTraveled != -1);
}

void Chassis::waitUntilDone() {
    do pros::delay(10);
    while (distTraveled != -1);
}

void Chassis::stopDrive() {
    drivetrain.leftMotors->brake();
    drivetrain.rightMotors->brake();
}

void Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    drivetrain.leftMotors->set_brake_mode_all(mode);
    drivetrain.rightMotors->set_brake_mode_all(mode);
}

// ----------------------------------------------------------------------------
// driver control
// ----------------------------------------------------------------------------

void Chassis::tank(int left, int right, bool disableDriveCurve) {
    const float leftPower = disableDriveCurve ? left : throttleCurve->curve(left);
    const float rightPower = disableDriveCurve ? right : throttleCurve->curve(right);
    drivetrain.leftMotors->move(leftPower);
    drivetrain.rightMotors->move(rightPower);
}

void Chassis::arcade(int throttle, int turn, bool disableDriveCurve) {
    const float t = disableDriveCurve ? throttle : throttleCurve->curve(throttle);
    const float s = disableDriveCurve ? turn : steerCurve->curve(turn);

    float leftPower = t + s;
    float rightPower = t - s;
    // desaturate so turning authority is preserved at full throttle
    const float ratio = std::fmax(std::fabs(leftPower), std::fabs(rightPower)) / 127.0f;
    if (ratio > 1) {
        leftPower /= ratio;
        rightPower /= ratio;
    }
    drivetrain.leftMotors->move(leftPower);
    drivetrain.rightMotors->move(rightPower);
}

void Chassis::curvature(int throttle, int turn, bool disableDriveCurve) {
    const float t = disableDriveCurve ? throttle : throttleCurve->curve(throttle);
    const float s = disableDriveCurve ? turn : steerCurve->curve(turn);

    // at zero throttle, fall back to turning in place
    if (t == 0) {
        drivetrain.leftMotors->move(s);
        drivetrain.rightMotors->move(-s);
        return;
    }

    // the turn input bends the path instead of setting a turn rate, so the
    // feel of a given stick deflection is the same at any speed
    float leftPower = t + std::fabs(t) * s / 127.0f;
    float rightPower = t - std::fabs(t) * s / 127.0f;
    const float ratio = std::fmax(std::fabs(leftPower), std::fabs(rightPower)) / 127.0f;
    if (ratio > 1) {
        leftPower /= ratio;
        rightPower /= ratio;
    }
    drivetrain.leftMotors->move(leftPower);
    drivetrain.rightMotors->move(rightPower);
}

} // namespace odyssey
