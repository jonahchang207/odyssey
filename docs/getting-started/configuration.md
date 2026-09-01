# Configuration

All configuration happens at the top of `src/main.cpp` as global objects.
This page walks through each piece. The full example lives in
[`src/main.cpp`](https://github.com/jonahchang207/odyssey/blob/main/src/main.cpp).

!!! abstract "Before you begin"
    Complete [Installation](installation.md) and record the measurements from
    [Hardware Setup](hardware.md). Keep the robot on blocks while checking
    motor and encoder directions.

## 1. Motors and sensors

```cpp
#include "main.h"
#include "odyssey/api.hpp"

// negative port = reversed. Reverse one side so positive = forward on both
pros::MotorGroup leftMotors({-1, -2, -3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({4, 5, 6}, pros::MotorGearset::blue);

pros::Imu imu(10);

pros::Rotation verticalEncoder(11);
pros::Rotation horizontalEncoder(12);
```

## 2. Tracking wheels

```cpp
// TrackingWheel(encoder, wheelDiameter, offset, gearRatio = 1)
odyssey::TrackingWheel verticalWheel(&verticalEncoder, odyssey::Omniwheel::NEW_2, -1.25);
odyssey::TrackingWheel horizontalWheel(&horizontalEncoder, odyssey::Omniwheel::NEW_2, -2.5);
```

- `wheelDiameter` — inches. Use the `odyssey::Omniwheel` constants.
- `offset` — signed distance from the tracking center, inches.
  Vertical: negative = left. Horizontal: negative = behind.
  See [Hardware Setup](hardware.md#measurements-you-will-need).
- `gearRatio` — wheel revolutions per sensor revolution. `1` when the sensor
  is on the wheel axle.

## 3. Drivetrain

```cpp
odyssey::Drivetrain drivetrain{
    &leftMotors,
    &rightMotors,
    11.5,                        // track width, inches
    odyssey::Omniwheel::NEW_325, // drive wheel diameter, inches
    450,                         // drive rpm after external gearing
    2                            // horizontal drift
};
```

`horizontalDrift` controls how hard `moveToPose` may corner: `2` for an
all-omni drive, `8` if you have center traction wheels. Higher = faster
through arcs.

## 4. PID settings

Two controllers: **lateral** (driving, error in inches) and **angular**
(turning, error in degrees). These starting values work for most robots —
you will tune them in [Tuning the PIDs](../tutorials/tuning.md).

```cpp
odyssey::ControllerSettings lateralSettings{
    10,  // kP
    0,   // kI
    3,   // kD
    3,   // anti-windup range, inches
    1,   // small error range, inches
    100, // small error timeout, ms
    3,   // large error range, inches
    500, // large error timeout, ms
    20   // slew: max output change per 10 ms
};

odyssey::ControllerSettings angularSettings{
    2,   // kP
    0,   // kI
    10,  // kD
    3,   // anti-windup range, degrees
    1,   // small error range, degrees
    100, // small error timeout, ms
    3,   // large error range, degrees
    500, // large error timeout, ms
    0    // slew (usually 0 for turns)
};
```

!!! info "How exit conditions work"
    A motion ends when its error stays inside the **small** range for the
    small timeout (clean finish), *or* inside the **large** range for the
    large timeout (the "good enough, stop wasting time" exit), *or* when the
    motion's overall `timeout` expires (the safety net).

## 5. Odometry sensors

Any sensor you don't have is `nullptr`:

```cpp
odyssey::OdomSensors sensors{
    &verticalWheel,   // vertical tracking wheel 1
    nullptr,          // vertical tracking wheel 2
    &horizontalWheel, // horizontal tracking wheel 1
    nullptr,          // horizontal tracking wheel 2
    &imu              // inertial sensor
};
```

!!! tip "No tracking wheels? No problem"
    `OdomSensors sensors{nullptr, nullptr, nullptr, nullptr, &imu};` is
    valid — `calibrate()` substitutes the drive motor encoders for the
    vertical wheels automatically.

## 6. The chassis

```cpp
// optional driver-control input shaping
odyssey::ExpoDriveCurve throttleCurve(3, 10, 1.019);
odyssey::ExpoDriveCurve steerCurve(3, 10, 1.019);

odyssey::Chassis chassis(drivetrain, lateralSettings, angularSettings, sensors,
                         &throttleCurve, &steerCurve);
```

The drive curves are optional — omit the last two arguments for linear
control. See [Driver Control](../tutorials/driver-control.md).

## 7. Calibrate in `initialize()`

```cpp
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate(); // ~3 s: calibrates the IMU, starts odometry

    // optional: live pose readout on the brain screen
    pros::Task screenTask([&]() {
        while (true) {
            odyssey::Pose pose = chassis.getPose();
            pros::lcd::print(0, "X: %.2f in", pose.x);
            pros::lcd::print(1, "Y: %.2f in", pose.y);
            pros::lcd::print(2, "Heading: %.2f deg", pose.theta);
            pros::delay(50);
        }
    });
}
```

!!! warning
    Do **not** move the robot while `calibrate()` is running — the IMU is
    measuring its gyro bias. Place the robot on the field, then turn it on.

## Configuration sanity check

Before writing an autonomous routine, verify these three behaviors:

1. Positive motor output drives both sides forward.
2. Pushing the robot forward increases each vertical tracking wheel; pushing
   right increases each horizontal wheel.
3. After calibration, turning the robot clockwise increases
   `chassis.getPose().theta`.

If a sensor counts backward, reverse its port. Do not change an offset sign to
compensate for a reversed encoder.

<div class="odyssey-next" markdown>

<p><strong>Chassis calibrated?</strong><br>Learn the field frame before choosing your starting pose and targets.</p>

[Continue to the coordinate system →](coordinates.md)

</div>
