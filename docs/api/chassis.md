# Chassis

`#include "odyssey/api.hpp"` — everything lives in the `odyssey` namespace.

Use this page when you need an exact signature or default. For a guided route,
start with [Configuration](../getting-started/configuration.md) and
[Your First Autonomous](../getting-started/first-autonomous.md).

## Construction

```cpp
Chassis(Drivetrain drivetrain, ControllerSettings lateralSettings,
        ControllerSettings angularSettings, OdomSensors sensors,
        DriveCurve* throttleCurve = nullptr, DriveCurve* steerCurve = nullptr);
```

### `Drivetrain`

| Field | Type | Description |
| ----- | ---- | ----------- |
| `leftMotors` | `pros::MotorGroup*` | left side motors |
| `rightMotors` | `pros::MotorGroup*` | right side motors |
| `trackWidth` | `float` | distance between left and right wheels, inches |
| `wheelDiameter` | `float` | drive wheel diameter, inches |
| `rpm` | `float` | wheel rpm after external gearing |
| `horizontalDrift` | `float` | cornering aggression for `moveToPose`. 2 = all omni (default), 8 = center traction |

### `ControllerSettings`

| Field | Type | Description |
| ----- | ---- | ----------- |
| `kP`, `kI`, `kD` | `float` | PID gains (error in inches / degrees, output ±127) |
| `windupRange` | `float` | integral accumulates only inside this error range. 0 = disabled |
| `smallError`, `smallErrorTimeout` | `float`, `int` | tight settle window + ms |
| `largeError`, `largeErrorTimeout` | `float`, `int` | loose bail-out window + ms |
| `slew` | `float` | max output change per 10 ms tick. 0 = disabled |

## Lifecycle

### `calibrate`

```cpp
void calibrate(bool calibrateImu = true);
```

Call once in `initialize()`. Calibrates the IMU (up to 3 attempts, ~3 s; the
controller rumbles `---` and the IMU is dropped if all fail), substitutes
drive encoders for missing vertical tracking wheels, zeroes all sensors, and
starts the 10 ms background tracking task.

### `setPose` / `getPose`

```cpp
void setPose(float x, float y, float theta, bool radians = false);
void setPose(Pose pose, bool radians = false);
Pose getPose(bool radians = false) const;
```

`getPose` returns raw odometry unless optional MCL correction mode is enabled
and its confidence gates pass. `getOdometryPose()` always exposes the unchanged
raw pose; see the [Localization API](localization.md).

Inches; compass degrees by default, standard math radians when
`radians = true`.

## Motions

All motions share these conventions:

- `timeout` (ms) is a hard cap on how long the motion may run.
- `async = true` (default): the call returns immediately. A second motion
  call queues behind the running one.
- `minSpeed` + `earlyExitRange` enable motion chaining: the motion exits
  while still moving once the error is inside `earlyExitRange`.

```cpp
void turnToHeading(float theta, int timeout, TurnToHeadingParams params = {}, bool async = true);
void turnToPoint(float x, float y, int timeout, TurnToPointParams params = {}, bool async = true);
void swingToHeading(float theta, DriveSide lockedSide, int timeout,
                    SwingToHeadingParams params = {}, bool async = true);
void moveToPoint(float x, float y, int timeout, MoveToPointParams params = {}, bool async = true);
void moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams params = {}, bool async = true);
void follow(const std::vector<Waypoint>& path, float lookahead, int timeout,
            bool forwards = true, bool async = true);
void follow(const std::string& fileName, float lookahead, int timeout,
            bool forwards = true, bool async = true);
```

### Parameter structs

All fields are optional (designated initializers):

```cpp
struct TurnToHeadingParams {
    AngularDirection direction = AngularDirection::AUTO; // force CW / CCW
    float maxSpeed = 127;
    float minSpeed = 0;
    float earlyExitRange = 0; // degrees
};

struct TurnToPointParams {
    bool forwards = true; // point the front or the back at the target
    AngularDirection direction = AngularDirection::AUTO;
    float maxSpeed = 127, minSpeed = 0, earlyExitRange = 0;
};

struct SwingToHeadingParams {
    AngularDirection direction = AngularDirection::AUTO;
    float maxSpeed = 127, minSpeed = 0, earlyExitRange = 0;
};

struct MoveToPointParams {
    bool forwards = true; // drive forwards or backwards to the point
    float maxSpeed = 127, minSpeed = 0;
    float earlyExitRange = 0; // inches
};

struct MoveToPoseParams {
    bool forwards = true;
    float lead = 0.6; // boomerang aggressiveness, 0-1. Bigger = wider arc
    float maxSpeed = 127, minSpeed = 0;
    float earlyExitRange = 0; // inches
};
```

### `Waypoint`

```cpp
struct Waypoint : public Pose {
    Waypoint(float x = 0, float y = 0, float speed = 0);
    float speed; // target speed at this point, -127 to 127
};
```

The file overload of `follow` reads
[path.jerryio](https://path.jerryio.com/) format from the micro SD card:
`x, y, speed` per line, terminated by `endData`.

## Motion management

```cpp
void waitUntil(float dist);  // block until the motion travels dist (in / deg)
void waitUntilDone();        // block until the motion finishes
void cancelMotion();         // stop the current motion; queued motions run
void cancelAllMotions();     // stop everything
bool isInMotion() const;
```

## Driver control

```cpp
void tank(int left, int right, bool disableDriveCurve = false);
void arcade(int throttle, int turn, bool disableDriveCurve = false);
void curvature(int throttle, int turn, bool disableDriveCurve = false);
void setBrakeMode(pros::motor_brake_mode_e mode);
```

Inputs are joystick values (-127 to 127). Positive `turn` turns right.
`curvature` makes the turn input bend the path instead of setting a turn
rate, which feels consistent at any speed; at zero throttle it pivots in
place.

## Related guides

- [Tune the PID controllers](../tutorials/tuning.md)
- [Create and follow a pure pursuit path](../tutorials/pure-pursuit.md)
- [Shape driver-control input](../tutorials/driver-control.md)
- [Understand pose tracking](../explanation/odometry-math.md)
