# Helpers

These small public types support chassis configuration and custom robot code.
For motion methods, see the [Chassis API](chassis.md); for a practical PID
tuning process, see [Tuning the PIDs](../tutorials/tuning.md).

The supporting classes — useful on their own for lifts, flywheels, and
custom motions.

## `Pose`

```cpp
class Pose {
    float x, y, theta;

    Pose(float x = 0, float y = 0, float theta = 0);

    Pose operator+(const Pose& other) const;  // x/y addition
    Pose operator-(const Pose& other) const;  // x/y subtraction
    float operator*(const Pose& other) const; // 2D dot product
    Pose operator*(float scalar) const;
    Pose operator/(float scalar) const;

    Pose lerp(Pose other, float t) const;  // interpolate, t in [0, 1]
    float distance(Pose other) const;      // inches
    float angle(Pose other) const;         // math radians to other
    Pose rotate(float angle) const;        // rotate about the origin
};
```

## `PID`

A general-purpose PID controller — use it for lifts and flywheels too.

```cpp
PID(float kP, float kI, float kD, float windupRange = 0, bool signFlipReset = false);

float update(float error); // error = target - current, returns output
void reset();              // call between movements
```

- `windupRange` — the integral only accumulates while `|error|` is inside
  this range, preventing huge integral buildup on long movements.
- `signFlipReset` — zero the integral when the error crosses 0, which kills
  integral-driven oscillation.

```cpp
odyssey::PID liftPID(2.5, 0, 8);

void liftTo(float target) {
    liftPID.reset();
    while (std::fabs(target - liftSensor.get_value()) > 5) {
        lift.move(liftPID.update(target - liftSensor.get_value()));
        pros::delay(10);
    }
    lift.brake();
}
```

## `ExitCondition`

Detects when a controller has settled: the input must stay inside `range`
for `time` ms continuously.

```cpp
ExitCondition(float range, int time);

bool update(float input); // feed the current error; returns done
bool getExit() const;
void reset();
```

## Drive curves

```cpp
class DriveCurve {                  // interface
    virtual float curve(float input) = 0;
};

class ExpoDriveCurve : public DriveCurve {
    ExpoDriveCurve(float deadband = 0, float minOutput = 0, float curve = 1);
};
```

See [Driver Control](../tutorials/driver-control.md) for tuning guidance.

## Utility functions

```cpp
template <typename T> int sgn(T value);     // -1, 0, or 1

float radToDeg(float rad);
float degToRad(float deg);

// wrap an angle into [0, 360) or [0, 2pi)
float sanitizeAngle(float angle, bool radians = true);

// shortest signed difference between two angles, or force a direction.
// compass frame: positive = clockwise. math frame: positive = counterclockwise
float angleError(float target, float position, bool radians = true,
                 AngularDirection direction = AngularDirection::AUTO);

float avg(const std::vector<float>& values);
float ema(float current, float previous, float smooth); // exponential moving average
float slew(float target, float current, float maxChange); // rate limiter

// signed curvature (1/in) of the arc from pose (tangent to its heading)
// through another point. Positive = curves left. theta in math radians
float getCurvature(Pose pose, Pose other);
```

```cpp
enum class AngularDirection { CW_CLOCKWISE, CCW_COUNTERCLOCKWISE, AUTO };
enum class DriveSide { LEFT, RIGHT };
```
