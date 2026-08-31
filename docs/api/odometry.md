# Odometry

The tracking layer. Most teams only touch this through
`Chassis::setPose` / `Chassis::getPose`, but the free functions are public
for advanced use (custom motions, telemetry, position resets from wall
alignment, etc).

## `OdomSensors`

```cpp
struct OdomSensors {
    TrackingWheel* vertical1 = nullptr;   // forward-facing tracking wheel
    TrackingWheel* vertical2 = nullptr;
    TrackingWheel* horizontal1 = nullptr; // sideways-facing tracking wheel
    TrackingWheel* horizontal2 = nullptr;
    pros::Imu* imu = nullptr;
};
```

Any combination works. Heading priority: IMU → two parallel vertical wheels
→ two parallel horizontal wheels. Translation uses whatever wheels exist
(two of the same orientation are averaged). If the IMU errors mid-match the
wheel fallback takes over for that tick automatically.

## `TrackingWheel`

```cpp
TrackingWheel(pros::Rotation* encoder, float wheelDiameter, float offset, float gearRatio = 1);
TrackingWheel(pros::adi::Encoder* encoder, float wheelDiameter, float offset, float gearRatio = 1);
TrackingWheel(pros::MotorGroup* motors, float wheelDiameter, float offset, float driveRpm);
```

| Parameter | Description |
| --------- | ----------- |
| `wheelDiameter` | inches — see the `Omniwheel` constants |
| `offset` | signed distance from the tracking center, inches. Vertical: negative = left. Horizontal: negative = behind |
| `gearRatio` | wheel revolutions per sensor revolution (1 = direct) |
| `driveRpm` | (motor constructor) wheel rpm after external gearing |

```cpp
void reset();                 // zero the encoder
float getDistanceTraveled();  // inches since last reset
float getOffset() const;      // the configured offset
```

### `Omniwheel` constants

Measured real-world diameters of common omni wheels:

```cpp
namespace Omniwheel {
constexpr float NEW_2 = 2.125;
constexpr float NEW_275 = 2.75;
constexpr float OLD_275 = 2.75;
constexpr float NEW_325 = 3.25;
constexpr float NEW_4 = 4.0;
constexpr float OLD_4 = 4.18;
}
```

## Free functions

```cpp
void setSensors(OdomSensors sensors); // called by Chassis::calibrate
Pose getPose(bool radians = false);
OdometryState getOdometryState(); // math-radian pose + reset generation
void setPose(Pose pose, bool radians = false);
void update();        // one tracking step; the background task calls this
void initOdometry();  // reset baselines + start the 10 ms tracking task
```

`getPose`/`setPose` are thread-safe (the pose is guarded by a mutex shared
with the tracking task).

!!! example "Re-zeroing against a wall"
    A classic mid-auton accuracy trick: drive into a known wall, then
    overwrite the coordinate you now know exactly.

    ```cpp
    chassis.moveToPoint(0, -60, 2000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    odyssey::Pose p = chassis.getPose();
    chassis.setPose(p.x, -62.5, p.theta); // y is now exact
    ```

For the algorithm itself, see [The Odometry Math](../explanation/odometry-math.md).
