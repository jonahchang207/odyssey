# Hardware Setup

Odometry is only as good as the sensors feeding it. This page covers what to
build and how to measure it.

## Recommended setups

In order of accuracy:

| Setup | Hardware | Accuracy |
| ----- | -------- | -------- |
| **1. Full tracking** | IMU + 1-2 vertical wheels + 1 horizontal wheel | Best — tracks sideways drift |
| **2. Standard** | IMU + 1 vertical tracking wheel | Great for most teams |
| **3. No extra hardware** | IMU only (drive encoders used automatically) | Good — degrades with wheel slip |
| **4. No IMU** | 2 parallel vertical tracking wheels | Good heading, needs careful build |

!!! note "What Odyssey does automatically"
    If you configure no vertical tracking wheels, `chassis.calibrate()`
    creates virtual ones from your drive motor encoders. If the IMU fails to
    calibrate three times in a row, the controller rumbles `---` and heading
    falls back to wheel-based tracking (which needs two parallel wheels —
    drive encoders count).

## Tracking wheel construction

- Use a **small omni wheel** (2" or 2.75") on a **V5 Rotation sensor** —
  rotation sensors are far more accurate than the old ADI shaft encoders.
- The wheel must be **spring- or rubber-band-loaded into the floor** so it
  never loses contact. A tracking wheel that bounces is worse than none.
- Mount it as **close to the center of the robot** as practical. Offsets
  are handled in software, but smaller offsets amplify measurement error
  less.
- Keep it **perfectly parallel** (vertical wheel) or **perpendicular**
  (horizontal wheel) to the drive direction. A 2° mounting skew causes ~3.5%
  systematic error.

### Vertical vs horizontal wheels

- A **vertical** wheel rolls when the robot drives forward/backward. It
  measures forward travel.
- A **horizontal** wheel rolls when the robot moves sideways (it is mounted
  90° to the drive wheels). It measures sideways drift — what happens when
  your robot gets bumped, arcs fast, or has worn omnis. If you skip it, the
  odometry assumes the robot never slides sideways.

## Measurements you will need

Measure these carefully — they go straight into your configuration:

1. **Wheel diameter.** Wheels are not the size on the box. Use the
   `odyssey::Omniwheel` constants (e.g. `NEW_275 = 2.75`, `NEW_2 = 2.125`),
   or better, measure: drive the robot exactly 100 inches and scale the
   diameter by `actual / reported`.
2. **Tracking wheel offsets.** The signed distance from the **tracking
   center** (the point midway between the drive wheels, on the centerline)
   to each tracking wheel:
    - vertical wheels: **negative = left** of center, **positive = right**
    - horizontal wheels: **negative = behind** center, **positive = in front**
3. **Track width.** Center of the left wheels to the center of the right
   wheels.
4. **Drive RPM.** The wheel rpm after external gearing. E.g. blue (600 rpm)
   motors with a 36:48 gear-down = `600 * 36 / 48 = 450`.

## Encoder direction check

Push the robot **forward**: every vertical tracking wheel must count **up**.
Push it **to the right**: every horizontal wheel must count **up**. If one
counts down, reverse it in the sensor constructor:

```cpp
pros::Rotation verticalEncoder(-11); // negative port = reversed
```
