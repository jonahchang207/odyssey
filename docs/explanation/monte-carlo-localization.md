# Monte Carlo localization

Odyssey's Monte Carlo localization (MCL) is an optional global field-localization
layer. It does **not** replace the 10 ms tracking-wheel/IMU odometry loop. Raw
odometry supplies each MCL prediction, remains available through
`getOdometryPose()`, and is the driving pose whenever MCL is disabled, stale, or
not permitted to correct.

## Research conclusions

The implementation was derived from the algorithms and APIs below, not copied
from another VEX project.

1. The [VEX V5 Distance Sensor guide](https://kb.vex.com/hc/en-us/articles/360050696511-Using-the-V5-Distance-Sensor)
   specifies a narrow forward beam, a 20–2000 mm range, approximately ±15 mm
   accuracy below 200 mm, and approximately 5% accuracy above 200 mm. Those are
   hardware limits, not evidence that every detected object is a field wall.
2. The [PROS 4 Distance API](https://pros.cs.purdue.edu/v5/pros-4/classpros_1_1v5_1_1Distance.html)
   and the repository's vendored 4.2.2 header define millimeter readings,
   `9999` for no object, `PROS_ERR` for API/device failure, and confidence in
   the range 0–63. Confidence is not comparable in the same way below 200 mm.
3. The [PROS RTOS API](https://pros.cs.purdue.edu/v5/pros-4/classpros_1_1rtos_1_1Task.html)
   provides `Task::delay_until`, which the localization task uses to avoid
   timing drift and a busy loop.
4. Dellaert, Fox, Burgard, and Thrun's original
   [Monte Carlo Localization paper](https://repository.gatech.edu/entities/publication/eb1809f9-30b9-4369-b641-21592caefb22)
   establishes the sample-based Bayes-filter architecture: motion prediction,
   measurement weighting, resampling, and posterior estimation.
5. Their [AAAI MCL paper](https://aaai.org/papers/050-aaai99-050-monte-carlo-localization-efficient-position-estimation-for-mobile-robots/)
   shows why particles are a practical alternative to a dense 3D pose grid.
6. [Robust MCL](https://robots.stanford.edu/papers/thrun.robust-mcl.html) addresses
   dynamic environments and localization loss. Odyssey consequently keeps a
   nonzero outlier likelihood and exposes recovery entry points, but does not
   enable aggressive global injection without field evidence.
7. [MCL with a mixture proposal distribution](https://robots.stanford.edu/papers/thrun.mclmix.html)
   explains why an overconfident sensor proposal can make localization less
   robust. Odyssey starts from the safer odometry proposal and known-start
   Gaussian initialization.
8. Doucet, Godsill, and Andrieu's
   [sequential Monte Carlo treatment](https://www.stats.ox.ac.uk/~doucet/doucet_godsill_andrieu_sequentialmontecarloforbayesfiltering.pdf)
   gives the estimated effective sample size
   `1 / sum(normalized_weight²)` and supports resampling only below a threshold.
9. Hol, Schön, and Gustafsson's
   [resampling comparison](https://people.isy.liu.se/rt/schon/Publications/HolSG2006.pdf)
   covers the variance and implementation tradeoffs among common particle
   resamplers. Odyssey uses linear-time systematic resampling.
10. [Nav2 AMCL configuration](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/)
    uses separate odometry and global frames, configurable odometry noise, beam
    skipping, and recovery behavior. The separate-frame idea maps cleanly to a
    smooth Odyssey odometry pose plus a gated field-to-odometry correction.
11. The [Nav2/ROS AMCL resampling source](https://docs.ros.org/en/noetic/api/amcl/html/pf_8c_source.html)
    is a modern implementation reference for particle statistics and recovery;
    Odyssey uses a smaller fixed-count filter suited to the V5 Brain.
12. The current official
    [V5RC Override game manual](https://content.vexrobotics.com/docs/2026-2027/override/files/v5rc-override-0.1.2.pdf)
    contains the field drawings. The default helper uses a configurable
    140.4-inch inside-wall square centered at the origin rather than assuming
    the inside walls are exactly 144 inches apart.

The public `u-k-g/monte-carlo-localization` VEX repository was reviewed only as
an architectural reference. It has no license, so no source code from it was
copied.

## Data flow and coordinate system

```text
tracking wheels / drive encoders + IMU
             |  existing 10 ms odometry
             v
        raw pose delta ---------> particle prediction

V5 Distance Sensors -> validity/confidence gates -> wall raycasts
                                                -> robust log weights
                                                -> conditional resampling

raw odometry + trusted MCL estimate -> bounded field-to-odometry transform
```

All filter math uses inches and standard math radians: `0` points along field
`+X` and positive rotation is counterclockwise. Public pose calls default to
Odyssey compass degrees: `0` points along `+Y` and positive rotation is
clockwise. A sensor mount uses `offsetX` positive to the robot's right,
`offsetY` positive forward, and `yaw` positive counterclockwise from forward.
The ray starts at the transformed sensor origin, not the robot center.

## Algorithms

Prediction transforms the raw odometry delta into robot-relative forward,
rightward, and angular motion. Every particle applies that same local motion in
its own heading frame. Four configurable terms increase translation noise with
translation/rotation and angular noise with rotation/translation.

Before weighting, a reading is rejected when its device is absent, the API
returns an error/no-object sentinel, it is outside the configured range, its
confidence is too low, the estimated wall is beyond sensor range, or its
residual at the current estimate exceeds the trusted-hit gate. At least
`minimumValidSensors` must remain. This estimate-level gate is deliberately
conservative: ambiguous evidence causes an odometry-only update.

For each retained observation, a particle receives a Gaussian wall-hit density
mixed with a uniform random/outlier density and a final probability floor.
Weights are accumulated in log space, normalized after subtracting the largest
log weight, and recovered to uniform if normalization is non-finite. One bad
sensor therefore cannot assign a literal zero to every particle.

The filter computes effective sample size before resampling. Systematic
resampling runs in O(N) only below the configured fraction of particle count,
then assigns uniform weights and optional small roughening noise. Position is a
weighted arithmetic mean; heading uses `atan2(sum(w sin theta), sum(w cos
theta))`. The status also reports X/Y variance and circular angular spread.

## Configuration

Configure localization after the chassis has calibrated, then start it. This
example is illustrative—measure every sensor origin and replace the field
bounds with the coordinate frame used by your autonomous paths.

```cpp
pros::Distance frontDistance(13);
pros::Distance rightDistance(14);
pros::Distance backDistance(15);
pros::Distance leftDistance(16);

void initialize() {
    chassis.calibrate();

    odyssey::LocalizationConfig localization;
    localization.mode = odyssey::LocalizationMode::SHADOW;
    localization.field = odyssey::mcl::FieldModel::rectangle(
        -70.2, 70.2, -70.2, 70.2);
    localization.distanceSensors = {
        {&frontDistance, {.offsetX = 0, .offsetY = 6, .yaw = 0}},
        {&rightDistance, {.offsetX = 6, .offsetY = 0, .yaw = -M_PI_2}},
        {&backDistance, {.offsetX = 0, .offsetY = -6, .yaw = M_PI}},
        {&leftDistance, {.offsetX = -6, .offsetY = 0, .yaw = M_PI_2}},
    };
    localization.telemetryEnabled = true;

    if (odyssey::configureLocalization(localization))
        odyssey::startLocalization();
}
```

Important conservative defaults:

| Parameter | Default | Unit / meaning |
| --- | ---: | --- |
| `particleCount` | 500 | fixed particles allocated at configuration |
| `updatePeriodMs` | 50 | 20 Hz localization task |
| initial X/Y sigma | 1.0 | inches |
| initial heading sigma | 0.035 | radians (about 2°) |
| `sigmaHit` | 2.5 | inches; intentionally wider than ideal bench precision |
| hit/random mixture | 0.90 / 0.10 | modeled wall / outlier |
| trusted residual | 18 | inches; pre-weight gate |
| minimum valid sensors | 2 | filter acceptance gate |
| ESS resample ratio | 0.50 | resample below `0.5 * N` |
| maximum correction | 8 / 0.175 | inches / radians (about 10°) |
| stable updates | 3 | consecutive gated estimates |
| correction step | 0.5 / 0.0175 | inches / radians per update |
| correction freshness | 200 | milliseconds since last accepted correction |

These are safe starting points, not final robot tuning. In particular, measure
mounting offsets, inside-wall coordinates, sensor confidence by distance and
surface, motion-noise coefficients, hit sigma, residual gate, correction
limits, and execution time on the actual V5 Brain.

## Modes and telemetry

- `stopLocalization()` stops filter work and immediately returns raw odometry.
- `setLocalizationMode(DISABLED)` keeps the task allocated but skips all MCL
  updates and returns raw odometry.
- `SHADOW` is the default. `Chassis::getPose()` and every existing motion use
  raw odometry while `getLocalizedPose()` and `getLocalizationStatus()` expose
  MCL for comparison.
- `CORRECTION` changes the chassis output only after sensor count, finite
  weights, ESS, spread, correction magnitude, stability, stationary, and
  freshness gates pass. It does not reset encoders or the IMU and never changes
  motor commands.

`LocalizationStatus` is a latest-value buffer; polling it does not print or
block the localization task. Enable `telemetryEnabled` to retain raw range,
confidence, predicted range, residual, and rejection reason per sensor. It also
reports raw/MCL/output poses, their difference, uncertainty, ESS, valid sensor
count, measurement/correction decisions, execution time, and raycast count.
Rate-limit serial formatting in a separate low-frequency task (5 Hz is a useful
starting point).

## Replay and deterministic tests

The core has no PROS dependency. Run its host tests and benchmark with:

```sh
make -C tests test
make -C tests benchmark
```

Build the optional CSV replay tool with:

```sh
g++ -std=c++17 -O2 -Iinclude \
  tools/mcl_replay.cpp src/odyssey/mcl.cpp src/odyssey/pose.cpp \
  -o mcl_replay
```

Input rows are `timestamp_ms, odom_x, odom_y, compass_heading_deg`, followed by
`range_inches, confidence` for every configured sensor. Mount arguments are
`right_offset:forward_offset:yaw_degrees[:minimum:maximum:minimum_confidence]`:

```sh
./mcl_replay trace.csv -70.2 70.2 -70.2 70.2 \
  0:6:0 6:0:-90 0:-6:180 -6:0:90 > replay-output.csv
```

## Physical validation

Keep `SHADOW` mode through these stages and save telemetry at each known pose:

1. Static tests at multiple X/Y positions and headings.
2. Known straight forward and reverse travel.
3. Repeated turns, confirming MCL does not degrade the continuous IMU heading.
4. A square or other closed route returning to the measured start.
5. Controlled wheel slip to create repeatable odometry error.
6. Obstruction of one sensor.
7. A movable game object or robot between a sensor and the wall.
8. `CORRECTION` only while stopped or between chassis motions.
9. Active-motion correction only if recorded evidence shows an improvement.

Known limitations: the initial map contains static segments only; movable game
objects are treated as outliers. The filter uses a known-start Gaussian rather
than global localization, performs no aggressive kidnapped-robot recovery, and
cannot validate robot-specific geometry or noise values without hardware logs.
