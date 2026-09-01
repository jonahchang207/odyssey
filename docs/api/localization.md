# Localization API

Include `odyssey/api.hpp`. MCL is optional and has no effect until configured
and started.

!!! warning "Validate in shadow mode first"
    `SHADOW` leaves every chassis motion on raw odometry while exposing the MCL
    estimate and telemetry for comparison. Move to `CORRECTION` only after the
    physical validation sequence consistently improves known-pose results.

## Lifecycle and pose selection

```cpp
bool configureLocalization(const LocalizationConfig& config);
bool startLocalization();
void stopLocalization();
bool resetLocalization();

void setLocalizationMode(LocalizationMode mode);
LocalizationMode getLocalizationMode();

Pose getOdometryPose(bool radians = false);
Pose getLocalizedPose(bool radians = false);
Pose getLocalizationOutputPose(bool radians = false);
LocalizationStatus getLocalizationStatus();
```

`configureLocalization` returns `false` for an empty particle set, empty field,
or reconfiguration while running. `startLocalization` allocates the PROS task
once and initializes particles at the current raw pose. `stopLocalization` and
`DISABLED` both make the output raw odometry; disabled mode also skips filter
work. `resetLocalization` performs known-pose Gaussian initialization around
raw odometry. `Chassis::setPose` calls it automatically, while the odometry reset
generation also protects direct free-function resets.

`getOdometryPose` is always raw. `getLocalizedPose` is the particle estimate.
`getLocalizationOutputPose` is what `Chassis::getPose` uses: raw in disabled and
shadow modes, or the fresh gated field-to-odometry transform in correction mode.

## Configuration types

`LocalizationConfig` centralizes the particle count, update period, static field
segments, N distance sensors, motion/measurement noise, ESS threshold,
roughening, pose-reset discontinuity limits, correction gates, and telemetry.
Device pointers are non-owning and must remain alive while localization runs.

`mcl::FieldModel::rectangle(minX, maxX, minY, maxY)` creates the four inside-wall
segments. Additional static `mcl::Segment` values may be appended. Do not add
movable game objects to the map.

See [Monte Carlo localization](../explanation/monte-carlo-localization.md) for
the complete configuration example, defaults, conventions, tuning, telemetry,
and validation procedure.

![How optional localization joins Odyssey's raw odometry and selected driving pose](../assets/system-overview.svg){ .odyssey-diagram }
