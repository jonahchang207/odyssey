#pragma once

#include <cstdint>
#include <vector>

#include "odyssey/mcl.hpp"
#include "pros/distance.hpp"

namespace odyssey {

enum class LocalizationMode {
    /** no MCL task work and raw odometry is always returned */
    DISABLED,
    /** run MCL for telemetry while all driving continues on raw odometry */
    SHADOW,
    /** apply only stable, gated corrections through a field-to-odometry transform */
    CORRECTION
};

enum class LocalizationState { UNINITIALIZED, ODOMETRY_ONLY, DEGRADED, TRACKING };

enum class CorrectionRejection {
    NONE,
    DISABLED,
    SHADOW_MODE,
    NO_MEASUREMENT,
    INSUFFICIENT_SENSORS,
    NONFINITE,
    LOW_EFFECTIVE_SAMPLE_SIZE,
    EXCESSIVE_POSITION_SPREAD,
    EXCESSIVE_ANGULAR_SPREAD,
    EXCESSIVE_TRANSLATION_CORRECTION,
    EXCESSIVE_HEADING_CORRECTION,
    ROBOT_MOVING,
    NOT_STABLE,
    ODOMETRY_RESET,
    ODOMETRY_DISCONTINUITY
};

/** A non-owning PROS device plus its robot-relative MCL model. */
struct DistanceSensorConfig {
    pros::Distance *device = nullptr;
    mcl::SensorModel model;
};

struct CorrectionConfig {
    std::size_t minimumValidSensors = 2;
    float minimumEffectiveSampleSizeRatio = 0.10f;
    float maximumSigmaX = 3.0f;
    float maximumSigmaY = 3.0f;
    float maximumSigmaTheta = 0.12f;
    float maximumTranslationCorrection = 8.0f;
    float maximumHeadingCorrection = 0.175f;
    std::size_t stableUpdatesRequired = 3;
    bool requireStationary = true;
    float stationaryTranslationThreshold = 0.35f;
    float stationaryRotationThreshold = 0.035f;
    float maximumTranslationStep = 0.5f;
    float maximumHeadingStep = 0.0175f;
    std::uint32_t maximumPoseAgeMs = 200;
};

/** Centralized configuration for the independent localization service. */
struct LocalizationConfig {
    mcl::Config filter;
    /** Default is the centered 140.4-inch V5RC inside-wall perimeter. */
    mcl::FieldModel field = mcl::FieldModel::rectangle(-70.2f, 70.2f, -70.2f, 70.2f);
    std::vector<DistanceSensorConfig> distanceSensors;
    LocalizationMode mode = LocalizationMode::SHADOW;
    std::uint32_t updatePeriodMs = 50;
    /** Reinitialize instead of treating an impossible jump as robot motion. */
    float maximumOdometryDelta = 24.0f;
    float maximumOdometryRotationDelta = 1.57f;
    CorrectionConfig correction;
    /** Retain per-sensor raw/predicted/residual data in the latest status. */
    bool telemetryEnabled = false;
};

struct SensorTelemetry {
    std::int32_t rawRangeMm = 0;
    std::int32_t rawConfidence = 0;
    float rangeInches = 0;
    float predictedRangeInches = 0;
    float residualInches = 0;
    mcl::ObservationRejection rejection = mcl::ObservationRejection::NONE;
};

/** Latest localization snapshot. Poses use inches and math-frame radians. */
struct LocalizationStatus {
    LocalizationState state = LocalizationState::UNINITIALIZED;
    LocalizationMode mode = LocalizationMode::DISABLED;
    Pose odometryPose;
    Pose localizedPose;
    Pose outputPose;
    float odometryToLocalizedDistance = 0;
    float odometryToLocalizedHeading = 0;
    float varianceX = 0;
    float varianceY = 0;
    float sigmaTheta = 0;
    float effectiveSampleSize = 0;
    float measurementQuality = 0;
    std::size_t validSensorCount = 0;
    std::size_t particleCount = 0;
    std::size_t raycastCount = 0;
    bool measurementAccepted = false;
    bool weightsFinite = true;
    bool resampled = false;
    bool correctionAccepted = false;
    CorrectionRejection correctionRejection = CorrectionRejection::DISABLED;
    std::uint32_t timestampMs = 0;
    std::uint32_t lastCorrectionTimestampMs = 0;
    std::uint32_t updatePeriodMs = 0;
    std::uint32_t executionTimeUs = 0;
    float averageExecutionTimeUs = 0;
    std::uint32_t maximumExecutionTimeUs = 0;
    std::vector<SensorTelemetry> sensors;
};

/** Configure MCL. Returns false if the service is currently running. */
bool configureLocalization(const LocalizationConfig &config);

/** Initialize at raw odometry and start the independent periodic task. */
bool startLocalization();

/** Stop filter updates. Raw odometry immediately becomes the output pose. */
void stopLocalization();

/** Reinitialize the particle cloud around the current raw odometry pose. */
bool resetLocalization();

void setLocalizationMode(LocalizationMode mode);
LocalizationMode getLocalizationMode();

/** Raw odometry, unaffected by localization. */
Pose getOdometryPose(bool radians = false);

/** Latest MCL estimate, or raw odometry before initialization. */
Pose getLocalizedPose(bool radians = false);

/** Selected driving pose; corrected only in CORRECTION mode after all gates pass. */
Pose getLocalizationOutputPose(bool radians = false);

LocalizationStatus getLocalizationStatus();

} // namespace odyssey
