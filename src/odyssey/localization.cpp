#include "odyssey/localization.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <utility>

#include "odyssey/odometry.hpp"
#include "odyssey/util.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"

namespace odyssey {
namespace {

constexpr float kMillimetersPerInch = 25.4f;
constexpr std::int32_t kNoObjectDistance = 9999;

struct RigidCorrection {
    float x = 0;
    float y = 0;
    float theta = 0;
    bool valid = false;
};

LocalizationConfig serviceConfig;
mcl::Filter *filter = nullptr;
pros::Task *localizationTask = nullptr;
pros::Mutex serviceMutex;
pros::Mutex statusMutex;
std::atomic<bool> configured(false);
std::atomic<bool> running(false);
std::atomic<LocalizationMode> activeMode(LocalizationMode::DISABLED);
std::atomic<std::uint32_t> activePeriodMs(50);
OdometryState previousOdometry;
RigidCorrection correction;
std::uint32_t correctionTimestampMs = 0;
LocalizationStatus latestStatus;
std::vector<mcl::Observation> observationBuffer;
std::vector<SensorTelemetry> sensorBuffer;
std::size_t stableUpdateCount = 0;
std::uint64_t totalExecutionTimeUs = 0;
std::uint32_t executionCount = 0;

Pose convertPose(Pose pose, bool radians) {
    if (radians)
        return pose;
    pose.theta = 90.0f - radToDeg(pose.theta);
    return pose;
}

Pose applyCorrection(const Pose &pose, const RigidCorrection &transform) {
    if (!transform.valid)
        return pose;
    const float cosine = std::cos(transform.theta);
    const float sine = std::sin(transform.theta);
    return Pose(cosine * pose.x - sine * pose.y + transform.x,
                sine * pose.x + cosine * pose.y + transform.y,
                mcl::wrapAngle(pose.theta + transform.theta));
}

RigidCorrection correctionFor(const Pose &odometryPose, const Pose &fieldPose) {
    RigidCorrection result;
    result.theta = mcl::shortestAngleDifference(fieldPose.theta, odometryPose.theta);
    const float cosine = std::cos(result.theta);
    const float sine = std::sin(result.theta);
    result.x = fieldPose.x - (cosine * odometryPose.x - sine * odometryPose.y);
    result.y = fieldPose.y - (sine * odometryPose.x + cosine * odometryPose.y);
    result.valid = true;
    return result;
}

Pose rawOutputPose(const Pose &rawPose) {
    if (!running.load() || activeMode.load() != LocalizationMode::CORRECTION)
        return rawPose;

    statusMutex.take(TIMEOUT_MAX);
    const RigidCorrection snapshot = correction;
    const std::uint32_t timestampMs = correctionTimestampMs;
    statusMutex.give();
    const std::uint32_t age = pros::millis() - timestampMs;
    if (!snapshot.valid || age > serviceConfig.correction.maximumPoseAgeMs)
        return rawPose;
    return applyCorrection(rawPose, snapshot);
}

mcl::OdometryDelta calculateDelta(const Pose &previous, const Pose &current) {
    const float deltaX = current.x - previous.x;
    const float deltaY = current.y - previous.y;
    return {
        deltaX * std::cos(previous.theta) + deltaY * std::sin(previous.theta),
        deltaX * std::sin(previous.theta) - deltaY * std::cos(previous.theta),
        mcl::shortestAngleDifference(current.theta, previous.theta),
    };
}

CorrectionRejection evaluateCorrection(const mcl::StepStatus &filterStatus,
                                       const mcl::OdometryDelta &delta, const Pose &rawPose,
                                       const Pose &localizedPose) {
    const CorrectionConfig &gates = serviceConfig.correction;
    if (!filterStatus.measurementAccepted)
        return CorrectionRejection::NO_MEASUREMENT;
    if (filterStatus.validSensorCount < gates.minimumValidSensors)
        return CorrectionRejection::INSUFFICIENT_SENSORS;
    if (!filterStatus.weightsFinite || !std::isfinite(localizedPose.x) ||
        !std::isfinite(localizedPose.y) || !std::isfinite(localizedPose.theta))
        return CorrectionRejection::NONFINITE;
    if (filterStatus.effectiveSampleSize <
        gates.minimumEffectiveSampleSizeRatio * serviceConfig.filter.particleCount)
        return CorrectionRejection::LOW_EFFECTIVE_SAMPLE_SIZE;
    if (std::sqrt(filterStatus.estimate.varianceX) > gates.maximumSigmaX ||
        std::sqrt(filterStatus.estimate.varianceY) > gates.maximumSigmaY)
        return CorrectionRejection::EXCESSIVE_POSITION_SPREAD;
    if (filterStatus.estimate.sigmaTheta > gates.maximumSigmaTheta)
        return CorrectionRejection::EXCESSIVE_ANGULAR_SPREAD;

    const Pose currentOutput = applyCorrection(rawPose, correction);
    if (currentOutput.distance(localizedPose) > gates.maximumTranslationCorrection)
        return CorrectionRejection::EXCESSIVE_TRANSLATION_CORRECTION;
    if (std::fabs(mcl::shortestAngleDifference(localizedPose.theta, currentOutput.theta)) >
        gates.maximumHeadingCorrection)
        return CorrectionRejection::EXCESSIVE_HEADING_CORRECTION;
    if (gates.requireStationary &&
        (std::hypot(delta.forward, delta.rightward) > gates.stationaryTranslationThreshold ||
         std::fabs(delta.theta) > gates.stationaryRotationThreshold))
        return CorrectionRejection::ROBOT_MOVING;
    return CorrectionRejection::NONE;
}

void updateCorrection(const Pose &rawPose, const Pose &localizedPose) {
    const Pose currentOutput = applyCorrection(rawPose, correction);
    const float dx = localizedPose.x - currentOutput.x;
    const float dy = localizedPose.y - currentOutput.y;
    const float distance = std::hypot(dx, dy);
    const float scale = distance > serviceConfig.correction.maximumTranslationStep
                            ? serviceConfig.correction.maximumTranslationStep / distance
                            : 1.0f;
    Pose boundedOutput(currentOutput.x + dx * scale, currentOutput.y + dy * scale,
                       currentOutput.theta);
    const float headingError =
        mcl::shortestAngleDifference(localizedPose.theta, currentOutput.theta);
    boundedOutput.theta = mcl::wrapAngle(
        currentOutput.theta + std::clamp(headingError,
                                         -std::fabs(serviceConfig.correction.maximumHeadingStep),
                                         std::fabs(serviceConfig.correction.maximumHeadingStep)));
    correction = correctionFor(rawPose, boundedOutput);
}

void initializeAtOdometry(const OdometryState &state) {
    filter->initialize(state.pose);
    previousOdometry = state;
    statusMutex.take(TIMEOUT_MAX);
    correction = {};
    correctionTimestampMs = 0;
    statusMutex.give();
    stableUpdateCount = 0;
}

void localizationUpdate() {
    const std::uint64_t startedUs = pros::micros();
    serviceMutex.take(TIMEOUT_MAX);
    if (!running.load() || filter == nullptr) {
        serviceMutex.give();
        return;
    }

    const OdometryState odometry = getOdometryState();
    mcl::OdometryDelta delta = calculateDelta(previousOdometry.pose, odometry.pose);
    CorrectionRejection forcedRejection = CorrectionRejection::NONE;
    if (odometry.resetGeneration != previousOdometry.resetGeneration) {
        initializeAtOdometry(odometry);
        delta = {};
        forcedRejection = CorrectionRejection::ODOMETRY_RESET;
    } else if (std::hypot(delta.forward, delta.rightward) > serviceConfig.maximumOdometryDelta ||
               std::fabs(delta.theta) > serviceConfig.maximumOdometryRotationDelta) {
        initializeAtOdometry(odometry);
        delta = {};
        forcedRejection = CorrectionRejection::ODOMETRY_DISCONTINUITY;
    }
    previousOdometry = odometry;

    for (std::size_t index = 0; index < serviceConfig.distanceSensors.size(); ++index) {
        const DistanceSensorConfig &configuredSensor = serviceConfig.distanceSensors[index];
        std::int32_t rawRange = PROS_ERR;
        std::int32_t rawConfidence = PROS_ERR;
        float rangeInches = std::numeric_limits<float>::quiet_NaN();
        if (configuredSensor.device != nullptr && configuredSensor.model.enabled) {
            rawRange = configuredSensor.device->get();
            rawConfidence = configuredSensor.device->get_confidence();
            if (rawRange != PROS_ERR && rawRange != kNoObjectDistance && rawRange >= 0)
                rangeInches = rawRange / kMillimetersPerInch;
        }
        observationBuffer[index] = {index, rangeInches, rawConfidence};
        if (serviceConfig.telemetryEnabled)
            sensorBuffer[index] = {
                rawRange, rawConfidence, rangeInches, 0, 0, mcl::ObservationRejection::NONE};
    }

    const mcl::StepStatus filterStatus = filter->step(delta, observationBuffer);
    const Pose localizedPose = filterStatus.estimate.pose;
    CorrectionRejection rejection =
        forcedRejection != CorrectionRejection::NONE
            ? forcedRejection
            : evaluateCorrection(filterStatus, delta, odometry.pose, localizedPose);
    if (rejection == CorrectionRejection::NONE)
        stableUpdateCount++;
    else
        stableUpdateCount = 0;

    bool correctionAccepted = false;
    if (activeMode.load() == LocalizationMode::DISABLED) {
        rejection = CorrectionRejection::DISABLED;
    } else if (activeMode.load() == LocalizationMode::SHADOW) {
        rejection = CorrectionRejection::SHADOW_MODE;
    } else if (rejection == CorrectionRejection::NONE) {
        if (stableUpdateCount < serviceConfig.correction.stableUpdatesRequired) {
            rejection = CorrectionRejection::NOT_STABLE;
        } else {
            statusMutex.take(TIMEOUT_MAX);
            updateCorrection(odometry.pose, localizedPose);
            correctionTimestampMs = pros::millis();
            statusMutex.give();
            correctionAccepted = true;
        }
    }

    const std::uint32_t finishedMs = pros::millis();
    const bool correctionFresh = correction.valid && finishedMs - correctionTimestampMs <=
                                                         serviceConfig.correction.maximumPoseAgeMs;
    const Pose outputPose = activeMode.load() == LocalizationMode::CORRECTION && correctionFresh
                                ? applyCorrection(odometry.pose, correction)
                                : odometry.pose;
    const std::uint32_t executionUs = static_cast<std::uint32_t>(pros::micros() - startedUs);
    totalExecutionTimeUs += executionUs;
    executionCount++;

    if (serviceConfig.telemetryEnabled) {
        const std::vector<mcl::ObservationResult> &results = filter->observationResults();
        for (std::size_t index = 0; index < std::min(results.size(), sensorBuffer.size());
             ++index) {
            sensorBuffer[index].predictedRangeInches = results[index].expectedRange;
            sensorBuffer[index].residualInches = results[index].residual;
            sensorBuffer[index].rejection = results[index].rejection;
        }
    }

    LocalizationStatus next;
    next.mode = activeMode.load();
    const bool confidentEstimate =
        filterStatus.weightsFinite &&
        filterStatus.validSensorCount >= serviceConfig.correction.minimumValidSensors &&
        filterStatus.effectiveSampleSize >=
            serviceConfig.correction.minimumEffectiveSampleSizeRatio *
                serviceConfig.filter.particleCount &&
        std::sqrt(filterStatus.estimate.varianceX) <= serviceConfig.correction.maximumSigmaX &&
        std::sqrt(filterStatus.estimate.varianceY) <= serviceConfig.correction.maximumSigmaY &&
        filterStatus.estimate.sigmaTheta <= serviceConfig.correction.maximumSigmaTheta;
    next.state =
        !filterStatus.measurementAccepted
            ? LocalizationState::ODOMETRY_ONLY
            : (confidentEstimate ? LocalizationState::TRACKING : LocalizationState::DEGRADED);
    next.odometryPose = odometry.pose;
    next.localizedPose = localizedPose;
    next.outputPose = outputPose;
    next.odometryToLocalizedDistance = odometry.pose.distance(localizedPose);
    next.odometryToLocalizedHeading =
        mcl::shortestAngleDifference(localizedPose.theta, odometry.pose.theta);
    next.varianceX = filterStatus.estimate.varianceX;
    next.varianceY = filterStatus.estimate.varianceY;
    next.sigmaTheta = filterStatus.estimate.sigmaTheta;
    next.effectiveSampleSize = filterStatus.effectiveSampleSize;
    next.measurementQuality = filterStatus.measurementQuality;
    next.validSensorCount = filterStatus.validSensorCount;
    next.particleCount = serviceConfig.filter.particleCount;
    next.raycastCount = filterStatus.raycastCount;
    next.measurementAccepted = filterStatus.measurementAccepted;
    next.weightsFinite = filterStatus.weightsFinite;
    next.resampled = filterStatus.resampled;
    next.correctionAccepted = correctionAccepted;
    next.correctionRejection = rejection;
    next.timestampMs = finishedMs;
    next.lastCorrectionTimestampMs = correctionTimestampMs;
    next.updatePeriodMs = activePeriodMs.load();
    next.executionTimeUs = executionUs;
    next.averageExecutionTimeUs =
        static_cast<float>(totalExecutionTimeUs) / static_cast<float>(executionCount);
    next.maximumExecutionTimeUs = std::max(latestStatus.maximumExecutionTimeUs, executionUs);
    statusMutex.take(TIMEOUT_MAX);
    if (serviceConfig.telemetryEnabled) {
        next.sensors.swap(latestStatus.sensors);
        next.sensors.assign(sensorBuffer.begin(), sensorBuffer.end());
    }
    latestStatus = std::move(next);
    statusMutex.give();
    serviceMutex.give();
}

void taskLoop() {
    std::uint32_t previousWake = pros::millis();
    while (true) {
        if (running.load() && activeMode.load() != LocalizationMode::DISABLED)
            localizationUpdate();
        const std::uint32_t period = activePeriodMs.load();
        pros::Task::delay_until(&previousWake, std::max<std::uint32_t>(period, 10));
    }
}

} // namespace

bool configureLocalization(const LocalizationConfig &config) {
    if (running.load() || config.filter.particleCount == 0 || config.field.segments.empty())
        return false;
    serviceMutex.take(TIMEOUT_MAX);
    std::vector<mcl::SensorModel> models;
    models.reserve(config.distanceSensors.size());
    for (const DistanceSensorConfig &sensor : config.distanceSensors)
        models.push_back(sensor.model);
    delete filter;
    serviceConfig = config;
    filter = new mcl::Filter(config.filter, config.field, std::move(models));
    observationBuffer.resize(config.distanceSensors.size());
    sensorBuffer.resize(config.distanceSensors.size());
    activePeriodMs.store(std::max<std::uint32_t>(config.updatePeriodMs, 10));
    activeMode.store(config.mode);
    configured.store(true);
    serviceMutex.give();
    return true;
}

bool startLocalization() {
    if (!configured.load() || running.load())
        return configured.load();
    serviceMutex.take(TIMEOUT_MAX);
    const OdometryState odometry = getOdometryState();
    if (serviceConfig.filter.randomSeed == 0)
        filter->setSeed(pros::millis() ^ 0x9e3779b9U);
    initializeAtOdometry(odometry);
    statusMutex.take(TIMEOUT_MAX);
    latestStatus = {};
    latestStatus.mode = activeMode.load();
    latestStatus.odometryPose = odometry.pose;
    latestStatus.localizedPose = odometry.pose;
    latestStatus.outputPose = odometry.pose;
    latestStatus.particleCount = serviceConfig.filter.particleCount;
    if (serviceConfig.telemetryEnabled)
        latestStatus.sensors.resize(serviceConfig.distanceSensors.size());
    statusMutex.give();
    totalExecutionTimeUs = 0;
    executionCount = 0;
    running.store(true);
    if (localizationTask == nullptr)
        localizationTask = new pros::Task(taskLoop);
    serviceMutex.give();
    return true;
}

void stopLocalization() { running.store(false); }

bool resetLocalization() {
    if (!configured.load())
        return false;
    serviceMutex.take(TIMEOUT_MAX);
    const OdometryState odometry = getOdometryState();
    initializeAtOdometry(odometry);
    statusMutex.take(TIMEOUT_MAX);
    latestStatus.state = LocalizationState::UNINITIALIZED;
    latestStatus.odometryPose = odometry.pose;
    latestStatus.localizedPose = odometry.pose;
    latestStatus.outputPose = odometry.pose;
    latestStatus.correctionAccepted = false;
    latestStatus.correctionRejection = CorrectionRejection::ODOMETRY_RESET;
    statusMutex.give();
    serviceMutex.give();
    return true;
}

void setLocalizationMode(LocalizationMode mode) {
    serviceMutex.take(TIMEOUT_MAX);
    const LocalizationMode previousMode = activeMode.load();
    if (mode != previousMode) {
        if (previousMode == LocalizationMode::DISABLED && mode != LocalizationMode::DISABLED &&
            configured.load())
            initializeAtOdometry(getOdometryState());
        statusMutex.take(TIMEOUT_MAX);
        correction = {};
        correctionTimestampMs = 0;
        statusMutex.give();
        stableUpdateCount = 0;
    }
    activeMode.store(mode);
    serviceMutex.give();
}

LocalizationMode getLocalizationMode() { return activeMode.load(); }

Pose getOdometryPose(bool radians) { return getPose(radians); }

Pose getLocalizedPose(bool radians) {
    if (!configured.load())
        return getPose(radians);
    statusMutex.take(TIMEOUT_MAX);
    const LocalizationState state = latestStatus.state;
    const Pose pose = latestStatus.localizedPose;
    statusMutex.give();
    if (state == LocalizationState::UNINITIALIZED)
        return getPose(radians);
    return convertPose(pose, radians);
}

Pose getLocalizationOutputPose(bool radians) {
    const Pose output = rawOutputPose(getPose(true));
    return convertPose(output, radians);
}

LocalizationStatus getLocalizationStatus() {
    statusMutex.take(TIMEOUT_MAX);
    LocalizationStatus snapshot = latestStatus;
    statusMutex.give();
    snapshot.mode = activeMode.load();
    return snapshot;
}

} // namespace odyssey
