#include "odyssey/mcl.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace odyssey::mcl {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kRayEpsilon = 1.0e-6f;

float cross(float ax, float ay, float bx, float by) { return ax * by - ay * bx; }

float clampPositive(float value, float fallback) {
    return std::isfinite(value) && value > 0 ? value : fallback;
}

} // namespace

FieldModel FieldModel::rectangle(float minX, float maxX, float minY, float maxY) {
    FieldModel field;
    field.segments.reserve(4);
    field.segments.push_back({minX, minY, maxX, minY});
    field.segments.push_back({maxX, minY, maxX, maxY});
    field.segments.push_back({maxX, maxY, minX, maxY});
    field.segments.push_back({minX, maxY, minX, minY});
    return field;
}

float wrapAngle(float angle) {
    if (!std::isfinite(angle))
        return 0;
    angle = std::fmod(angle + kPi, kTwoPi);
    if (angle < 0)
        angle += kTwoPi;
    return angle - kPi;
}

float shortestAngleDifference(float target, float current) { return wrapAngle(target - current); }

float expectedRange(const Pose &pose, const SensorModel &sensor, const FieldModel &field) {
    if (!sensor.enabled || !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(pose.theta)) {
        return std::numeric_limits<float>::infinity();
    }

    // Sensor offsets use robot coordinates: +x right, +y forward. Pose theta
    // is the math-frame direction of robot forward.
    const float cosTheta = std::cos(pose.theta);
    const float sinTheta = std::sin(pose.theta);
    const float originX = pose.x + sensor.offsetY * cosTheta + sensor.offsetX * sinTheta;
    const float originY = pose.y + sensor.offsetY * sinTheta - sensor.offsetX * cosTheta;
    const float rayTheta = pose.theta + sensor.yaw;
    const float directionX = std::cos(rayTheta);
    const float directionY = std::sin(rayTheta);

    float nearest = std::numeric_limits<float>::infinity();
    for (const Segment &segment : field.segments) {
        const float segmentX = segment.x2 - segment.x1;
        const float segmentY = segment.y2 - segment.y1;
        const float denominator = cross(directionX, directionY, segmentX, segmentY);
        if (std::fabs(denominator) <= kRayEpsilon)
            continue;

        const float toSegmentX = segment.x1 - originX;
        const float toSegmentY = segment.y1 - originY;
        const float rayDistance = cross(toSegmentX, toSegmentY, segmentX, segmentY) / denominator;
        const float segmentFraction =
            cross(toSegmentX, toSegmentY, directionX, directionY) / denominator;
        if (rayDistance >= -kRayEpsilon && segmentFraction >= -kRayEpsilon &&
            segmentFraction <= 1.0f + kRayEpsilon) {
            nearest = std::min(nearest, std::max(0.0f, rayDistance));
        }
    }
    return nearest;
}

bool normalizeLogWeights(const std::vector<float> &logWeights,
                         std::vector<float> &normalizedWeights) {
    normalizedWeights.assign(logWeights.size(), 0);
    if (logWeights.empty())
        return false;

    float maximum = -std::numeric_limits<float>::infinity();
    for (const float value : logWeights) {
        if (std::isfinite(value))
            maximum = std::max(maximum, value);
    }
    if (!std::isfinite(maximum)) {
        const float uniform = 1.0f / static_cast<float>(logWeights.size());
        std::fill(normalizedWeights.begin(), normalizedWeights.end(), uniform);
        return false;
    }

    double sum = 0;
    for (std::size_t i = 0; i < logWeights.size(); ++i) {
        if (!std::isfinite(logWeights[i]))
            continue;
        normalizedWeights[i] = std::exp(logWeights[i] - maximum);
        sum += normalizedWeights[i];
    }
    if (!std::isfinite(sum) || sum <= 0) {
        const float uniform = 1.0f / static_cast<float>(logWeights.size());
        std::fill(normalizedWeights.begin(), normalizedWeights.end(), uniform);
        return false;
    }
    for (float &value : normalizedWeights)
        value = static_cast<float>(value / sum);
    return true;
}

float effectiveSampleSize(const std::vector<Particle> &particles) {
    double sumSquares = 0;
    for (const Particle &particle : particles) {
        if (!std::isfinite(particle.weight) || particle.weight < 0)
            return 0;
        sumSquares += static_cast<double>(particle.weight) * particle.weight;
    }
    if (!std::isfinite(sumSquares) || sumSquares <= 0)
        return 0;
    return static_cast<float>(1.0 / sumSquares);
}

Estimate weightedEstimate(const std::vector<Particle> &particles) {
    Estimate result;
    if (particles.empty())
        return result;

    double weightSum = 0;
    double x = 0;
    double y = 0;
    double sinTheta = 0;
    double cosTheta = 0;
    for (const Particle &particle : particles) {
        if (!std::isfinite(particle.weight) || particle.weight < 0)
            continue;
        weightSum += particle.weight;
        x += particle.weight * particle.pose.x;
        y += particle.weight * particle.pose.y;
        sinTheta += particle.weight * std::sin(particle.pose.theta);
        cosTheta += particle.weight * std::cos(particle.pose.theta);
    }
    if (!std::isfinite(weightSum) || weightSum <= 0)
        return result;

    result.pose.x = static_cast<float>(x / weightSum);
    result.pose.y = static_cast<float>(y / weightSum);
    result.pose.theta = std::atan2(static_cast<float>(sinTheta), static_cast<float>(cosTheta));

    double varianceX = 0;
    double varianceY = 0;
    for (const Particle &particle : particles) {
        const double dx = particle.pose.x - result.pose.x;
        const double dy = particle.pose.y - result.pose.y;
        varianceX += particle.weight * dx * dx;
        varianceY += particle.weight * dy * dy;
    }
    result.varianceX = static_cast<float>(varianceX / weightSum);
    result.varianceY = static_cast<float>(varianceY / weightSum);
    const double resultant = std::min(1.0, std::hypot(sinTheta, cosTheta) / weightSum);
    result.sigmaTheta =
        resultant > 0 ? static_cast<float>(std::sqrt(std::max(0.0, -2.0 * std::log(resultant))))
                      : kPi;
    return result;
}

Filter::Filter(Config config, FieldModel field, std::vector<SensorModel> sensors)
    : config_(config), field_(std::move(field)), sensors_(std::move(sensors)),
      random_(config.randomSeed == 0 ? 1 : config.randomSeed) {
    if (config_.particleCount == 0)
        config_.particleCount = 1;
    particles_.resize(config_.particleCount);
    scratch_.resize(config_.particleCount);
    logWeights_.resize(config_.particleCount);
    normalizedWeights_.resize(config_.particleCount);
    usableObservationIndices_.reserve(sensors_.size());
    observationResults_.reserve(sensors_.size());
}

void Filter::setSeed(std::uint32_t seed) { random_.seed(seed == 0 ? 1 : seed); }

float Filter::gaussian(float sigma) {
    if (!std::isfinite(sigma) || sigma <= 0)
        return 0;
    std::normal_distribution<float> distribution(0, sigma);
    return distribution(random_);
}

void Filter::initialize(const Pose &mean) {
    const float uniformWeight = 1.0f / static_cast<float>(particles_.size());
    for (Particle &particle : particles_) {
        particle.pose.x = mean.x + gaussian(config_.initialSigmaX);
        particle.pose.y = mean.y + gaussian(config_.initialSigmaY);
        particle.pose.theta = wrapAngle(mean.theta + gaussian(config_.initialSigmaTheta));
        particle.weight = uniformWeight;
    }
    estimate_ = weightedEstimate(particles_);
}

void Filter::predict(const OdometryDelta &delta) {
    const float translation = std::hypot(delta.forward, delta.rightward);
    const float rotationMagnitude = std::fabs(delta.theta);
    const float translationSigma =
        std::fabs(config_.motionNoise.translationFromTranslation) * translation +
        std::fabs(config_.motionNoise.translationFromRotation) * rotationMagnitude;
    const float rotationSigma =
        std::fabs(config_.motionNoise.rotationFromRotation) * rotationMagnitude +
        std::fabs(config_.motionNoise.rotationFromTranslation) * translation;

    for (Particle &particle : particles_) {
        const float noisyForward = delta.forward + gaussian(translationSigma);
        const float noisyRightward = delta.rightward + gaussian(translationSigma);
        const float noisyTheta = delta.theta + gaussian(rotationSigma);
        const float middleTheta = particle.pose.theta + noisyTheta / 2.0f;
        particle.pose.x +=
            noisyForward * std::cos(middleTheta) + noisyRightward * std::sin(middleTheta);
        particle.pose.y +=
            noisyForward * std::sin(middleTheta) - noisyRightward * std::cos(middleTheta);
        particle.pose.theta = wrapAngle(particle.pose.theta + noisyTheta);
    }
}

bool Filter::selectObservations(const std::vector<Observation> &observations, StepStatus &status) {
    usableObservationIndices_.clear();
    observationResults_.clear();
    observationResults_.reserve(std::max(observationResults_.capacity(), observations.size()));

    double qualitySum = 0;
    for (const Observation &observation : observations) {
        ObservationResult result;
        result.observation = observation;
        if (observation.sensorIndex >= sensors_.size()) {
            result.rejection = ObservationRejection::INVALID_INDEX;
        } else {
            const SensorModel &sensor = sensors_[observation.sensorIndex];
            if (!sensor.enabled)
                result.rejection = ObservationRejection::DISABLED;
            else if (!std::isfinite(observation.range))
                result.rejection = ObservationRejection::NONFINITE;
            else if (observation.range < sensor.minimumRange ||
                     observation.range > sensor.maximumRange)
                result.rejection = ObservationRejection::OUT_OF_RANGE;
            else if (observation.range >= sensor.confidenceMinimumRange &&
                     observation.confidence < sensor.minimumConfidence)
                result.rejection = ObservationRejection::LOW_CONFIDENCE;
            else {
                result.expectedRange = expectedRange(estimate_.pose, sensor, field_);
                status.raycastCount++;
                if (!std::isfinite(result.expectedRange))
                    result.rejection = ObservationRejection::NO_MAP_INTERSECTION;
                else if (result.expectedRange > sensor.maximumRange)
                    result.rejection = ObservationRejection::EXPECTED_OUT_OF_RANGE;
                else {
                    result.residual = observation.range - result.expectedRange;
                    if (std::fabs(result.residual) >
                        config_.measurement.maximumResidualForTrustedHit)
                        result.rejection = ObservationRejection::RESIDUAL_GATE;
                    else {
                        result.rejection = ObservationRejection::NONE;
                        usableObservationIndices_.push_back(observationResults_.size());
                        const float scale =
                            clampPositive(config_.measurement.maximumResidualForTrustedHit, 1.0f);
                        qualitySum += std::max(0.0f, 1.0f - std::fabs(result.residual) / scale);
                    }
                }
            }
        }
        observationResults_.push_back(result);
    }

    status.validSensorCount = usableObservationIndices_.size();
    status.measurementQuality =
        status.validSensorCount > 0 ? static_cast<float>(qualitySum / status.validSensorCount) : 0;
    return status.validSensorCount >= config_.measurement.minimumValidSensors;
}

bool Filter::applyMeasurement(StepStatus &status) {
    const float sigma = clampPositive(config_.measurement.sigmaHit, 1.0f);
    const float normalScale = 1.0f / (sigma * std::sqrt(2.0f * kPi));

    for (std::size_t particleIndex = 0; particleIndex < particles_.size(); ++particleIndex) {
        const Particle &particle = particles_[particleIndex];
        logWeights_[particleIndex] =
            std::log(std::max(particle.weight, config_.measurement.minimumLikelihood));
        for (const std::size_t resultIndex : usableObservationIndices_) {
            const ObservationResult &result = observationResults_[resultIndex];
            const SensorModel &sensor = sensors_[result.observation.sensorIndex];
            const float expected = expectedRange(particle.pose, sensor, field_);
            status.raycastCount++;
            float likelihood = config_.measurement.minimumLikelihood;
            if (std::isfinite(expected) && expected <= sensor.maximumRange) {
                const float residual = result.observation.range - expected;
                const float gaussianLikelihood =
                    normalScale * std::exp(-0.5f * residual * residual / (sigma * sigma));
                const float usableSpan = std::max(1.0f, sensor.maximumRange - sensor.minimumRange);
                likelihood = config_.measurement.hitWeight * gaussianLikelihood +
                             config_.measurement.randomWeight / usableSpan;
                likelihood = std::max(likelihood, config_.measurement.minimumLikelihood);
            }
            logWeights_[particleIndex] += std::log(likelihood);
        }
    }

    const bool normalized = normalizeLogWeights(logWeights_, normalizedWeights_);
    for (std::size_t i = 0; i < particles_.size(); ++i)
        particles_[i].weight = normalizedWeights_[i];
    status.weightsFinite = normalized;
    return normalized;
}

StepStatus Filter::step(const OdometryDelta &delta, const std::vector<Observation> &observations) {
    StepStatus status;
    predict(delta);
    estimate_ = weightedEstimate(particles_);

    if (selectObservations(observations, status)) {
        status.measurementAccepted = applyMeasurement(status);
        if (status.measurementAccepted)
            estimate_ = weightedEstimate(particles_);
    }

    status.estimate = estimate_;
    status.effectiveSampleSize = effectiveSampleSize(particles_);
    const float threshold = std::clamp(config_.resampleEffectiveSampleSizeRatio, 0.0f, 1.0f) *
                            static_cast<float>(particles_.size());
    if (status.measurementAccepted && status.effectiveSampleSize < threshold) {
        systematicResample();
        status.resampled = true;
    }
    return status;
}

void Filter::systematicResample() {
    if (particles_.empty())
        return;
    const float step = 1.0f / static_cast<float>(particles_.size());
    std::uniform_real_distribution<float> uniform(0, step);
    const float start = uniform(random_);
    std::size_t sourceIndex = 0;
    float cumulative = particles_[0].weight;
    const float uniformWeight = step;

    for (std::size_t outputIndex = 0; outputIndex < particles_.size(); ++outputIndex) {
        const float target = start + static_cast<float>(outputIndex) * step;
        while (target > cumulative && sourceIndex + 1 < particles_.size()) {
            sourceIndex++;
            cumulative += particles_[sourceIndex].weight;
        }
        scratch_[outputIndex] = particles_[sourceIndex];
        scratch_[outputIndex].pose.x += gaussian(config_.rougheningSigmaX);
        scratch_[outputIndex].pose.y += gaussian(config_.rougheningSigmaY);
        scratch_[outputIndex].pose.theta =
            wrapAngle(scratch_[outputIndex].pose.theta + gaussian(config_.rougheningSigmaTheta));
        scratch_[outputIndex].weight = uniformWeight;
    }
    particles_.swap(scratch_);
}

const std::vector<Particle> &Filter::particles() const { return particles_; }

const std::vector<ObservationResult> &Filter::observationResults() const {
    return observationResults_;
}

const Config &Filter::config() const { return config_; }

} // namespace odyssey::mcl
