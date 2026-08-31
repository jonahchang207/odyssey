#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "odyssey/pose.hpp"

namespace odyssey::mcl {

/** A weighted hypothesis of the robot pose. Angles are math-frame radians. */
struct Particle {
    Pose pose;
    float weight = 0;
};

/** A finite, static field-map segment, in inches. */
struct Segment {
    float x1;
    float y1;
    float x2;
    float y2;
};

/** Static geometry used by distance-sensor raycasts. */
struct FieldModel {
    std::vector<Segment> segments;

    /** Build an axis-aligned perimeter from inside-wall coordinates. */
    static FieldModel rectangle(float minX, float maxX, float minY, float maxY);
};

/** Robot-relative distance-sensor mounting and validity parameters. */
struct SensorModel {
    /** sensor origin rightward from robot center, inches */
    float offsetX = 0;
    /** sensor origin forward from robot center, inches */
    float offsetY = 0;
    /** sensor yaw counterclockwise from robot forward, radians */
    float yaw = 0;
    float minimumRange = 0.8f;
    float maximumRange = 78.0f;
    /** confidence is ignored below confidenceMinimumRange */
    float confidenceMinimumRange = 7.874f;
    int minimumConfidence = 20;
    bool enabled = true;
};

/** One already-sampled distance reading. Range is in inches. */
struct Observation {
    std::size_t sensorIndex = 0;
    float range = 0;
    int confidence = 0;
};

/** Robot-relative incremental motion from the existing odometry. */
struct OdometryDelta {
    float forward = 0;
    float rightward = 0;
    float theta = 0;
};

struct MotionNoise {
    /** translation sigma per inch translated */
    float translationFromTranslation = 0.025f;
    /** translation sigma in inches per radian rotated */
    float translationFromRotation = 0.15f;
    /** rotation sigma per radian rotated */
    float rotationFromRotation = 0.04f;
    /** rotation sigma in radians per inch translated */
    float rotationFromTranslation = 0.002f;
};

struct MeasurementModel {
    float sigmaHit = 2.5f;
    float hitWeight = 0.90f;
    float randomWeight = 0.10f;
    float minimumLikelihood = 1.0e-6f;
    float maximumResidualForTrustedHit = 18.0f;
    std::size_t minimumValidSensors = 2;
};

/** Filter parameters. All distances are inches and all angles are radians. */
struct Config {
    std::size_t particleCount = 500;
    std::uint32_t randomSeed = 1;
    float initialSigmaX = 1.0f;
    float initialSigmaY = 1.0f;
    float initialSigmaTheta = 0.035f;
    MotionNoise motionNoise;
    MeasurementModel measurement;
    float resampleEffectiveSampleSizeRatio = 0.5f;
    float rougheningSigmaX = 0.03f;
    float rougheningSigmaY = 0.03f;
    float rougheningSigmaTheta = 0.001f;
};

enum class ObservationRejection {
    NONE,
    DISABLED,
    INVALID_INDEX,
    NONFINITE,
    OUT_OF_RANGE,
    LOW_CONFIDENCE,
    NO_MAP_INTERSECTION,
    EXPECTED_OUT_OF_RANGE,
    RESIDUAL_GATE
};

struct ObservationResult {
    Observation observation;
    float expectedRange = 0;
    float residual = 0;
    ObservationRejection rejection = ObservationRejection::NONE;
};

struct Estimate {
    Pose pose;
    float varianceX = 0;
    float varianceY = 0;
    /** circular standard deviation, radians */
    float sigmaTheta = 0;
};

struct StepStatus {
    Estimate estimate;
    float effectiveSampleSize = 0;
    float measurementQuality = 0;
    std::size_t validSensorCount = 0;
    std::size_t raycastCount = 0;
    bool measurementAccepted = false;
    bool weightsFinite = true;
    bool resampled = false;
};

/** Wrap an angle into [-pi, pi). */
float wrapAngle(float angle);

/** Shortest signed math-frame angular difference target - current. */
float shortestAngleDifference(float target, float current);

/**
 * Expected sensor-to-map range for a pose, or infinity when no positive
 * intersection exists. The sensor origin includes its mounting offset.
 */
float expectedRange(const Pose &pose, const SensorModel &sensor, const FieldModel &field);

/** Normalize log weights with max-subtraction. Returns false on recovery. */
bool normalizeLogWeights(const std::vector<float> &logWeights,
                         std::vector<float> &normalizedWeights);

float effectiveSampleSize(const std::vector<Particle> &particles);
Estimate weightedEstimate(const std::vector<Particle> &particles);

/** Production particle-filter core with no dependency on PROS or hardware. */
class Filter {
    public:
    Filter(Config config, FieldModel field, std::vector<SensorModel> sensors);

    void setSeed(std::uint32_t seed);
    void initialize(const Pose &mean);
    StepStatus step(const OdometryDelta &delta, const std::vector<Observation> &observations);

    const std::vector<Particle> &particles() const;
    const std::vector<ObservationResult> &observationResults() const;
    const Config &config() const;

    /** Deterministic O(N) systematic resampling of the current particles. */
    void systematicResample();

    private:
    Config config_;
    FieldModel field_;
    std::vector<SensorModel> sensors_;
    std::vector<Particle> particles_;
    std::vector<Particle> scratch_;
    std::vector<float> logWeights_;
    std::vector<float> normalizedWeights_;
    std::vector<std::size_t> usableObservationIndices_;
    std::vector<ObservationResult> observationResults_;
    std::mt19937 random_;
    Estimate estimate_;

    float gaussian(float sigma);
    void predict(const OdometryDelta &delta);
    bool selectObservations(const std::vector<Observation> &observations, StepStatus &status);
    bool applyMeasurement(StepStatus &status);
};

} // namespace odyssey::mcl
