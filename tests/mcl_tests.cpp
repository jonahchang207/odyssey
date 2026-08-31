#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "odyssey/mcl.hpp"

namespace {

constexpr float kPi = 3.14159265358979323846f;
int failures = 0;

void check(bool condition, const std::string &name) {
    if (!condition) {
        std::cerr << "FAIL: " << name << '\n';
        failures++;
    }
}

void near(float actual, float expected, float tolerance, const std::string &name) {
    check(std::isfinite(actual) && std::fabs(actual - expected) <= tolerance,
          name + " (actual=" + std::to_string(actual) + ", expected=" + std::to_string(expected) +
              ")");
}

odyssey::mcl::Config deterministicConfig(std::size_t particleCount = 100) {
    odyssey::mcl::Config config;
    config.particleCount = particleCount;
    config.randomSeed = 42;
    config.initialSigmaX = 0;
    config.initialSigmaY = 0;
    config.initialSigmaTheta = 0;
    config.motionNoise = {};
    config.motionNoise.translationFromTranslation = 0;
    config.motionNoise.translationFromRotation = 0;
    config.motionNoise.rotationFromRotation = 0;
    config.motionNoise.rotationFromTranslation = 0;
    config.rougheningSigmaX = 0;
    config.rougheningSigmaY = 0;
    config.rougheningSigmaTheta = 0;
    return config;
}

void testGeometry() {
    using namespace odyssey;
    using namespace odyssey::mcl;
    const FieldModel field = FieldModel::rectangle(-10, 10, -10, 10);
    SensorModel sensor;

    near(expectedRange(Pose(0, 0, 0), sensor, field), 10, 1.0e-4f, "ray intersects vertical wall");
    near(expectedRange(Pose(0, 0, kPi / 2), sensor, field), 10, 1.0e-4f,
         "ray intersects horizontal wall");
    near(expectedRange(Pose(0, 0, kPi / 4), sensor, field), std::sqrt(200.0f), 1.0e-3f,
         "ray intersects corner");

    FieldModel parallel;
    parallel.segments.push_back({-10, 1, 10, 1});
    check(!std::isfinite(expectedRange(Pose(0, 0, 0), sensor, parallel)),
          "parallel ray has no intersection");
    check(!std::isfinite(expectedRange(Pose(0, 0, 1.0e-8f), sensor, parallel)),
          "near-parallel ray is stable");

    sensor.offsetY = 2;
    near(expectedRange(Pose(0, 0, 0), sensor, field), 8, 1.0e-4f,
         "forward mounting offset changes origin");
    sensor.offsetY = 0;
    sensor.offsetX = 2;
    sensor.yaw = kPi / 2;
    near(expectedRange(Pose(0, 0, 0), sensor, field), 12, 1.0e-4f,
         "rightward mounting offset and yaw transform");
    near(expectedRange(Pose(0, 0, kPi / 2), sensor, field), 12, 1.0e-4f,
         "robot heading transforms sensor mount");
}

void testAnglesAndEstimate() {
    using namespace odyssey;
    using namespace odyssey::mcl;
    near(wrapAngle(3 * kPi), -kPi, 1.0e-5f, "angle normalization");
    near(shortestAngleDifference(-179 * kPi / 180, 179 * kPi / 180), 2 * kPi / 180, 1.0e-5f,
         "shortest angle through wrap");

    std::vector<Particle> particles{
        {Pose(1, 3, 179 * kPi / 180), 0.5f},
        {Pose(3, 5, -179 * kPi / 180), 0.5f},
    };
    const Estimate estimate = weightedEstimate(particles);
    near(estimate.pose.x, 2, 1.0e-5f, "weighted x mean");
    near(estimate.pose.y, 4, 1.0e-5f, "weighted y mean");
    near(std::fabs(estimate.pose.theta), kPi, 1.0e-4f, "circular heading mean");
}

void testNormalizationAndEss() {
    using namespace odyssey;
    using namespace odyssey::mcl;
    std::vector<float> weights;
    check(normalizeLogWeights({-1000, -1001}, weights), "log weights normalize");
    near(weights[0] + weights[1], 1, 1.0e-6f, "normalized weight sum");
    check(weights[0] > weights[1], "log normalization preserves ordering");

    check(!normalizeLogWeights(
              {-std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()},
              weights),
          "invalid log weights recover");
    near(weights[0], 0.5f, 1.0e-6f, "invalid weights recover uniformly");

    std::vector<Particle> particles{
        {Pose(), 0.25f}, {Pose(), 0.25f}, {Pose(), 0.25f}, {Pose(), 0.25f}};
    near(effectiveSampleSize(particles), 4, 1.0e-5f, "uniform ESS");
    particles[0].weight = 1;
    particles[1].weight = particles[2].weight = particles[3].weight = 0;
    near(effectiveSampleSize(particles), 1, 1.0e-5f, "collapsed ESS");
    particles[0].weight = std::numeric_limits<float>::quiet_NaN();
    near(effectiveSampleSize(particles), 0, 0, "NaN ESS protection");
}

void testPredictionAndInitialization() {
    using namespace odyssey;
    using namespace odyssey::mcl;
    const FieldModel field = FieldModel::rectangle(-20, 20, -20, 20);
    Config config = deterministicConfig(50);
    Filter filter(config, field, {});
    filter.initialize(Pose(1, 2, kPi / 2));
    StepStatus status = filter.step({4, 2, kPi / 2}, {});
    near(status.estimate.pose.x, -0.4142135f, 1.0e-4f, "particle motion x");
    near(status.estimate.pose.y, 6.2426405f, 1.0e-4f, "particle motion y");
    near(std::fabs(status.estimate.pose.theta), kPi, 1.0e-4f, "particle motion heading wraps");

    config.initialSigmaX = 2;
    config.initialSigmaY = 3;
    config.initialSigmaTheta = 0.2f;
    config.particleCount = 4000;
    Filter spreadFilter(config, field, {});
    spreadFilter.initialize(Pose(5, -4, 0.5f));
    const Estimate spread = weightedEstimate(spreadFilter.particles());
    near(spread.pose.x, 5, 0.12f, "initial x distribution mean");
    near(spread.pose.y, -4, 0.16f, "initial y distribution mean");
    near(spread.pose.theta, 0.5f, 0.02f, "initial heading distribution mean");
    near(std::sqrt(spread.varianceX), 2, 0.12f, "initial x sigma");
    near(std::sqrt(spread.varianceY), 3, 0.16f, "initial y sigma");
}

void testMeasurementsAndResampling() {
    using namespace odyssey;
    using namespace odyssey::mcl;
    const FieldModel field = FieldModel::rectangle(-10, 10, -10, 10);
    SensorModel front;
    SensorModel left;
    left.yaw = kPi / 2;
    SensorModel back;
    back.yaw = kPi;
    Config config = deterministicConfig(64);
    config.measurement.minimumValidSensors = 2;
    config.measurement.maximumResidualForTrustedHit = 5;
    config.resampleEffectiveSampleSizeRatio = 1.0f;
    Filter filter(config, field, {front, left, back});
    filter.initialize(Pose(0, 0, 0));

    StepStatus status = filter.step({}, {});
    check(!status.measurementAccepted && status.validSensorCount == 0,
          "no valid sensors skips measurement");

    status = filter.step({}, {{0, 10, 63}});
    check(!status.measurementAccepted && status.validSensorCount == 1,
          "single sensor rejected when minimum is two");

    status = filter.step({}, {{0, 10, 63}, {1, 10, 63}});
    check(status.measurementAccepted && status.validSensorCount == 2,
          "multiple consistent sensors accepted");
    check(status.weightsFinite, "measurement weights remain finite");

    status = filter.step({}, {{0, 10, 63}, {1, 10, 63}, {2, 1, 63}});
    check(status.measurementAccepted && status.validSensorCount == 2,
          "one extreme outlier is gated while consistent sensors remain usable");

    status = filter.step({}, {{0, std::numeric_limits<float>::quiet_NaN(), 63}, {1, 10, 0}});
    check(!status.measurementAccepted && status.validSensorCount == 0,
          "invalid and low-confidence readings rejected");

    Config spreadConfig = deterministicConfig(200);
    spreadConfig.initialSigmaX = 4;
    spreadConfig.measurement.minimumValidSensors = 1;
    spreadConfig.measurement.maximumResidualForTrustedHit = 20;
    spreadConfig.resampleEffectiveSampleSizeRatio = 1.0f;
    Filter resampleFilter(spreadConfig, field, {front});
    resampleFilter.initialize(Pose(0, 0, 0));
    status = resampleFilter.step({}, {{0, 10, 63}});
    check(status.measurementAccepted, "single valid sensor can be configured");
    check(status.resampled, "systematic resampling triggers below ESS threshold");
    near(effectiveSampleSize(resampleFilter.particles()), 200, 1.0e-2f,
         "systematic resampling restores uniform weights");
}

} // namespace

int main() {
    testGeometry();
    testAnglesAndEstimate();
    testNormalizationAndEss();
    testPredictionAndInitialization();
    testMeasurementsAndResampling();

    if (failures != 0) {
        std::cerr << failures << " test(s) failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "All MCL tests passed\n";
    return EXIT_SUCCESS;
}
