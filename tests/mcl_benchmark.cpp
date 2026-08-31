#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "odyssey/mcl.hpp"

int main() {
    constexpr float pi = 3.14159265358979323846f;
    odyssey::mcl::Config config;
    config.particleCount = 500;
    config.randomSeed = 42;
    config.initialSigmaX = 2;
    config.initialSigmaY = 2;
    config.initialSigmaTheta = 0.05f;

    std::vector<odyssey::mcl::SensorModel> sensors(4);
    sensors[1].yaw = pi / 2;
    sensors[2].yaw = pi;
    sensors[3].yaw = -pi / 2;
    odyssey::mcl::Filter filter(
        config, odyssey::mcl::FieldModel::rectangle(-70.2f, 70.2f, -70.2f, 70.2f), sensors);
    filter.initialize(odyssey::Pose(0, 0, 0));
    const std::vector<odyssey::mcl::Observation> observations{
        {0, 70.2f, 63}, {1, 70.2f, 63}, {2, 70.2f, 63}, {3, 70.2f, 63}};

    constexpr std::size_t iterations = 1000;
    std::uint64_t totalUs = 0;
    std::uint64_t maximumUs = 0;
    std::size_t raycasts = 0;
    for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
        const auto start = std::chrono::steady_clock::now();
        const odyssey::mcl::StepStatus status = filter.step({}, observations);
        const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - start)
                                 .count();
        totalUs += static_cast<std::uint64_t>(elapsed);
        maximumUs = std::max(maximumUs, static_cast<std::uint64_t>(elapsed));
        raycasts = status.raycastCount;
    }

    std::cout << "host_particle_count=" << config.particleCount << '\n'
              << "host_sensor_count=" << sensors.size() << '\n'
              << "raycasts_per_update=" << raycasts << '\n'
              << "host_average_us=" << totalUs / iterations << '\n'
              << "host_maximum_us=" << maximumUs << '\n';
}
