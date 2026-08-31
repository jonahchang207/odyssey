#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "odyssey/mcl.hpp"

namespace {

constexpr float kPi = 3.14159265358979323846f;

std::vector<std::string> split(const std::string &value, char delimiter) {
    std::vector<std::string> fields;
    std::stringstream stream(value);
    std::string field;
    while (std::getline(stream, field, delimiter))
        fields.push_back(field);
    return fields;
}

bool parseFloat(const std::string &value, float &output) {
    char *end = nullptr;
    output = std::strtof(value.c_str(), &end);
    return end != value.c_str() && *end == '\0' && std::isfinite(output);
}

odyssey::mcl::OdometryDelta deltaBetween(const odyssey::Pose &previous,
                                         const odyssey::Pose &current) {
    const float dx = current.x - previous.x;
    const float dy = current.y - previous.y;
    return {dx * std::cos(previous.theta) + dy * std::sin(previous.theta),
            dx * std::sin(previous.theta) - dy * std::cos(previous.theta),
            odyssey::mcl::shortestAngleDifference(current.theta, previous.theta)};
}

void usage(const char *program) {
    std::cerr << "usage: " << program
              << " TRACE.csv MIN_X MAX_X MIN_Y MAX_Y X:Y:YAW_DEG[:MIN:MAX:CONF] ...\n"
              << "CSV: timestamp_ms,odom_x,odom_y,compass_heading_deg,range_in,confidence,...\n";
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 7) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    float minX;
    float maxX;
    float minY;
    float maxY;
    if (!parseFloat(argv[2], minX) || !parseFloat(argv[3], maxX) || !parseFloat(argv[4], minY) ||
        !parseFloat(argv[5], maxY)) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    std::vector<odyssey::mcl::SensorModel> sensors;
    for (int argument = 6; argument < argc; ++argument) {
        const std::vector<std::string> fields = split(argv[argument], ':');
        if (fields.size() != 3 && fields.size() != 6) {
            usage(argv[0]);
            return EXIT_FAILURE;
        }
        odyssey::mcl::SensorModel sensor;
        float yawDegrees;
        if (!parseFloat(fields[0], sensor.offsetX) || !parseFloat(fields[1], sensor.offsetY) ||
            !parseFloat(fields[2], yawDegrees)) {
            usage(argv[0]);
            return EXIT_FAILURE;
        }
        sensor.yaw = yawDegrees * kPi / 180.0f;
        if (fields.size() == 6) {
            float confidence;
            if (!parseFloat(fields[3], sensor.minimumRange) ||
                !parseFloat(fields[4], sensor.maximumRange) || !parseFloat(fields[5], confidence)) {
                usage(argv[0]);
                return EXIT_FAILURE;
            }
            sensor.minimumConfidence = static_cast<int>(confidence);
        }
        sensors.push_back(sensor);
    }

    std::ifstream trace(argv[1]);
    if (!trace) {
        std::cerr << "could not open " << argv[1] << '\n';
        return EXIT_FAILURE;
    }

    odyssey::mcl::Config config;
    config.randomSeed = 1;
    odyssey::mcl::Filter filter(config, odyssey::mcl::FieldModel::rectangle(minX, maxX, minY, maxY),
                                sensors);

    std::cout << "timestamp_ms,odom_x,odom_y,odom_heading_deg,mcl_x,mcl_y,"
                 "mcl_heading_deg,sigma_x,sigma_y,sigma_heading_deg,neff,valid_sensors,"
                 "measurement_accepted,resampled,raycasts\n";
    std::string line;
    odyssey::Pose previous;
    bool initialized = false;
    while (std::getline(trace, line)) {
        const std::vector<std::string> fields = split(line, ',');
        if (fields.size() < 4 + sensors.size() * 2)
            continue;
        float timestamp;
        float x;
        float y;
        float compassHeading;
        if (!parseFloat(fields[0], timestamp) || !parseFloat(fields[1], x) ||
            !parseFloat(fields[2], y) || !parseFloat(fields[3], compassHeading))
            continue; // permits one header row

        const odyssey::Pose odometry(x, y, (90.0f - compassHeading) * kPi / 180.0f);
        if (!initialized) {
            filter.initialize(odometry);
            previous = odometry;
            initialized = true;
        }
        std::vector<odyssey::mcl::Observation> observations;
        observations.reserve(sensors.size());
        bool rowValid = true;
        for (std::size_t sensor = 0; sensor < sensors.size(); ++sensor) {
            float range;
            float confidence;
            if (!parseFloat(fields[4 + sensor * 2], range) ||
                !parseFloat(fields[5 + sensor * 2], confidence)) {
                rowValid = false;
                break;
            }
            observations.push_back({sensor, range, static_cast<int>(confidence)});
        }
        if (!rowValid)
            continue;

        const odyssey::mcl::StepStatus status =
            filter.step(deltaBetween(previous, odometry), observations);
        previous = odometry;
        std::cout << static_cast<std::uint32_t>(timestamp) << ',' << odometry.x << ',' << odometry.y
                  << ',' << compassHeading << ',' << status.estimate.pose.x << ','
                  << status.estimate.pose.y << ','
                  << 90.0f - status.estimate.pose.theta * 180.0f / kPi << ','
                  << std::sqrt(status.estimate.varianceX) << ','
                  << std::sqrt(status.estimate.varianceY) << ','
                  << status.estimate.sigmaTheta * 180.0f / kPi << ',' << status.effectiveSampleSize
                  << ',' << status.validSensorCount << ',' << status.measurementAccepted << ','
                  << status.resampled << ',' << status.raycastCount << '\n';
    }
}
