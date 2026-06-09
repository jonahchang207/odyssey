#include <cmath>

#include "odyssey/trackingwheel.hpp"

namespace odyssey {

TrackingWheel::TrackingWheel(pros::Rotation* encoder, float wheelDiameter, float offset,
                             float gearRatio)
    : type(Type::ROTATION),
      rotation(encoder),
      diameter(wheelDiameter),
      offset(offset),
      gearRatio(gearRatio) {}

TrackingWheel::TrackingWheel(pros::adi::Encoder* encoder, float wheelDiameter, float offset,
                             float gearRatio)
    : type(Type::ADI_ENCODER),
      adiEncoder(encoder),
      diameter(wheelDiameter),
      offset(offset),
      gearRatio(gearRatio) {}

TrackingWheel::TrackingWheel(pros::MotorGroup* motors, float wheelDiameter, float offset,
                             float driveRpm)
    : type(Type::MOTOR),
      motors(motors),
      diameter(wheelDiameter),
      offset(offset),
      gearRatio(1),
      rpm(driveRpm) {
    // measure motor position in rotations so the math below is simple
    motors->set_encoder_units_all(pros::v5::MotorUnits::rotations);
}

void TrackingWheel::reset() {
    switch (type) {
        case Type::ROTATION: rotation->reset_position(); break;
        case Type::ADI_ENCODER: adiEncoder->reset(); break;
        case Type::MOTOR: motors->tare_position_all(); break;
    }
}

float TrackingWheel::getDistanceTraveled() {
    const float circumference = diameter * static_cast<float>(M_PI);
    switch (type) {
        case Type::ROTATION:
            // get_position returns centidegrees
            return (rotation->get_position() / 36000.0f) * gearRatio * circumference;
        case Type::ADI_ENCODER:
            // get_value returns degrees
            return (adiEncoder->get_value() / 360.0f) * gearRatio * circumference;
        case Type::MOTOR: {
            // average all motors in the group, accounting for each motor's
            // cartridge so mixed cartridges still work
            const std::vector<double> positions = motors->get_position_all();
            const std::vector<pros::v5::MotorGears> gearsets = motors->get_gearing_all();
            float total = 0;
            int count = 0;
            for (size_t i = 0; i < positions.size(); i++) {
                float cartridgeRpm;
                switch (gearsets.at(i)) {
                    case pros::v5::MotorGears::red: cartridgeRpm = 100; break;
                    case pros::v5::MotorGears::green: cartridgeRpm = 200; break;
                    case pros::v5::MotorGears::blue: cartridgeRpm = 600; break;
                    default: cartridgeRpm = 200; break;
                }
                // wheel revolutions = motor revolutions * external ratio
                total += positions.at(i) * (rpm / cartridgeRpm) * circumference;
                count++;
            }
            return count > 0 ? total / count : 0;
        }
    }
    return 0;
}

float TrackingWheel::getOffset() const { return offset; }

} // namespace odyssey
