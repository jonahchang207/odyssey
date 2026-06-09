#include "main.h"
#include "odyssey/api.hpp"

// =============================================================================
// Example Odyssey configuration
//
// Copy this into src/main.cpp of your PROS project and change the ports and
// dimensions to match your robot. Negative port numbers reverse the device.
// =============================================================================

// ---------------------------------------------------------------------------
// devices
// ---------------------------------------------------------------------------
pros::Controller master(pros::E_CONTROLLER_MASTER);

// drive motors. Left side reversed so positive = forward on both sides
pros::MotorGroup leftMotors({-1, -2, -3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({4, 5, 6}, pros::MotorGearset::blue);

// inertial sensor on port 10
pros::Imu imu(10);

// tracking wheel encoders
pros::Rotation verticalEncoder(11);    // vertical wheel, left of center
pros::Rotation horizontalEncoder(12);  // horizontal wheel, behind center

// tracking wheels: 2" wheels, vertical 1.25" left of center, horizontal
// 2.5" behind center
odyssey::TrackingWheel verticalWheel(&verticalEncoder, odyssey::Omniwheel::NEW_2, -1.25);
odyssey::TrackingWheel horizontalWheel(&horizontalEncoder, odyssey::Omniwheel::NEW_2, -2.5);

// ---------------------------------------------------------------------------
// chassis configuration
// ---------------------------------------------------------------------------
odyssey::Drivetrain drivetrain{
    &leftMotors,
    &rightMotors,
    11.5, // track width, inches
    odyssey::Omniwheel::NEW_325, // 3.25" omni wheels
    450,  // drive rpm
    2     // horizontal drift (2 = all omni wheels)
};

// lateral (driving) PID. Error in inches, output -127 to 127
odyssey::ControllerSettings lateralSettings{
    10,  // kP
    0,   // kI
    3,   // kD
    3,   // anti-windup range, inches
    1,   // small error range, inches
    100, // small error timeout, ms
    3,   // large error range, inches
    500, // large error timeout, ms
    20   // max output change per 10ms (slew)
};

// angular (turning) PID. Error in degrees, output -127 to 127
odyssey::ControllerSettings angularSettings{
    2,   // kP
    0,   // kI
    10,  // kD
    3,   // anti-windup range, degrees
    1,   // small error range, degrees
    100, // small error timeout, ms
    3,   // large error range, degrees
    500, // large error timeout, ms
    0    // slew (0 = disabled for turns)
};

odyssey::OdomSensors sensors{
    &verticalWheel, // vertical tracking wheel 1
    nullptr,        // vertical tracking wheel 2
    &horizontalWheel, // horizontal tracking wheel 1
    nullptr,        // horizontal tracking wheel 2
    &imu
};

// driver control input curves: small deadband, slight minimum output, gentle expo
odyssey::ExpoDriveCurve throttleCurve(3, 10, 1.019);
odyssey::ExpoDriveCurve steerCurve(3, 10, 1.019);

odyssey::Chassis chassis(drivetrain, lateralSettings, angularSettings, sensors,
                         &throttleCurve, &steerCurve);

// ---------------------------------------------------------------------------
// competition functions
// ---------------------------------------------------------------------------

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate(); // calibrate the IMU and start odometry

    // print the pose to the brain screen for debugging
    pros::Task screenTask([&]() {
        while (true) {
            odyssey::Pose pose = chassis.getPose();
            pros::lcd::print(0, "X: %.2f in", pose.x);
            pros::lcd::print(1, "Y: %.2f in", pose.y);
            pros::lcd::print(2, "Heading: %.2f deg", pose.theta);
            pros::delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // tell the robot where it starts on the field
    chassis.setPose(0, 0, 0);

    // drive to (0, 24) facing the point, then turn to 90 degrees
    chassis.moveToPoint(0, 24, 4000);
    chassis.turnToHeading(90, 1000);

    // drive into the pose (24, 24) arriving facing 90 degrees
    chassis.moveToPose(24, 24, 90, 4000);

    // raise an intake (for example) once the robot has driven 10 inches
    chassis.moveToPoint(48, 24, 4000);
    chassis.waitUntil(10);
    // intake.move(127);
    chassis.waitUntilDone();

    // drive backwards to the start
    chassis.moveToPoint(0, 0, 4000, {.forwards = false});
}

void opcontrol() {
    while (true) {
        const int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(throttle, turn);
        pros::delay(20);
    }
}
