#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"

std::int8_t rightMotor1Port = 20;
std::int8_t rightMotor2Port = 18;
std::int8_t rightMotor3Port = -8; // top

std::int8_t leftMotor1Port = -11;
std::int8_t leftMotor2Port = -13;
std::int8_t leftMotor3Port = 3; // top

std::int8_t intakeMotorPort = -10;
std::int8_t topIntakeMotorPort = 1;
std::int8_t middleIntakeMotorPort = -2;

std::int8_t horizontalEncoderPort = 5;
std::int8_t verticalEncoderPort = 6;
std::int8_t imuPort = 15;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Autonomous selector (0 = left, 1 = right, 2 = skills, 3 = test)
int auton_selector = 0;

// motor groups
pros::MotorGroup rightMotors({rightMotor1Port, rightMotor2Port, rightMotor3Port}, pros::MotorGearset::blue);
pros::MotorGroup leftMotors({leftMotor1Port,leftMotor2Port, leftMotor3Port}, pros::MotorGearset::blue);

// Set per-motor gearsets to support mixed cartridges (use a static initializer)
// namespace {
// struct MotorGearsInit {
//     MotorGearsInit() {
//         rightMotors.set_gearing({pros::MotorGearset::blue, pros::MotorGearset::blue, pros::MotorGearset::green});
//         leftMotors.set_gearing({pros::MotorGearset::blue, pros::MotorGearset::blue, pros::MotorGearset::green});
//     }
// } motorGearsInit;
// }

pros::Motor intakeMotor(intakeMotorPort, pros::MotorGearset::green);
pros::Motor topIntake(topIntakeMotorPort, pros::MotorGearset::green);
pros::Motor middleIntake(middleIntakeMotorPort, pros::MotorGearset::green);

pros::MotorGroup intake({intakeMotorPort, topIntakeMotorPort, middleIntakeMotorPort}, pros::MotorGearset::green);

pros::adi::DigitalOut matchloader('H');

pros::adi::DigitalOut flap('A');

// Sensors
pros::IMU imu(imuPort);
pros::Rotation horizontal_encoder(horizontalEncoderPort);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, 0.1);

pros::Rotation vertical_encoder(verticalEncoderPort);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, 0.625);

// dimensions: width 12.75, length 15.5 <-- Remeasure this from the middle of the middle wheels because that maye be a source of ERROR in the auton
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11.75, lemlib::Omniwheel::NEW_325, 450, 0);
// lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, nullptr); // IMEs
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, nullptr, nullptr, &imu); // tracking wheels

// * values (24, 160), (13, 110)
// lateral PID controller
lemlib::ControllerSettings lateral_controller(24, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              160, // derivative gain (kD)
                                              0, // anti windup
                                              0.5, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// ! NOT TESTED !
// angular PID controller
// * values: (9, 80), (4, 25)
// * possibles : (14, 140), (12, 105)
lemlib::ControllerSettings angular_controller(9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              80, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);


lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);