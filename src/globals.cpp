#include "main.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup rightMotors({20, 19, -18}, pros::MotorGearset::green);
pros::MotorGroup leftMotors({-11, -13, 12}, pros::MotorGearset::green);

pros::IMU imu(1);
pros::Rotation horizontal_encoder(2);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, 0.1);

pros::Rotation vertical_encoder(2);
lemlib::TrackingWheel vertical_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, 0.1);

// pros::Vision vision_sensor(0);

// dimensions: width 12.75, length 15.5 <-- Remeasure this from the middle of the middle wheels because that maye be a source of ERROR in the auton
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12.75, lemlib::Omniwheel::NEW_325, 400, 8);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              5, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0.4 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              0.1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);