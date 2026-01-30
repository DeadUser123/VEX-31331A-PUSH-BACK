#include "main.h"
#include <cstdlib>
#include "globals.hpp"
#include "helper.hpp"
#include "pros/misc.h"

// * check globals.cpp for global variables and stuff, helper.cpp for helper functions, globals.hpp for paths (because ASSET(x) provides an extern)

std::string auton_state = "test"; // default auton state

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Intialized");
	pros::lcd::register_btn1_cb(on_center_button);
	chassis.calibrate();
	toggleMatchloader();
	toggleflap();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	if (auton_state == "test") {
		chassis.setPose(-62.307, 18.553, 90);
		chassis.follow(leftB1_txt, 15, 5000, true, false);
		chassis.turnToHeading(315, 5000);
		chassis.follow(leftB2_txt, 15, 5000, true, false);
		chassis.follow(leftB3_txt, 15, 5000, false, false);
		// chassis.turnToHeading(90, 10000);
	} else if (auton_state == "left") {
		toggleflap();
		toggleMatchloader();
		chassis.setPose(-62.532, 14.472, 90);
		chassis.follow(left1_txt, 15, 5000, true, false);
		chassis.turnToHeading(0, 1000);
		chassis.follow(left2_txt, 15, 5000, true, false);
		chassis.turnToHeading(270, 1000);
		chassis.follow(left3_txt, 15, 5000, true, false);
		intake.move(127);
		pros::delay(2000);
		intake.move(0);
		chassis.follow(left4_txt, 15, 5000, false, false);
		toggleflap();
		intake.move(127);
	} else if (auton_state == "right") {
		toggleflap();
		toggleMatchloader();
		chassis.setPose(-62.532, -14.472, 90);
		chassis.follow(right1_txt, 15, 5000, true, false);
		chassis.turnToHeading(180, 1000);
		chassis.follow(right2_txt, 15, 5000, true, false);
		chassis.turnToHeading(270, 1000);
		chassis.follow(right3_txt, 15, 5000, true, false);
		intake.move(127);
		pros::delay(2000);
		intake.move(0);
		chassis.follow(right4_txt, 15, 5000, false, false);
		toggleflap();
		intake.move(127);
	} else if (auton_state == "skills") {
		// TODO: add proper robot rotations and positionings and intakings between paths
		chassis.setPose(0, 0, 0);
		chassis.follow(planA1_txt, 15, 5000, true, false);
		chassis.turnToHeading(90, 5000);
		chassis.follow(planA2_txt, 15, 5000, true, false);
		chassis.turnToHeading(180+45, 5000);
		chassis.follow(planA3_txt, 15, 5000, true, false);
		chassis.turnToHeading(135, 5000);
		chassis.follow(planA4_txt, 15, 5000, true, false);
		chassis.turnToHeading(225, 5000);
		chassis.follow(planA5_txt, 15, 5000, true, false);
		chassis.turnToHeading(45, 5000);
		chassis.follow(planA6_txt, 15, 5000, true, false);
		chassis.turnToHeading(45, 5000);
		chassis.follow(planA7_txt, 15, 5000, true, false);
		chassis.turnToHeading(270, 500);
		chassis.follow(planA8_txt, 15, 5000, true, false);
		chassis.turnToHeading(0, 5000);
		chassis.follow(planA9_txt, 15, 5000, true, false);
		chassis.turnToHeading(90, 5000);
		chassis.follow(planA10_txt, 15, 5000, true, false);
		chassis.turnToHeading(0, 5000);
		chassis.follow(planA11_txt, 15, 5000, true, false);
		chassis.turnToHeading(90, 5000);
		chassis.follow(planA12_txt, 15, 5000, true, false);
		chassis.turnToHeading(270, 5000);
		chassis.follow(planA13_txt, 15, 5000, true, false);
	} else if (auton_state == "bskills") {
		chassis.setPose(0, 0, 0);
		chassis.arcade(127, 0);
		intake.move(127);
		pros::delay(5000);
		chassis.arcade(0, 0);
	} else if (auton_state == "move") {
		chassis.setPose(0, 0, 0);
		chassis.moveToPoint(0, 10, 5000);
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
	
	while (true) {
        // get left y and right y positions
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        // move the robot
        chassis.arcade(rightY, 0.8 * rightX);

		// intake.move(leftY);

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			intake.move(-127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			intake.move(127);
		} else {
			intake.move(0);
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			toggleflap();
		}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			toggleMatchloader();
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intakeMotor.move(127);
			middleIntake.move(-127);
			topIntake.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intakeMotor.move(-127);
			middleIntake.move(127);
			topIntake.move(-127);
		}
        pros::delay(25);
    }
}