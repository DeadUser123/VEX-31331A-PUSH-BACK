#include "lemlib/asset.hpp"
#include "main.h"
#include "pros/motors.hpp"

extern pros::Controller controller;
extern lemlib::Chassis chassis;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup leftMotors;

// Autonomous selector
extern int auton_selector;

extern pros::Motor intakeMotor;
extern pros::Motor topIntake;
extern pros::Motor middleIntake;
extern pros::MotorGroup intake;

extern pros::adi::DigitalOut matchloader;
extern pros::adi::DigitalOut flap;

ASSET(left1_txt); // full explanation: ASSET(x) is a macro and is basically the equivalent of #include x
ASSET(left2_txt);
ASSET(left3_txt);
ASSET(left4_txt);
ASSET(right1_txt);
ASSET(right2_txt);
ASSET(right3_txt);
ASSET(right4_txt);
ASSET(planA1_txt); // original autons
ASSET(planA2_txt);
ASSET(planA3_txt);
ASSET(planA4_txt);
ASSET(planA5_txt);
ASSET(planA6_txt);
ASSET(planA7_txt);
ASSET(planA8_txt);
ASSET(planA9_txt);
ASSET(planA10_txt);
ASSET(planA11_txt);
ASSET(planA12_txt);
ASSET(planA13_txt);
ASSET(thedream1_txt); // ideal scenario
ASSET(thedream2_txt);
ASSET(thedream3_txt);
ASSET(thedream4_txt);
ASSET(test_txt)

// 3
ASSET(leftB1_txt);
ASSET(leftB2_txt);
ASSET(leftB3_txt);

// 5 files
ASSET(CSkills1_txt);
ASSET(CSkills2_txt);
ASSET(CSkills3_txt);
ASSET(CSkills4_txt);
ASSET(CSkills5_txt);

// 23 files
ASSET(DSkills1_txt);
ASSET(DSkills2_txt);
ASSET(DSkills3_txt);
ASSET(DSkills4_txt);
ASSET(DSkills5_txt);
ASSET(DSkills6_txt);
ASSET(DSkills7_txt);
ASSET(DSkills8_txt);
ASSET(DSkills9_txt);
ASSET(DSkills10_txt);
ASSET(DSkills11_txt);
ASSET(DSkills12_txt);
ASSET(DSkills13_txt);
ASSET(DSkills14_txt);
ASSET(DSkills15_txt);
ASSET(DSkills16_txt);
ASSET(DSkills17_txt);
ASSET(DSkills18_txt);
ASSET(DSkills19_txt);
ASSET(DSkills20_txt);
ASSET(DSkills21_txt);
ASSET(DSkills22_txt);
ASSET(DSkills23_txt);