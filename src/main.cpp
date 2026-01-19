#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>
#include <random>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motors/motor groups/pnuematics
pros::MotorGroup rightMotors({-14, 2, 3}, pros::MotorGearset::blue); // left motor group
pros::MotorGroup leftMotors({-8, 9, -10}, pros::MotorGearset::blue); // right motor group
pros::Motor Outtake(-4);
pros::Motor Intake(7);
pros::adi::DigitalOut Descorer('A');
pros::adi::DigitalOut Loader('B');
pros::adi::DigitalOut Middle_Goal('C');

// Inertial Sensor on port 10
pros::Imu imu(16);

// tracking wheels
pros::Rotation verticalEnc(-17);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 1);

pros::Rotation horizontalEnc(-1);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -1.75);

// Sensors
pros::Distance Back(14);
pros::Distance Right(15);
pros::Distance Left(16);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 13, lemlib::Omniwheel::OLD_325, 600, 8);

// lateral motion controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angular_controller(1.6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              15, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
// sensors for odometry
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

// input curves for driver control
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttleCurve, &steerCurve);

// single path asset
ASSET(Lower_Red_txt);
ASSET(Lower_Blue_txt);
ASSET(Upper_Red_txt);
ASSET(Upper_Blue_txt);

//========================================================================================
// AUTON SELECTOR CODE (with guard)
//========================================================================================

static bool descorerClosed = true;
static bool loaderClosed = true;
static bool middleGoalClosed = true;

//========================================================================================
// END OF AUTON SELECTOR CODE
//========================================================================================

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

// Global variable to track the selected autonomous mode
int selected_auton = 1;

void outtake(int delay_ms) {
    Intake.move_voltage(12000);
    Outtake.move_voltage(12000);
    pros::delay(delay_ms);
    Outtake.move_voltage(0);
    Intake.move_voltage(0);
}

ASSET(First_Long_Turn_txt);
ASSET(Second_Long_Turn_txt);
ASSET(Goal_To_Goal_Turn_txt);


void autonomous() {
    Descorer.set_value(descorerClosed);
    Loader.set_value(loaderClosed);
    Middle_Goal.set_value(middleGoalClosed);

    if (selected_auton == 0) {
        // LEFT SIDE
        Loader.set_value(false);
        Middle_Goal.set_value(true);
        Intake.move_voltage(12000);
        chassis.moveToPoint(-6, 45, 2500, {.maxSpeed=45});
        chassis.turnToHeading(-135, 500);
        chassis.moveToPoint(-31.5, 24, 1500);
        chassis.turnToHeading(180, 500);
        Loader.set_value(true);
        chassis.moveToPoint(-31.5, 4, 1500, {.maxSpeed=50});
        Intake.move_voltage(12000);
        pros::delay(2500);
        chassis.moveToPoint(-32, 49, 1500, {.forwards=false, .maxSpeed=40}, false);
        Outtake.move_voltage(12000);
        pros::delay(100000);
        Outtake.move_voltage(0);
    } else if (selected_auton == 1) {
        // RIGHT SIDE
        Loader.set_value(false);
        Middle_Goal.set_value(true);
        Intake.move_voltage(12000);
        chassis.moveToPoint(6, 45, 2500, {.maxSpeed=45});
        chassis.turnToHeading(135, 500);
        chassis.moveToPoint(31.5, 24, 1500);
        chassis.turnToHeading(180, 500);
        Loader.set_value(true);
        chassis.moveToPoint(32, 2.6125, 1500, {.maxSpeed=60});
        Intake.move_voltage(12000);
        pros::delay(2500);
        chassis.moveToPoint(33.5, 48, 1500, {.forwards=false, .maxSpeed=60}, false);
        Outtake.move_voltage(12000);
        pros::delay(100000);
        Outtake.move_voltage(0);
    } else if (selected_auton == 2) {
        // SKILLS
        Loader.set_value(false);
        Middle_Goal.set_value(true);
        chassis.setPose(-47, 0, 0);
        chassis.moveToPoint(-47, 47, 2000);
        chassis.turnToHeading(-90, 500);
        Intake.move_voltage(12000);
        Loader.set_value(true);
        chassis.moveToPoint(-67, 47, 2000, {.maxSpeed=60});
        pros::delay(2000);
        Intake.move_voltage(0);
        chassis.moveToPoint(-47, 47, 2000, {.forwards=false});
        chassis.follow(First_Long_Turn_txt, 5, 3000, true);
        chassis.turnToHeading(90, 500);
        chassis.moveToPoint(30, 47, 2000);
        outtake(2500);
        chassis.follow(Goal_To_Goal_Turn_txt, 5, 3000);
    }
}

void opcontrol() {
    Descorer.set_value(descorerClosed);
    Loader.set_value(loaderClosed);
    Middle_Goal.set_value(middleGoalClosed);

    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            chassis.setPose(0, 0, 0);
            Loader.set_value(true);
            chassis.moveToPoint(-14, 0, 500);
            Intake.move_voltage(12000);
            pros::delay(1500);
            chassis.moveToPoint(18, 0, 1000);
            Outtake.move_voltage(12000);
            pros::delay(3000);
        } else {
            int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            chassis.tank(leftY, rightY);

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                Intake.move_voltage(12000);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                Intake.move_voltage(-12000);
                Outtake.move_voltage(-12000);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                Intake.move_voltage(12000);
                Outtake.move_voltage(12000);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                middleGoalClosed = false;
                Middle_Goal.set_value(middleGoalClosed);
                Intake.move_voltage(12000);
                Outtake.move_voltage(12000);
            } else {
                Intake.move_voltage(0);
                Outtake.move(0);
                middleGoalClosed = true;
                Middle_Goal.set_value(middleGoalClosed);
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                descorerClosed = !descorerClosed;
                Descorer.set_value(descorerClosed);
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                loaderClosed = !loaderClosed;
                Loader.set_value(loaderClosed);
            }

            

            pros::delay(10);
        }
    }
}
