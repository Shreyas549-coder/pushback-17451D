#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
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

void autonomous() {
    Descorer.set_value(descorerClosed);
    Loader.set_value(loaderClosed);
    Middle_Goal.set_value(middleGoalClosed);

    if (selected_auton == 0) {
        // Left Side auton
        Middle_Goal.set_value(true);
        Intake.move_voltage(12000);
        chassis.moveToPoint(-12, 48, 1500);
        Loader.set_value(true);
        chassis.turnToHeading(135, 500);
        chassis.moveToPoint(-36, 24, 1500);
        chassis.turnToHeading(180, 500);
        chassis.moveToPoint(-36, 0, 1500);
        Intake.move_voltage(12000);
        pros::delay(2500);
        Intake.move_voltage(0);
        chassis.moveToPoint(-36, 48, 1500, {.forwards=false});
        Outtake.move_voltage(12000);
        pros::delay(1500);
        Outtake.move_voltage(0);
    } else if (selected_auton == 1) {
        Loader.set_value(false);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 48, 2000);
        chassis.turnToHeading(-90, 500);
        Intake.move_voltage(12000);
        Loader.set_value(true);
        chassis.moveToPoint(-14, 48, 2500);
        pros::delay(2500);
        chassis.moveToPoint(0, 48, 2000, {.forwards=false});
        Intake.move_voltage(0);
        Loader.set_value(false); //Loader 1 Clear

        chassis.turnToHeading(180, 500);
        chassis.moveToPoint(0, 24, 1500);
        chassis.turnToHeading(90, 500);
        chassis.moveToPoint(84, 24, 3000);
        chassis.turnToHeading(0, 500);
        chassis.moveToPoint(84, 47, 3000);
        chassis.turnToHeading(90, 500);
        chassis.moveToPoint(60, 46.5, 3000, {.forwards=false});
        Outtake.move_voltage(12000);
        pros::delay(1500);
        Outtake.move_voltage(0); //Loader 1 Scored

        chassis.moveToPose(108, 46.5, 90,3000);
        Intake.move_voltage(12000);
        Loader.set_value(true);
        chassis.moveToPoint(84, 47, 3000, {.forwards=false});
        Intake.move_voltage(0);
        Loader.set_value(false); // Loader 2 Clear

        chassis.turnToHeading(180, 500);
        chassis.moveToPoint(96, -47, 3000);
        chassis.turnToHeading(90, 500);
        chassis.moveToPoint(60, -47, 3000, {.forwards=false});
        Outtake.move_voltage(12000);
        pros::delay(1500);
        Outtake.move_voltage(0); // Loader 2 Scored

        Loader.set_value(true);
        Intake.move_voltage(12000);
        chassis.moveToPose(107, -47, 90, 3000);
        Intake.move_voltage(12000);
        Loader.set_value(false);
        chassis.moveToPoint(84, -47, 3000, {.forwards=false}); // Loader 3 Clear

        chassis.turnToHeading(0, 500);
        chassis.moveToPoint(84, -34, 3000);
        chassis.turnToHeading(-90, 500);
        chassis.moveToPoint(0, -34, 3000);
        chassis.turnToHeading(180, 500);
        chassis.moveToPoint(0, -47, 3000);
        chassis.turnToHeading(-90, 500);
        chassis.moveToPoint(24, -47, 3000, {.forwards=false});
        Outtake.move_voltage(12000);
        pros::delay(1500);
        Outtake.move_voltage(0); // Loader 3 Scored

        Loader.set_value(true);
        chassis.moveToPoint(-14, -48, 3000);
        Intake.move_voltage(12000);
        chassis.moveToPoint(24, -47, 3000, {.forwards=false});
        Intake.move_voltage(12000);
        Loader.set_value(false); // Loader 4 Clear

        Outtake.move_voltage(12000);
        pros::delay(1500);
        Outtake.move_voltage(0); // Loader 4 Scored

        chassis.moveToPose(12, 0, 0, 3000);
        chassis.turnToHeading(90, 500);
        chassis.moveToPose(-24, 0, 0, 3000);
    } else if (selected_auton == 2) {

    } else if (selected_auton == 3) {
  
    } else if (selected_auton == 4) {
        chassis.setPose(0, 0, 0);
        chassis.turnToHeading(180, 9999);
    } else if (selected_auton == 5) {
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 12, 5000);
    } else if (selected_auton == 6) {
        pros::delay(15000);
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

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
                controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                autonomous();
            }

            pros::delay(10);
        }
    }
}
