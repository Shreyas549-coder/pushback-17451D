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
lemlib::ControllerSettings linearController(3, 0, 45, 3, 1, 100, 3, 500, 20);

// angular motion controller
lemlib::ControllerSettings angularController(0.6, 0, 5, 3, 1, 100, 3, 500, 0);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

// input curves for driver control
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

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
int selected_auton = 2;

void autonomous() {
    Descorer.set_value(descorerClosed);
    Loader.set_value(loaderClosed);
    Middle_Goal.set_value(middleGoalClosed);

    if (selected_auton == 0) {
        Middle_Goal.set_value(true);
        Intake.move_voltage(12000);
        chassis.moveToPoint(0, 30, 1500, {.maxSpeed = 50}, true);
        Loader.set_value(true);
        chassis.turnToHeading(-90, 800);
        chassis.moveToPoint(-32, 30, 2000, {.maxSpeed = 60});
        chassis.turnToHeading(-135, 500);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, -28, 1000, {.maxSpeed = 60}, true);
        Outtake.move_voltage(12000);
        pros::delay(1500);
        Outtake.move_voltage(0);
    } else if (selected_auton == 1) {
        Loader.set_value(true);
        chassis.setPose(0, 5, 0);
        chassis.moveToPose(24, -30, -90, 3000, {.forwards = false});
        pros::delay(1500);
        Outtake.move_voltage(12000);
        pros::delay(500);
        chassis.moveToPoint(0, -30, 750);
        chassis.turnToHeading(0, 750);
        Intake.move_voltage(12000);
        chassis.moveToPoint(3, -3, 500);
        chassis.turnToHeading(90, 500);
        chassis.moveToPoint(29, -3, 1000, {.maxSpeed = 60});
        pros::delay(1500);
        chassis.turnToPoint(40, 8, 500);
        chassis.moveToPoint(48, 16, 2000, {.maxSpeed = 20});
        pros::delay(250);
        Intake.move_voltage(-12000);
        pros::delay(2000);
        chassis.moveToPoint(28, -7, 1000, {.forwards = false});
        chassis.turnToPoint(28, 36, 500);
        chassis.moveToPoint(28, 28, 2000);
        Intake.move_voltage(12000);
        chassis.moveToPoint(28, 36, 1250, {.maxSpeed = 40});
        chassis.moveToPose(0, 69, -90, 1500);
        chassis.moveToPoint(18, 69, 1000, {.forwards = false});
        Outtake.move_voltage(12000);
        pros::delay(5000);
    } else if (selected_auton == 2) {
        Loader.set_value(false);
        chassis.setPose(0, 0, 0);
        Intake.move_voltage(12000);
        chassis.moveToPoint(-5, 34, 1500, {.maxSpeed = 40});
        chassis.moveToPoint(-5, 38, 1500, {.maxSpeed = 35}, true);
        pros::delay(250);
        Loader.set_value(true);
        chassis.turnToHeading(-170, 500);
        chassis.moveToPoint(-36, 0, 1500);
        chassis.turnToPoint(-38, 32, 1500, {.forwards = false});
        chassis.moveToPoint(-38, 32, 1500, {.forwards = false}, false);
        Outtake.move_voltage(12000);
        pros::delay(1500);
        Outtake.move_voltage(0);
        Loader.set_value(true);
        Intake.move_voltage(12000);
        chassis.moveToPoint(-34, -13.75, 2000, {.maxSpeed = 45}, true);
        chassis.moveToPoint(-34, -13.25, 500, {.forwards = false, .maxSpeed = 40});
        pros::delay(4000);
        Intake.move_voltage(0);
        chassis.moveToPoint(-33, 32, 1500, {.forwards = false, .maxSpeed = 80}, false);
        Intake.move_voltage(12000);
        Outtake.move_voltage(12000);
        pros::delay(1750);
    } else if (selected_auton == 3) {
        Loader.set_value(true);
        chassis.setPose(0, 0, 0);
        Intake.move_voltage(12000);
        chassis.moveToPoint(5, 34, 1500, {.maxSpeed = 40});
        chassis.moveToPoint(5, 38, 1500, {.maxSpeed = 35}, true);
        pros::delay(250);
        Loader.set_value(false);
        chassis.turnToHeading(145, 500);
        chassis.moveToPoint(34, 0, 1500);
        chassis.turnToHeading(180, 1500);
        chassis.moveToPoint(34, 32, 1500, {.forwards = false}, false);
        Outtake.move_voltage(12000);
        pros::delay(1500);
        Outtake.move_voltage(0);
        Loader.set_value(false);
        Intake.move_voltage(12000);
        chassis.moveToPoint(32.25, -9.5, 2000, {.maxSpeed = 40}, true);
        pros::delay(4000);
        Intake.move_voltage(0);
        chassis.moveToPoint(32, 32, 1500, {.forwards = false, .maxSpeed = 80}, false);
        Intake.move_voltage(12000);
        Outtake.move_voltage(12000);
        pros::delay(1750);
    } else if (selected_auton == 4) {
        chassis.setPose(0, 0, 0);
        chassis.turnToHeading(180, 9999);
    } else if (selected_auton == 5) {
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 48, 99999);
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
