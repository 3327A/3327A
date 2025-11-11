#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/competition.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include "pros/serial.hpp"
#include "pros/misc.hpp"
#include "pros/imu.hpp"
#include "pros/adi.hpp"
#include "pros/adi.hpp"
#include <cstdlib>
#include <ctime>
#include <cmath>

// Motor definitions
pros::Motor frontMotorRight(16, pros::E_MOTOR_GEARSET_18, false);
pros::Motor middleMotorRight(17, pros::E_MOTOR_GEARSET_18, false);
pros::Motor backMotorRight(18, pros::E_MOTOR_GEARSET_18, false);

pros::Motor frontMotorLeft(15, pros::E_MOTOR_GEARSET_18, false);
pros::Motor middleMotorLeft(14, pros::E_MOTOR_GEARSET_18, false);
pros::Motor backMotorLeft(13, pros::E_MOTOR_GEARSET_18, false);

pros::MotorGroup rightMotors({frontMotorRight, middleMotorRight, backMotorRight});
pros::MotorGroup leftMotors({frontMotorLeft, middleMotorLeft, backMotorLeft});

// Intake/output
pros::Motor rubberBandThing(20, pros::E_MOTOR_GEARSET_18, false);
pros::Motor outputBelt(10, pros::E_MOTOR_GEARSET_06, false);

// Pneumatics
pros::ADIDigitalOut pistonA('A');
pros::ADIDigitalOut pistonB('B');
bool pistonAState = false;
bool pistonBState = false;

// Controller
pros::Controller Controller(pros::E_CONTROLLER_MASTER);

// Utility functions
int clamp(int value, int min_val, int max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

void togglePistonA() {
    pistonAState = !pistonAState;
    pistonA.set_value(pistonAState);
}

void togglePistonB() {
    pistonBState = !pistonBState;
    pistonB.set_value(pistonBState);
}

// Intake/output controls
void intakePressed() { rubberBandThing.move_velocity(100); }
void intakeReleased() { rubberBandThing.move_velocity(0); }
void intakeBackwardsPressed() { rubberBandThing.move_velocity(-100); }
void intakeBackwardsReleased() { rubberBandThing.move_velocity(0); }

void outputPressed() { outputBelt.move_velocity(100); }
void outputReleased() { outputBelt.move_velocity(0); }
void outputBackwardsPressed() { outputBelt.move_velocity(-100); }
void outputBackwardsReleased() { outputBelt.move_velocity(0); }

// Competition callbacks
void autonomous() {
    rightMotors.move_velocity(50 * 200 / 100); // 200 RPM max for 18:1 gearset
    leftMotors.move_velocity(50 * 200 / 100);
    pros::delay(15000); // 15 seconds
    rightMotors.move_velocity(0);
    leftMotors.move_velocity(0);
}

void opcontrol() {
    pistonA.set_value(false);
    pistonB.set_value(false);

    while (true) {
        int dir_move = Controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int dir_turn = Controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        int left_speed = clamp(dir_move + dir_turn, -100, 100);
        int right_speed = clamp(dir_move - dir_turn, -100, 100);

        leftMotors.move_velocity(left_speed * 200 / 100);
        rightMotors.move_velocity(right_speed * 200 / 100);

        // Controller button handling
        if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) intakePressed();
        else intakeReleased();

        if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) outputPressed();
        else outputReleased();

        if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intakeBackwardsPressed();
        else intakeBackwardsReleased();

        if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) outputBackwardsPressed();
        else outputBackwardsReleased();

        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) togglePistonA();
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) togglePistonB();

        pros::delay(10);
    }
}

// Pre-auton
void initializeRandomSeed() {
    std::srand(std::time(nullptr));
}

void initialize() {
    initializeRandomSeed();
}

// PROS competition setup
pros::Competition Competition;

int main() {
    initialize();
    Competition.autonomous(autonomous);
    Competition.operator_control(opcontrol);

    // For standalone testing
    if (!Competition.is_connected()) {
        opcontrol();
    }

    while (true) pros::delay(100);
}