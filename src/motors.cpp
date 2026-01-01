#include "api.h"
#include "motors.hpp"
#include "def.hpp"

int clamp(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
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

// Intake functions
void intakeForward() {
    rubberBandThing.move(127);
}

void intakeStop() {
    rubberBandThing.move(0);
}

void intakeBackward() {
    rubberBandThing.move(-127);
}

// Output functions
void outputForward() {
    outputBelt.move(127);
}

void outputStop() {
    outputBelt.move(0);
}

void outputBackward() {
    outputBelt.move(-127);
}

// Drive functions
void setLeftMotors(int speed) {
    frontMotorLeft.move(speed);
    backMotorLeft.move(speed);
    middleMotorLeft.move(speed);
}

void setRightMotors(int speed) {
    frontMotorRight.move(speed);
    backMotorRight.move(speed);
    middleMotorRight.move(speed);
}

void stopLeftMotors() {
    frontMotorLeft.move(0);
    backMotorLeft.move(0);
    middleMotorLeft.move(0);
}

void stopRightMotors() {
    frontMotorRight.move(0);
    backMotorRight.move(0);
    middleMotorRight.move(0);
}

void stopAllDrive() {
    stopLeftMotors();
    stopRightMotors();
}


/**
 * Moves the robot forward by a specified number of inches
 * Positive inches = forward, negative inches = backward
 */
void move_in(int inches) {
    // Calculate wheel circumference
    double circumference = WHEEL_DIAMETER * M_PI;
    
    // Calculate how many degrees the motor needs to rotate
    double rotations = (inches / circumference) * GEAR_RATIO;
    double degrees = rotations * 360.0;
    
    // Reset encoder positions
    frontMotorLeft.tare_position();
    frontMotorRight.tare_position();
    
    // Set target position
    int speed = (inches > 0) ? 80 : -80;  // Adjust speed as needed
    
    // Move motors
    setLeftMotors(speed);
    setRightMotors(speed);
    
    // Wait until target is reached
    while (fabs(frontMotorLeft.get_position()) < fabs(degrees)) {
        pros::delay(10);
    }
    
    // Stop motors
    stopAllDrive();
    pros::delay(100);  // Small delay for stability
}

/**
 * Turns the robot left by a specified number of degrees
 */
void turn_degrees_left(int degrees) {
    // Calculate arc length for the turn
    double arc_length = (WHEEL_BASE_WIDTH * M_PI * degrees) / 360.0;
    
    // Calculate wheel rotations needed
    double circumference = WHEEL_DIAMETER * M_PI;
    double wheel_rotations = (arc_length / circumference) * GEAR_RATIO;
    double motor_degrees = wheel_rotations * 360.0;
    
    // Reset encoder positions
    frontMotorLeft.tare_position();
    frontMotorRight.tare_position();
    
    // Turn left: left wheels backward, right wheels forward
    setLeftMotors(-60);   // Adjust speed as needed
    setRightMotors(60);
    
    // Wait until target is reached
    while (fabs(frontMotorRight.get_position()) < fabs(motor_degrees)) {
        pros::delay(10);
    }
    
    // Stop motors
    stopAllDrive();
    pros::delay(100);  // Small delay for stability
}

/**
 * Turns the robot right by a specified number of degrees
 */
void turn_degrees_right(int degrees) {
    // Calculate arc length for the turn
    double arc_length = (WHEEL_BASE_WIDTH * M_PI * degrees) / 360.0;
    
    // Calculate wheel rotations needed
    double circumference = WHEEL_DIAMETER * M_PI;
    double wheel_rotations = (arc_length / circumference) * GEAR_RATIO;
    double motor_degrees = wheel_rotations * 360.0;
    
    // Reset encoder positions
    frontMotorLeft.tare_position();
    frontMotorRight.tare_position();
    
    // Turn right: left wheels forward, right wheels backward
    setLeftMotors(60);    // Adjust speed as needed
    setRightMotors(-60);
    
    // Wait until target is reached
    while (fabs(frontMotorLeft.get_position()) < fabs(motor_degrees)) {
        pros::delay(10);
    }
    
    // Stop motors
    stopAllDrive();
    pros::delay(100);  // Small delay for stability
}

