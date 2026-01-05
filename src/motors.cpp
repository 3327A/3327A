#include "api.h"
#include "motors.hpp"
#include "def.hpp"

auto clamp(int value, int min, int max) -> int {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

auto togglePistonA(void) -> void {
    pistonAState = !pistonAState;
    pistonA.set_value(pistonAState);
}

auto togglePistonB(void) -> void {
    pistonBState = !pistonBState;
    pistonB.set_value(pistonBState);
}

// Intake functions
auto intakeForward(void) -> void {
    rubberBandThing.move(127);
}

auto intakeStop(void) -> void {
    rubberBandThing.move(0);
}

auto intakeBackward(void) -> void {
    rubberBandThing.move(-127);
}

// Output functions
auto outputForward(void) -> void {
    outputBelt.move(127);
}

auto outputStop(void) -> void {
    outputBelt.move(0);
}

auto outputBackward(void) -> void {
    outputBelt.move(-127);
}

// Drive functions
auto setLeftMotors(int speed) -> void {
    frontMotorLeft.move(speed);
    backMotorLeft.move(speed);
    middleMotorLeft.move(speed);
}

auto setRightMotors(int speed) -> void {
    frontMotorRight.move(speed);
    backMotorRight.move(speed);
    middleMotorRight.move(speed);
}

auto stopLeftMotors(void) -> void {
    frontMotorLeft.move(0);
    backMotorLeft.move(0);
    middleMotorLeft.move(0);
}

auto stopRightMotors(void) -> void {
    frontMotorRight.move(0);
    backMotorRight.move(0);
    middleMotorRight.move(0);
}

auto stopAllDrive(void) -> void {
    stopLeftMotors();
    stopRightMotors();
}


/**
 * Moves the robot forward by a specified number of inches
 * Positive inches = forward, negative inches = backward
 */
auto move_in(int inches) -> void {
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
auto turn_degrees_left(int degrees) -> void{
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
auto turn_degrees_right(int degrees) -> void {
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

