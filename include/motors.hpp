#ifndef MOTORS_HPP
#define MOTORS_HPP

#include "api.h"
#include "motors.hpp"


int clamp(int value, int min, int max);

// Pneumatic functions
void togglePistonA();
void togglePistonB();

// Intake functions
void intakeForward();
void intakeStop();
void intakeBackward();

// Output functions
void outputForward();
void outputStop();
void outputBackward();

// Drive functions
void setLeftMotors(int speed);
void setRightMotors(int speed);
void stopLeftMotors();
void stopRightMotors();
void stopAllDrive();

/**
 * Moves the robot forward by a specified number of inches
 * Positive inches = forward, negative inches = backward
 */
void move_in(int inches);

/**
 * Turns the robot left by a specified number of degrees
 */
void turn_degrees_left(int degrees);

/**
 * Turns the robot right by a specified number of degrees
 */
void turn_degrees_right(int degrees);

#endif // MOTORS_HPP
