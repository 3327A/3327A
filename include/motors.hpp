#ifndef MOTORS_HPP
#define MOTORS_HPP

#include "api.h"
#include "motors.hpp"


auto clamp(int value, int min, int max) -> int;

// Pneumatic functions
auto togglePistonA(void) -> void;
auto togglePistonB(void) -> void;

// Intake functions
auto intakeForward(void) -> void;
auto intakeStop(void) -> void;
auto intakeBackward(void) -> void;

// Output functions
auto outputForward(void) -> void;
auto outputStop(void) -> void;
auto outputBackward(void) -> void;

// Drive functions
auto setLeftMotors(int speed) -> void;
auto setRightMotors(int speed) -> void;
auto stopLeftMotors(void) -> void;
auto stopRightMotors(void) -> void;
auto stopAllDrive(void) -> void;

/**
 * Moves the robot forward by a specified number of inches
 * Positive inches = forward, negative inches = backward
 */
auto move_in(int inches) -> void;

/**
 * Turns the robot left by a specified number of degrees
 */
auto turn_degrees_left(int degrees) -> void;

/**
 * Turns the robot right by a specified number of degrees
 */
auto turn_degrees_right(int degrees) -> void;

#endif // MOTORS_HPP
