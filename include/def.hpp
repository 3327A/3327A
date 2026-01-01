#ifndef DEF_HPP
#define DEF_HPP

#include "api.h"

// TODO - update these
#define WHEEL_DIAMETER      4.0
#define WHEEL_BASE_WIDTH    12.0
#define GEAR_RATIO          1.0
// Gear ratio is (output wheel rotations) / (motor rotations)

// Motor definitions
extern pros::Motor frontMotorRight;
extern pros::Motor backMotorRight;
extern pros::Motor middleMotorRight;
extern pros::Motor backMotorLeft;
extern pros::Motor frontMotorLeft;
extern pros::Motor middleMotorLeft;

// Intake and output motors
extern pros::Motor rubberBandThing;
extern pros::Motor outputBelt;

// Pneumatics
extern pros::adi::DigitalOut pistonA;
extern pros::adi::DigitalOut pistonB;
extern bool pistonAState;
extern bool pistonBState;

// Controller
extern pros::Controller controller;

#endif // DEF_HPP
