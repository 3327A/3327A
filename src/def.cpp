#include "def.hpp"

// Motor definitions
pros::Motor frontMotorRight(16, pros::v5::MotorGears::blue);
pros::Motor backMotorRight(18, pros::v5::MotorGears::blue);
pros::Motor middleMotorRight(17, pros::v5::MotorGears::blue);
pros::Motor backMotorLeft(-13, pros::v5::MotorGears::blue);
pros::Motor frontMotorLeft(-15, pros::v5::MotorGears::blue);
pros::Motor middleMotorLeft(-14, pros::v5::MotorGears::blue);

// Intake and output motors
pros::Motor rubberBandThing(20, pros::v5::MotorGears::blue);
pros::Motor outputBelt(10, pros::v5::MotorGears::green);

// Pneumatics
pros::adi::DigitalOut pistonA('A');
pros::adi::DigitalOut pistonB('B');
bool pistonAState(false);
bool pistonBState(false);

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
