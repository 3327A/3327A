#include "main.h"

// TODO - update these
#define WHEEL_DIAMETER      4.0
#define WHEEL_BASE_WIDTH    12.0  
#define GEAR_RATIO          1.0 
// Gear ratio is (output wheel rotations) / (motor rotations)

// Motor definitions
pros::Motor frontMotorRight(16, pros::v5::MotorGears::blue);
pros::Motor backMotorRight(18, pros::v5::MotorGears::blue);
pros::Motor middleMotorRight(17, pros::v5::MotorGears::blue);
pros::Motor backMotorLeft(-13, pros::v5::MotorGears::blue);  // Negative port = reversed
pros::Motor frontMotorLeft(-15, pros::v5::MotorGears::blue);
pros::Motor middleMotorLeft(-14, pros::v5::MotorGears::blue);

// Intake and output motors
pros::Motor rubberBandThing(20, pros::v5::MotorGears::blue);
pros::Motor outputBelt(10, pros::v5::MotorGears::green);

// Pneumatics
pros::adi::DigitalOut pistonA('A');
pros::adi::DigitalOut pistonB('B');
bool pistonAState = false;
bool pistonBState = false;

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Helper functions
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

// NEW MOVEMENT FUNCTIONS

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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "I AM TIKING IT");
    
    // Make sure pistons start in down position
    pistonA.set_value(false);
    pistonB.set_value(false);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch.
 */
void competition_initialize() {}

/**
 * Runs the autonomous code.
 */
void autonomous() {
    // Example usage of the new functions:
    // move_in(24);              // Move forward 24 inches
    // turn_degrees_left(90);    // Turn left 90 degrees
    // move_in(12);              // Move forward 12 inches
    // turn_degrees_right(90);   // Turn right 90 degrees
}

/**
 * Runs the operator control code.
 */
void opcontrol() {
    // Make sure pistons are down
    pistonA.set_value(false);
    pistonB.set_value(false);
    
    // Button state tracking for toggles
    bool buttonUpPressed = false;
    bool buttonXPressed = false;
    
    for (;;) {
        // Get joystick values
        int dir_move = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int dir_turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        
        // Calculate motor speeds (arcade drive)
        int left_speed = clamp(dir_move + dir_turn, -127, 127);
        int right_speed = clamp(dir_move - dir_turn, -127, 127);
        
        // Drive motors
        setLeftMotors(left_speed);
        setRightMotors(right_speed);
        
        // Intake controls (R1 = forward, R2 = backward)
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intakeForward();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intakeBackward();
        } else {
            intakeStop();
        }
        
        // Output controls (L1 = forward, L2 = backward)
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            outputForward();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            outputBackward();
        } else {
            outputStop();
        }
        
        // Pneumatics toggle with button state tracking
        bool buttonUpCurrent = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        if (buttonUpCurrent && !buttonUpPressed) {
            togglePistonA();
        }
        buttonUpPressed = buttonUpCurrent;
        
        bool buttonXCurrent = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        if (buttonXCurrent && !buttonXPressed) {
            togglePistonB();
        }
        buttonXPressed = buttonXCurrent;
        
        pros::delay(10);
    }
}
