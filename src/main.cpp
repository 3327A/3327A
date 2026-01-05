#include "main.h"
#include "skills.hpp"
#include "autons.hpp"
#include "def.hpp"
#include "motors.hpp"

// Enums for auton selection
enum class Alliance {
    RED,
    BLUE,
    SKILLS,
    NONE
};

enum class Side {
    LEFT,
    RIGHT,
    NONE
};

// Global variables for auton selection
Alliance selected_alliance = Alliance::NONE;
Side selected_side = Side::NONE;

// Screen dimensions
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 240

// Button dimensions and positions
#define BUTTON_HEIGHT 80
#define BUTTON_WIDTH 150
#define SIDE_BUTTON_SIZE 100

// Alliance selection stage
bool alliance_selected = false;

// Forward declaration
auto auton_selector_task(void) -> void;

/**
 * Draws the alliance selection screen (Red, Blue, Skills)
 */
auto draw_alliance_selection(void) -> void {
     pros::screen::erase();

    // Red button (left)
    pros::screen::set_pen(pros::c::COLOR_RED);
    pros::screen::fill_rect(20, 80, 20 + BUTTON_WIDTH, 80 + BUTTON_HEIGHT);
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM_CENTER, 95, 110, "RED");

    // Blue button (middle)
    pros::screen::set_pen(pros::c::COLOR_BLUE);
    pros::screen::fill_rect(190, 80, 190 + BUTTON_WIDTH, 80 + BUTTON_HEIGHT);
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM_CENTER, 265, 110, "BLUE");

    // Skills button (right)
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::fill_rect(360, 80, 360 + BUTTON_WIDTH, 80 + BUTTON_HEIGHT);
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM_CENTER, 435, 110, "SKILLS");

    // Title
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE_CENTER, SCREEN_WIDTH/2, 20, "Select Alliance");
}

/**
 * Draws the side selection screen (Left/Yellow or Right/Purple)
 */
auto draw_side_selection(void) -> void {
    pros::screen::erase();

    // Yellow button with L (left side)
    pros::screen::set_pen(pros::c::COLOR_YELLOW);
    pros::screen::fill_rect(90, 70, 90 + SIDE_BUTTON_SIZE, 70 + SIDE_BUTTON_SIZE);
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE_CENTER, 140, 110, "L");

    // Purple button with R (right side)
    pros::screen::set_pen(pros::c::COLOR_PURPLE);
    pros::screen::fill_rect(290, 70, 290 + SIDE_BUTTON_SIZE, 70 + SIDE_BUTTON_SIZE);
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE_CENTER, 340, 110, "R");

    // Title
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE_CENTER, SCREEN_WIDTH/2, 20, "Select Side");
}

/**
 * Displays the final selection
 */
auto display_selection(void) -> void {
    pros::screen::erase();
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE_CENTER, SCREEN_WIDTH/2, 80, "Auton Selected:");

    std::string alliance_str;
    if (selected_alliance == Alliance::RED) alliance_str = "RED";
    else if (selected_alliance == Alliance::BLUE) alliance_str = "BLUE";
    else if (selected_alliance == Alliance::SKILLS) alliance_str = "SKILLS";

    std::string side_str = "";
    if (selected_side == Side::LEFT) side_str = " - LEFT";
    else if (selected_side == Side::RIGHT) side_str = " - RIGHT";

    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM_CENTER, SCREEN_WIDTH/2, 120, 
                       (alliance_str + side_str).c_str());
}

/**
 * Waits for touch to be released
 */
auto wait_for_release(void) -> void {
    while (true) {
        pros::screen_touch_status_s_t status = pros::screen::touch_status();
        if (status.touch_status == pros::E_TOUCH_RELEASED) {
            break;
        }
        pros::delay(20);
    }
    pros::delay(100); // Extra delay to ensure clean release
}

/**
 * Handles touch screen input for auton selection
 */
auto auton_selector_task(void) -> void {
    draw_alliance_selection();

    // Wait for any existing touch to be released before starting
    wait_for_release();

    while (true) {
        pros::screen_touch_status_s_t status = pros::screen::touch_status();

        if (status.touch_status == pros::E_TOUCH_PRESSED) {
            int x = status.x;
            int y = status.y;

            if (!alliance_selected) {
                // Check alliance buttons
                if (y >= 80 && y <= 160) {
                    // Red button
                    if (x >= 20 && x <= 170) {
                        selected_alliance = Alliance::RED;
                        alliance_selected = true;
                        wait_for_release();
                        draw_side_selection();
                    }
                    // Blue button
                    else if (x >= 190 && x <= 340) {
                        selected_alliance = Alliance::BLUE;
                        alliance_selected = true;
                        wait_for_release();
                        draw_side_selection();
                    }
                    // Skills button
                    else if (x >= 360 && x <= 510) {
                        selected_alliance = Alliance::SKILLS;
                        selected_side = Side::NONE;
                        alliance_selected = true;
                        wait_for_release();
                        display_selection();
                        break;
                    }
                }
            }
            else {
                // Check side buttons (only if not skills)
                if (y >= 70 && y <= 170) {
                    // Yellow/Left button
                    if (x >= 90 && x <= 190) {
                        selected_side = Side::LEFT;
                        wait_for_release();
                        display_selection();
                        break;
                    }
                    // Purple/Right button
                    else if (x >= 290 && x <= 390) {
                        selected_side = Side::RIGHT;
                        wait_for_release();
                        display_selection();
                        break;
                    }
                }
            }
        }

        pros::delay(20);
    }
}

/**
 * Callback for center button press - resets selection
 */
auto on_center_button(void) -> void {
    // Reset selection and restart selector
    selected_alliance = Alliance::NONE;
    selected_side = Side::NONE;
    alliance_selected = false;
    auton_selector_task();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
auto initialize(void) -> void {
    pros::lcd::initialize();
    pros::lcd::register_btn1_cb(on_center_button);
    
CheckScreen:
    if (pros::c::lcd_is_initialized()) {
        // Run the auton selector
        auton_selector_task();
    } else goto CheckScreen;
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
auto autonomous(void) -> void {
    pros::screen::erase();
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE_CENTER, SCREEN_WIDTH/2, 80, "Press A to start");

    while (!(controller.ButtonA.pressing())) { pros::delay(10); } // hang until button is pressed signifing that game started

    pros::screen::erase();
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE_CENTER, SCREEN_WIDTH/2, 80, "starting auton");


    if (selected_alliance == Alliance::RED) {
        if (selected_side == Side::LEFT) {
            red_left_auton();
        }
        else if (selected_side == Side::RIGHT) {
            red_right_auton();
        }
    }
    else if (selected_alliance == Alliance::BLUE) {
        if (selected_side == Side::LEFT) {
            blue_left_auton();
        }
        else if (selected_side == Side::RIGHT) {
            blue_right_auton();
        }
    }
    else if (selected_alliance == Alliance::SKILLS) {
        skills();
    }
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
auto disabled(void) -> void {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
auto competition_initialize(void) -> void {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
auto opcontrol(void) -> void {
    pros::screen::erase();
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE_CENTER, SCREEN_WIDTH/2, 80, "OP-CONTROL");

    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE_CENTER, SCREEN_WIDTH/2, 20, "TIKKKKK!");

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
