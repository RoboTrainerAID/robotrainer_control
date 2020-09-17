 
#ifndef ROBOTRAINER_CONTROLLERS_FTS_CONTROLLERS_LED_DEFINES_HPP
#define ROBOTRAINER_CONTROLLERS_FTS_CONTROLLERS_LED_DEFINES_HPP

#include <iirob_led/BlinkyAction.h>

namespace robotrainer_controllers {

// TODO: use this namespace
// namespace led_defines {
    
// TODO: add real-time manager for actions
    

    
enum class controller_led_phases : std::uint8_t
{
    UNDEFINED = 0,
    UNLOCKED = 1,
    SHOW_FORCE = 2,

    // Adaptive Controller
    WAIT_FOR_INPUT = 10,
    WALK_FORWARD = 11,
    WALK_BACKWARDS = 12,
    ALMOST_RETURNED = 13,
    PHASE_FINISHED = 14,
    STEP_AWAY_FROM_ROBOT = 15,
    ROBOT_IN_AUTOMATIC_MOVEMENT = 16,
    SHOW_ADAPTED = 17
};

class LedBlinkyGoals
{
public:
    //general LED goals:
    iirob_led::BlinkyGoal blinkyStartGreen_;
    iirob_led::BlinkyGoal blinkyPhaseYellow_;
    iirob_led::BlinkyGoal blinkyFinishedRed_;
    iirob_led::BlinkyGoal blinkyFreeMovementBlue_;
    iirob_led::BlinkyGoal blinkyStepAway_;
    // dimension dependant goals:
    iirob_led::BlinkyGoal blinkyStart_x_;
    iirob_led::BlinkyGoal blinkyStart_x_back_;
    iirob_led::BlinkyGoal blinkyStart_y_left_;
    iirob_led::BlinkyGoal blinkyStart_y_right_;
    iirob_led::BlinkyGoal blinkyStart_rot_left_;
    iirob_led::BlinkyGoal blinkyStart_rot_right_;
    //legDistance and adaptX goals:
    iirob_led::BlinkyGoal blinkyWalkForward_;
    iirob_led::BlinkyGoal blinkyWalkBackwards_;
    iirob_led::BlinkyGoal blinkyAlmostFinishedRed_;
    //Automatic movement
    iirob_led::BlinkyGoal blinkyRobotAutomaticMovement_;
    iirob_led::BlinkyGoal blinkyRobotAutomaticMovement_x_;
    iirob_led::BlinkyGoal blinkyRobotAutomaticMovement_y_left_;
    iirob_led::BlinkyGoal blinkyRobotAutomaticMovement_y_right_;
    iirob_led::BlinkyGoal blinkyRobotAutomaticMovement_rot_left_;
    iirob_led::BlinkyGoal blinkyRobotAutomaticMovement_rot_right_;
    
    LedBlinkyGoals() 
    {
        /* Initialize Blinky messages */
        blinkyStartGreen_.color.r = 0.0;
        blinkyStartGreen_.color.g = 0.5;
        blinkyStartGreen_.color.b = 0.0;
        blinkyStartGreen_.color.a = 1.0;
        blinkyStartGreen_.blinks = 1;
        blinkyStartGreen_.duration_on = 3.0;
        blinkyStartGreen_.duration_off = 0.0;
        blinkyStartGreen_.start_led = 1;
        blinkyStartGreen_.end_led = 231;  // SR2: 384;
        blinkyStartGreen_.num_leds = 0;
        blinkyStartGreen_.fade_in = false;
        blinkyStartGreen_.fade_out = false;

        blinkyStart_x_ = blinkyStartGreen_;
        blinkyStart_x_.blinks = 100;
        blinkyStart_x_.duration_on = 0.25;
        blinkyStart_x_.duration_off = 0.05;
        blinkyStart_x_.start_led = 231-72; // SR2: 300;
        blinkyStart_x_.end_led = 231; // SR2: 384;

        blinkyStart_x_back_= blinkyStart_x_;
        blinkyStart_x_back_.start_led = 45; // SR2: 108;
        blinkyStart_x_back_.end_led = 44+72; // SR2: 192;

        blinkyStart_y_left_ = blinkyStart_x_;
        blinkyStart_y_left_.start_led = 0;
        blinkyStart_y_left_.end_led = 44; // SR2: 108;

        blinkyStart_y_right_ = blinkyStart_x_;
        blinkyStart_y_right_.start_led = 44+72+1; // SR2: 192;
        blinkyStart_y_right_.end_led = 44+72+43; // SR2: 300;

        blinkyStart_rot_left_ = blinkyStart_x_;
        blinkyStart_rot_left_.start_led = 210; // SR2: 361;
        blinkyStart_rot_left_.end_led = 27;

        blinkyStart_rot_right_ = blinkyStart_x_;
        blinkyStart_rot_right_.start_led = 44+72+43-16; // SR2: 279;
        blinkyStart_rot_right_.end_led = 44+72+43+15; // SR2: 327;

        blinkyWalkForward_ = blinkyStartGreen_;
        blinkyWalkForward_.blinks = 50;
        blinkyWalkForward_.duration_on = 0.6;
        blinkyWalkForward_.duration_off = 0.15;
        blinkyWalkForward_.start_led = blinkyStart_x_.start_led;
        blinkyWalkForward_.end_led = blinkyStart_x_.end_led;

        blinkyWalkBackwards_ = blinkyWalkForward_;
        blinkyWalkBackwards_.start_led = blinkyStart_x_back_.start_led;
        blinkyWalkBackwards_.end_led = blinkyStart_x_back_.end_led;

        blinkyPhaseYellow_.color.r = 0.5;
        blinkyPhaseYellow_.color.g = 0.5;
        blinkyPhaseYellow_.color.b = 0.0;
        blinkyPhaseYellow_.color.a = 1.0;
        blinkyPhaseYellow_.blinks = 2;
        blinkyPhaseYellow_.duration_on = 0.20;
        blinkyPhaseYellow_.duration_off = 0.05;
        blinkyPhaseYellow_.start_led = 1;
        blinkyPhaseYellow_.end_led = 231;
        blinkyPhaseYellow_.num_leds = 0;
        blinkyPhaseYellow_.fade_in = false;
        blinkyPhaseYellow_.fade_out = false;

        blinkyFinishedRed_.color.r =0.55;
        blinkyFinishedRed_.color.g = 0.0;
        blinkyFinishedRed_.color.b = 0.55;
        blinkyFinishedRed_.color.a = 1.0;
        blinkyFinishedRed_.blinks = 1;
        blinkyFinishedRed_.duration_on = 0.5;
        blinkyFinishedRed_.duration_off = 0.0;
        blinkyFinishedRed_.start_led = 1;
        blinkyFinishedRed_.end_led = 231;
        blinkyFinishedRed_.num_leds = 0;
        blinkyFinishedRed_.fade_in = false;
        blinkyFinishedRed_.fade_out = false;

        blinkyAlmostFinishedRed_ = blinkyFinishedRed_;
        blinkyAlmostFinishedRed_.blinks = 10;
        blinkyAlmostFinishedRed_.duration_on = 0.15;
        blinkyAlmostFinishedRed_.duration_off = 0.05;

        blinkyStepAway_ = blinkyFinishedRed_;
        blinkyStepAway_.color.r = 0.8;
        blinkyStepAway_.color.b = 0.3;
        blinkyStepAway_.blinks = 100;
        blinkyStepAway_.duration_on = 0.1;
        blinkyStepAway_.duration_off = 0.05;

        blinkyRobotAutomaticMovement_ = blinkyAlmostFinishedRed_;
        blinkyRobotAutomaticMovement_.color.r =0.8;
        blinkyRobotAutomaticMovement_.blinks = 200;

        blinkyRobotAutomaticMovement_x_ = blinkyRobotAutomaticMovement_;
        blinkyRobotAutomaticMovement_x_.start_led = blinkyStart_x_back_.start_led;
        blinkyRobotAutomaticMovement_x_.end_led = blinkyStart_x_back_.end_led;

        blinkyRobotAutomaticMovement_y_left_ = blinkyRobotAutomaticMovement_;
        blinkyRobotAutomaticMovement_y_left_.start_led = blinkyStart_y_right_.start_led;
        blinkyRobotAutomaticMovement_y_left_.end_led = blinkyStart_y_right_.end_led;

        blinkyRobotAutomaticMovement_y_right_ = blinkyRobotAutomaticMovement_;
        blinkyRobotAutomaticMovement_y_right_.start_led = blinkyStart_y_left_.start_led;
        blinkyRobotAutomaticMovement_y_right_.end_led = blinkyStart_y_left_.end_led;

        blinkyRobotAutomaticMovement_rot_left_ = blinkyRobotAutomaticMovement_;
        blinkyRobotAutomaticMovement_rot_left_.start_led = blinkyStart_rot_right_.start_led;
        blinkyRobotAutomaticMovement_rot_left_.end_led = blinkyStart_rot_right_.end_led;

        blinkyRobotAutomaticMovement_rot_right_ = blinkyRobotAutomaticMovement_;
        blinkyRobotAutomaticMovement_rot_right_.start_led = blinkyStart_rot_left_.start_led;
        blinkyRobotAutomaticMovement_rot_right_.end_led = blinkyStart_rot_left_.end_led;

        blinkyFreeMovementBlue_.color.r =0.11;
        blinkyFreeMovementBlue_.color.g = 0.56;
        blinkyFreeMovementBlue_.color.b = 1.0;
        blinkyFreeMovementBlue_.color.a = 0.4;
        blinkyFreeMovementBlue_.blinks = 1;
        blinkyFreeMovementBlue_.duration_on = 0.7;
        blinkyFreeMovementBlue_.duration_off = 0.0;
        blinkyFreeMovementBlue_.start_led = 1;
        blinkyFreeMovementBlue_.end_led = 231;
        blinkyFreeMovementBlue_.num_leds = 0;
        blinkyFreeMovementBlue_.fade_in = false;
        blinkyFreeMovementBlue_.fade_out = false;
        
    };
    
    ~LedBlinkyGoals() = default;
};

// }  // led_defines
    
}  // robotrainer_controllers

#endif  // ROBOTRAINER_CONTROLLERS_FTS_CONTROLLERS_LED_DEFINES_HPP
