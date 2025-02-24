#include "autons.hpp"
#include <cmath>
#include <ios>
#include "EZ-Template/util.hpp"
#include "intake.hpp"
#include "lbArm.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 127;
const int SWING_SPEED = 127;

///
// Constants
///
void match_constants() 
{
    // P, I, D, and Start I
    chassis.pid_drive_constants_set(9.8, 0, 7.3);         // Fwd/rev constants, used for odom and non odom motions
    chassis.pid_heading_constants_set(5.5, 0, 30);        // Holds the robot straight while going forward without odom
    chassis.pid_turn_constants_set(4, 0.05, 21, 15);     // Turn in place constants
    chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
    chassis.pid_odom_angular_constants_set(6.5, 0.0, 50);    // Angular control for odom motions
    chassis.pid_odom_boomerang_constants_set(6, 0.0, 30);  // Angular control for boomerang motions

    // Exit conditions
    chassis.pid_turn_exit_condition_set(50_ms, 4_deg, 250_ms, 7_deg, 500_ms, 500_ms);
    chassis.pid_swing_exit_condition_set(50_ms, 4_deg, 250_ms, 7_deg, 500_ms, 500_ms);
    chassis.pid_drive_exit_condition_set(50_ms, 1.5_in, 250_ms, 3_in, 200_ms, 200_ms);
    chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
    chassis.pid_odom_drive_exit_condition_set(75_ms, 1.5_in, 250_ms, 3_in, 300_ms, 300_ms);
    chassis.pid_turn_chain_constant_set(4_deg);
    chassis.pid_swing_chain_constant_set(5_deg);
    chassis.pid_drive_chain_constant_set(3_in);

    // Slew constants
    chassis.slew_drive_set(true);
    chassis.slew_turn_constants_set(3_deg, 70);
    chassis.slew_drive_constants_set(5_in,70);
    chassis.slew_swing_constants_set(3_in, 80);

    // The amount that turns are prioritized over driving in odom motions
    // - if you have tracking wheels, you can run this higher.  1.0 is the max
    chassis.odom_turn_bias_set(0.9);

    chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
    chassis.odom_boomerang_distance_set(5_in);  // This sets the maximum distance away from target that the carrot point can be
    chassis.odom_boomerang_dlead_set(0.5);     // This handles how aggressive the end of boomerang motions are

    chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void skills_constants()
{
    chassis.pid_heading_constants_set(4.3, .5, 50);
    chassis.pid_drive_constants_set(13, 0, 21);
    chassis.pid_turn_constants_set(4, 0.05, 21, 15);
    chassis.pid_swing_constants_set(9, 0.5, 150);

    chassis.pid_turn_exit_condition_set(40_ms, 2_deg, 250_ms, 3_deg, 500_ms, 500_ms);
    chassis.pid_swing_exit_condition_set(35_ms, 3.5_deg, 250_ms, 7_deg, 500_ms, 500_ms);
    chassis.pid_drive_exit_condition_set(50_ms, 2_in, 250_ms, 3_in, 500_ms, 500_ms);

    chassis.pid_turn_chain_constant_set(3.5_deg);
    chassis.pid_swing_chain_constant_set(7_deg);
    chassis.pid_drive_chain_constant_set(5.5_in);
    chassis.slew_drive_set(false);
}

void pid_tuning()
{
    match_constants();

    chassis.pid_drive_set(48, 127);
    chassis.pid_wait();
}

void base_sawp_wq() 
{
    /* Initialization */
    chassis.odom_xyt_set(60, 17.125, 90);

    /* Move to alliance stake */
    chassis.pid_odom_set({{62, 15}, fwd, 115});
    chassis.pid_wait();

    /* Score on alliance stake */
    lbSetPosition(2);
    pros::delay(350);

    /* Move to mobile goal*/
    chassis.pid_odom_set({{45, 56}, rev, 127});
    chassis.pid_wait();
    
    /* Reset arm position */
    lbSetPosition(0);

    /* Clamp onto mobile goal */
    clampCylinder.set(true);

    pros::delay(100);

    /* Move to stack */
    chassis.pid_odom_set({{35, 60}, fwd, 127});
    
    /* Start intake after starting movement with delay */
    pros::delay(150);
    intake.move(127);
    chassis.pid_wait_quick_chain();
    
    /* Turn to face second ring */
    chassis.pid_turn_set(-95_deg, 127);
    chassis.pid_wait();

    /* Pick up rings from stack */
    chassis.pid_odom_set(14_in, 40);
    chassis.pid_wait_quick_chain();

    /* Move backwards to prepare for 3rd ring and prevent crossing*/
    chassis.pid_odom_set({{38, 53}, rev, 127});
    chassis.pid_wait_quick_chain();

    /* Move to 3rd ring */
    chassis.pid_odom_set({{30, 49}, fwd, 127});
    chassis.pid_wait_quick_chain();

    /* Stop intake to prevent turn from flinging */
    intake.move(0);

    /* Move to align with alliance stake stack */
    chassis.pid_odom_set({{60, 26}, fwd, 127});
    
    /* Restart intake after delay */
    pros::delay(600);
    intake.move(127);
    chassis.pid_wait_quick_chain();

    /* Pickup red and color sort blue from stack */
    chassis.pid_odom_set({{102, 22}, fwd, 55});
    chassis.pid_wait_quick_chain();

    /* Ensure red ring scored before dropping goal */
    pros::delay(100);

    /* Turn and throw mobile goal towards positive */
    chassis.pid_turn_set(-45_deg, 127);
    pros::delay(250);
    clampCylinder.set(false);
    chassis.pid_wait_quick_chain();
    
    /* Stop intake to prevent conveyer jam on goal */
    intake.move(0);

    /* Move to 2nd mobile goal */
    chassis.pid_odom_set({{98, 54}, rev, 127});
    chassis.pid_wait();

    /* Clamp onto mobile goal */
    clampCylinder.set(true);

    /* Raise arm early for touch*/
    lbSet(450);

    /* Move to 2nd mobile goal stack*/
    chassis.pid_odom_set({{118, 48}, fwd, 127});
    
    /* Restart intake */
    intake.move(127);
    chassis.pid_wait_quick_chain();
    
    /* Move backwards to touch, set constants full send for time */
    chassis.pid_drive_constants_set(14, 0, 25);
    chassis.pid_drive_set(-41_in, 127, false);
    chassis.pid_wait();
}

void red_sawp_wq()
{
    /* Initialize color sort */
    allianceColor = pros::Color::red;

    base_sawp_wq();
}

void blue_sawp_wq()
{
    /* Initialize color sort */
    allianceColor = pros::Color::blue;

    /* Initialization */
    chassis.odom_xyt_set(60, 127, 90);

    /* Move to alliance stake */
    chassis.pid_odom_set({{61.2, 128.6}, fwd, 115});
    chassis.pid_wait();

    /* Score on alliance stake */
    lbSetPosition(2);
    pros::delay(350);

    /* Move to mobile goal*/
    chassis.pid_odom_set({{44.2, 88}, rev, 110});
    chassis.pid_wait();
    
    /* Reset arm position */
    lbSetPosition(0);

    /* Clamp onto mobile goal */
    clampCylinder.set(true);
    pros::delay(100);

    /* Move to stack */
    chassis.pid_odom_set({{34, 82.3}, fwd, 127});
    
    /* Start intake after starting movement with delay */
    pros::delay(150);
    intake.move(127);
    chassis.pid_wait_quick_chain();
    
    /* Turn to face second ring */
    chassis.pid_turn_set(-85_deg, 127);
    chassis.pid_wait();

    /* Pick up rings from stack */
    chassis.pid_odom_set(14_in, 40);
    chassis.pid_wait_quick_chain();

    /* Move backwards to prepare for 3rd ring and prevent crossing*/
    chassis.pid_odom_set({{38, 91}, rev, 127});
    chassis.pid_wait_quick_chain();

    /* Move to 3rd ring */
    chassis.pid_odom_set({{30, 95}, fwd, 127});
    chassis.pid_wait_quick_chain();

    /* Stop intake to prevent turn from flinging */
    intake.move(0);

    /* Move to align with alliance stake stack */
    chassis.pid_odom_set({{60, 118}, fwd, 127});
    
    /* Restart intake after delay */
    pros::delay(600);
    intake.move(127);
    chassis.pid_wait_quick_chain();

    /* Pickup blue and color sort red from stack */
    chassis.pid_odom_set({{102, 122}, fwd, 48});
    chassis.pid_wait_quick_chain();

    /* Ensure blue ring scored before dropping goal */
    pros::delay(100);

    /* Turn and throw mobile goal towards positive */
    chassis.pid_turn_set(-135_deg, 127);
    pros::delay(250);
    clampCylinder.set(false);
    chassis.pid_wait_quick_chain();
    
    /* Stop intake to prevent conveyer jam on goal */
    intake.move(0);

    /* Move to 2nd mobile goal */
    chassis.pid_odom_set({{96.5, 86.5}, rev, 127});
    chassis.pid_wait();

    /* Clamp onto mobile goal */
    clampCylinder.set(true);

    /* Raise arm early for touch*/
    lbSet(450);
    pros::delay(75);
    /* Move to 2nd mobile goal stack*/
    chassis.pid_odom_set({{118, 96}, fwd, 127});
    
    /* Restart intake */
    pros::delay(175);
    intake.move(127);
    chassis.pid_wait_quick_chain();
    
    /* Move backwards to touch, set constants full send for time */
    chassis.pid_turn_set(80, 127);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_constants_set(14, 0, 25);
    chassis.pid_drive_set(-40_in, 127, false);
    chassis.pid_wait();
}

void base_ring_rush() 
{
    allianceColor = pros::Color::red;
}

void base_goal_side()
{
    allianceColor = pros::Color::red;

    /* Initialization */
    chassis.odom_xyt_set(96, 17, 180);
    
    chassis.pid_odom_set({{96, 46}, rev, 110});
    chassis.pid_wait();

    clampCylinder.set(true);
    pros::delay(100);
    intake.move(127);

    chassis.pid_odom_set(12, 127);
    chassis.pid_wait_quick_chain();

    chassis.pid_odom_set({{75, 59}, fwd, 110});
    chassis.pid_wait();
    rightDoinker.set(true);

    chassis.pid_odom_set({{77.5, 59.6}, fwd, 30});
    chassis.pid_wait();
    leftDoinker.set(true);
}

void base_goal_rush()
{
    allianceColor = pros::Color::red;
}

void skills()
{
    /* Initialization */
    chassis.odom_xyt_set(72, 17, 180);

    /* Score on alliance stake */
    lbSetPosition(2);
    pros::delay(700);
    lbSetPosition(0);
    pros::delay(100);
    
    /* Move towards first mobile goal */
    chassis.pid_odom_set({{105, 24.2}, rev, 127});
    chassis.pid_wait();

    /* Clamp on first mobile goal */
    clampCylinder.set(true);
    pros::delay(100);

    /* Move to first ring */
    chassis.pid_odom_set({{100, 45}, fwd, 127});
    pros::delay(200);
    intake.move(127);
    chassis.pid_wait_quick_chain();

    /* Move to second ring*/
    chassis.pid_odom_set({{121, 95}, fwd, 127});
    chassis.pid_wait();

    /* Raise LB to loading while backing up */
    chassis.pid_odom_set({{115, 70.5}, rev, 127});
    /* pros::delay(400);
    lbSetPosition(1);*/
    chassis.pid_wait_quick_chain();

    // TEMP NO WALL STAKE 
    /*
    // Move to ring in front of wall stake
    chassis.pid_odom_set({{122, 70.5}, fwd, 35});
    chassis.pid_wait();
    
    // Set more aggressive constants to align with wall stake better
    chassis.pid_drive_constants_set(16, 0 ,20);
    chassis.pid_odom_set(12_in, 127);
    chassis.pid_wait();

    // Delay before scoring//
    pros::delay(300);
    intake.move(-60); // Reverse to prevent jam
    lbSetPosition(2);
    pros::delay(600);

    // Reset constants to default
    match_constants();

    // Move backwards
    chassis.pid_odom_set({{122, 72}, rev, 127});
    chassis.pid_wait();
    
    // Raise LB to loading
    lbSetPosition(0);

    // Start intake   
    intake.move(127);*/

    /* Intake first of corner rings */
    chassis.pid_odom_set({{125, 40}, fwd, 127});
    chassis.pid_wait_quick_chain();

    /* Pick up the 2nd corner ring */
    chassis.pid_odom_set({{126, 12}, fwd, 52});
    chassis.pid_wait_quick_chain();

    /* Pick up the 3rd corner ring */
    chassis.pid_odom_set({{138, 26}, fwd, 50});
    chassis.pid_wait_quick_chain();
    pros::delay(200);

    // put in corner
    chassis.pid_odom_set({{140, 14}, rev, 127});
    pros::delay(300);
    intake.move(-127);
    pros::delay(100);
    clampCylinder.set(false);
    chassis.pid_wait_quick_chain();

    intake.move(127);

    chassis.pid_drive_constants_set(14, 0, 41); // increase kp for long movements

    chassis.pid_odom_set({{123, 100}, fwd, 127});
    // intake.move(127);

    chassis.pid_wait();

     // USE RIGHT DOINKER HERE ONCE WIRED
    rightDoinker.set(true);

    match_constants(); // reset pid constants for short movements

    chassis.pid_odom_set({{124, 108}, fwd, 60});
    chassis.pid_wait();

 // BRING UP RIGHT DOINKER HERE ONCE WIRED

    // get 2nd mogo
    chassis.pid_odom_set({{96, 127}, rev, 127});
    chassis.pid_wait();

    intake.move(127);

    clampCylinder.set(true);
    pros::delay(75);
    rightDoinker.set(false);

    leftDoinker.set(true);
    
    chassis.pid_odom_set({{127, 127}, fwd, 127});
    chassis.pid_wait_quick_chain();

    chassis.pid_turn_set(-125_deg, 127);
    chassis.pid_wait_quick_chain();
    
    intake.move(-127);
    leftDoinker.set(false);
    clampCylinder.set(false);
    
    chassis.pid_odom_set(-7, 127);
    chassis.pid_wait();

    chassis.pid_odom_set({{115, 113}, fwd, 127});
    intake.move(127);

    chassis.pid_wait_quick_chain();

    chassis.pid_odom_set({{75, 113}, rev, 95});
    chassis.pid_wait();

    intake.move(0);

    pros::delay(400);

    clampCylinder.set(true);
    pros::delay(400);

    chassis.pid_odom_set({{76, 113}, fwd, 127});
    intake.move_velocity(127);
    chassis.pid_wait();

    // pick up first ring

    chassis.pid_odom_set({{106, 93}, fwd, 70});
    chassis.pid_wait();

    pros::delay(500);

    // pick up ring in the middle

    /*chassis.pid_odom_set({{80, 60}, fwd, 127});
    chassis.pid_wait_quick_chain();*/

    // pick up the first of the 4 
    chassis.pid_odom_set({{74, 52}, fwd, 127});
    pros::delay(700);
    intake.move(0);
    chassis.pid_wait_quick_chain();
    // pick up the 2nd one
    chassis.pid_odom_set({{48, 15}, fwd, 30});
    intake.move(127);

    chassis.pid_wait_quick_chain();

    // pick up the 3rd
    
    chassis.pid_odom_set({{48, 0}, fwd, 40});
    chassis.pid_wait_quick_chain();

    // pick up thelast ring

    chassis.pid_odom_set({{30, 8}, fwd, 60});
    chassis.pid_wait();

    // move to corner

    chassis.pid_odom_set({{28, -4}, rev, 127});
    chassis.pid_wait();

    intake.move(-127);
    pros::delay(250);
    intake.move(0);

    clampCylinder.set(false);
    chassis.pid_wait();

    intake.move(95);
    // lbSetPosition(1); // load
    //chassis.pid_wait();

    // move to ring
    chassis.pid_odom_set({{36, 32}, fwd, 127});
    chassis.pid_wait();

    intake.move(0);

    chassis.pid_odom_set({{68, 6}, rev, 60});
    chassis.pid_wait();

    clampCylinder.set(true);
    chassis.pid_wait();

    pros::delay(200);

    intake.move(127);


    chassis.pid_odom_set({{39, 57}, fwd, 127});
    chassis.pid_wait();

    // get the ring
    chassis.pid_odom_set({{30, 75}, fwd, 127});
    chassis.pid_wait();

    pros::delay(200);


    // get the mext ring
    chassis.pid_odom_set({{52, 80}, fwd, 127});
    chassis.pid_wait();

    // get the mext ring
    chassis.pid_odom_set({{35, 95}, fwd, 80});
    chassis.pid_wait_quick_chain();

    pros::delay(200);


    // get the ring after that
    chassis.pid_odom_set({{28, 106}, fwd, 40});
    chassis.pid_wait_quick_chain();

    
    // back up and get the ring after that
    chassis.pid_odom_set({{32, 93}, rev, 70});
    chassis.pid_wait_quick_chain();

    chassis.pid_odom_set({{20, 99}, fwd, 70});
    rightDoinker.set(true);

    chassis.pid_wait_quick_chain();

    // back up and clear the corner
    chassis.pid_odom_set({{10, 80}, rev, 127});
    chassis.pid_wait_quick_chain();

    rightDoinker.set(false);

    leftDoinker.set(true);

    // clear corner
    chassis.pid_odom_set({{14, 109}, fwd, 80});
    chassis.pid_wait_quick_chain();

    leftDoinker.set(false);

    // place mogo in corner
    chassis.pid_odom_set({{2, 118}, rev, 90});
    pros::delay(200);

    intake.move(-127);
    chassis.pid_wait();
    chassis.pid_odom_set(5, 50);
    clampCylinder.set(false);
    chassis.pid_wait();
}