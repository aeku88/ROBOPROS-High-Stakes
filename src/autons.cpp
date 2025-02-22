#include "autons.hpp"
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
    chassis.pid_drive_constants_set(9.5, 0, 6.5);         // Fwd/rev constants, used for odom and non odom motions
    chassis.pid_heading_constants_set(5.5, 0, 30);        // Holds the robot straight while going forward without odom
    chassis.pid_turn_constants_set(4, 0.05, 21, 15);     // Turn in place constants
    chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
    chassis.pid_odom_angular_constants_set(6.5, 0.0, 50);    // Angular control for odom motions
    chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

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
    chassis.slew_drive_constants_set(5_in,50);
    chassis.slew_swing_constants_set(3_in, 80);

    // The amount that turns are prioritized over driving in odom motions
    // - if you have tracking wheels, you can run this higher.  1.0 is the max
    chassis.odom_turn_bias_set(0.9);

    chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
    chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
    chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

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
    allianceColor = pros::Color::red;
    chassis.odom_xyt_set(60, 17.125, 90);

    chassis.pid_odom_set({{62, 15}, fwd, 100});
    chassis.pid_wait();

    lbSetPosition(2);

    pros::delay(400);

    chassis.pid_odom_set({{48, 54}, rev, 127});
    chassis.pid_wait();

    lbSetPosition(0);

    clampCylinder.set(true);

    pros::delay(250);

    intake.move(127);

    chassis.pid_odom_set({{35, 59}, fwd, 127});
    chassis.pid_wait_quick_chain();

    chassis.pid_turn_set(-90, 127);
    chassis.pid_wait();

    chassis.pid_odom_set(12_in, 40);
    chassis.pid_wait_quick_chain();

    chassis.pid_odom_set({{38, 53}, rev, 127});
    chassis.pid_wait_quick_chain();

    chassis.pid_odom_set({{28, 48}, fwd, 127});
    chassis.pid_wait();

    chassis.pid_odom_set({{52, 23}, fwd, 127});
    intake.move(0);
    pros::delay(300);
    intake.move(127);
    chassis.pid_wait();

    chassis.pid_odom_set({{95, 23}, fwd, 80});
    chassis.pid_wait();

    clampCylinder.set(false);

    pros::delay(150);

    intake.move(0);

    chassis.pid_odom_set({{98, 50}, rev, 127});
    chassis.pid_wait();

    clampCylinder.set(true);

    pros::delay(300);

    intake.move(127);

    chassis.pid_odom_set({{115, 50}, fwd, 127});
    chassis.pid_wait_quick_chain();
    
    chassis.pid_odom_set({{90, 60}, rev, 127});
    chassis.pid_wait();
}

void base_ring_rush() 
{
    allianceColor = pros::Color::red;
   
}

void base_goal_side()
{
    allianceColor = pros::Color::red;
}

void base_goal_rush()
{
    allianceColor = pros::Color::red;
}

void skills()
{
    skills_constants();
    chassis.odom_xyt_set(72, 17, 180);

    lbSetPosition(2);
    pros::delay(800);
    lbSetPosition(0);
    pros::delay(100);
    //  
    chassis.pid_odom_set({{102, 24}, rev, 75});
    chassis.pid_wait();
    pros::delay(200);

    clampCylinder.set(true);
    chassis.pid_wait();

    chassis.pid_odom_set({{97, 50}, fwd, 110});
    pros::delay(140);
    intake.move(127);
    chassis.pid_wait();
    pros::delay(100);

    chassis.pid_odom_set({{121, 95}, fwd, 110});
   // lbSetPosition(1);
    //lbSet(185);
    chassis.pid_wait();

    // pros::delay(800);

    chassis.pid_odom_set({{118, 72.5}, rev, 110});
    chassis.pid_wait();
    lbSetPosition(1);
  //  pros::delay(700);

    /*
    intakeMotors.move_relative(-10, 600);
    while (intakeMotors.get_position() != intakeMotors.get_target_position())
        pros::delay(5);
    */

   // pros::delay(200);
/* 
    intake.move(-50);
    pros::delay(80);
    intake.move(0);
    pros::delay(500);

    // armControlCopy->setMaxVelocity(60);
    lbPID.constants_set(.1, 0, 0.07); // increase deceleration

    // lbSet(700);
    pros::delay(300);
    intake.move(127);

    chassis.pid_odom_set({{134, 69}, fwd, 35});
    chassis.pid_wait();

    // armControlCopy->setMaxVelocity(200);
    lbPID.constants_set(.7, 0.0, 0.7); // revert to normal
*/

    // score on wallstake
    chassis.pid_odom_set({{135, 72.5}, fwd, 30});
    chassis.pid_wait();

    intake.move(127);
    pros::delay(200);
    intake.move(-30);
    pros::delay(70);
    intake.move(0);
    pros::delay(300);

    chassis.pid_odom_set({{136, 72.5}, fwd, 110});
    lbSetPosition(2);

    chassis.pid_wait();

    pros::delay(200);

    // back up and arm down
    lbSetPosition(0);
    chassis.pid_odom_set({{126, 73}, rev, 110});
    chassis.pid_wait();

    // turn towards the last 4 rings
    intake.move(127);

    chassis.pid_odom_set({{126, 73}, fwd, 110});
    chassis.pid_wait();

    // pick up 2nd ring
    chassis.pid_odom_set({{126, 25}, fwd, 50});
    chassis.pid_wait();

    // pick up 3rd ring
    chassis.pid_odom_set({{126, 14}, fwd, 40});
    chassis.pid_wait();

    // pick up last ring
    chassis.pid_odom_set({{135, 24}, fwd, 50});
    chassis.pid_wait();

    // put in corner
    chassis.pid_odom_set({{140, 11}, rev, 90});
    chassis.pid_wait();

    intake.move(-127);

    pros::delay(160);
    intake.move(0);

    clampCylinder.set(false);
    chassis.pid_wait();
    
  //  chassis.pid_drive_constants_set(16, 0, 36); // increase kp for long movements

    chassis.pid_odom_set({{124, 98}, fwd, 127});
    intake.move(127);

    lbSetPosition(1);
    chassis.pid_wait();

     // USE RIGHT DOINKER HERE ONCE WIRED
   // leftDoinker.set(true);

  //  skills_constants(); // reset pid constants for short movements

    chassis.pid_odom_set({{124, 108}, fwd, 60});
    chassis.pid_wait();

    pros::delay(300);

   // leftDoinker.set(false);
 // BRING UP RIGHT DOINKER HERE ONCE WIRED

    // get 2nd mogo
    chassis.pid_odom_set({{98, 126}, rev, 60});
    chassis.pid_wait();

    clampCylinder.set(true);
    chassis.pid_wait();

    intake.move(0);

   leftDoinker.set(true);
    
    chassis.pid_odom_set({{120, 125}, fwd, 110});
    chassis.pid_wait();

    chassis.pid_odom_set({{120, 115}, fwd, 110});
    chassis.pid_wait();

    leftDoinker.set(false);

    chassis.pid_odom_set({{130, 122}, rev, 70});
    chassis.pid_wait();

    clampCylinder.set(false);
    chassis.pid_wait();

    pros::delay(300);

    chassis.pid_odom_set({{115, 108}, fwd, 110});
    chassis.pid_wait();

    chassis.pid_odom_set({{74, 108}, rev, 60});
    chassis.pid_wait();

    clampCylinder.set(true);
    pros::delay(300);
    chassis.pid_wait();

    chassis.pid_odom_set({{74, 123}, fwd, 60});
    chassis.pid_wait();

    chassis.pid_odom_set({{74, 116}, rev, 110});
    chassis.pid_wait();

    // armControlCopy->setMaxVelocity(200);

    intake.move(127);
    pros::delay(100);
    intake.move(-30);
    pros::delay(70);
    intake.move(0);
    pros::delay(300);

    // armControlCopy->setMaxVelocity(65);
    lbSetPosition(2); // score
    pros::delay(700);

    lbSetPosition(0); // reset
    pros::delay(200);

    // back up

    chassis.pid_odom_set({{76, 106}, rev, 110});
    chassis.pid_wait();

    intake.move(127);

    // pick up first ring

    chassis.pid_odom_set({{102, 79}, fwd, 80});
    chassis.pid_wait();

    // pick up ring in the middle

    chassis.pid_odom_set({{80, 60}, fwd, 110});
    chassis.pid_wait();

    // pick up the first of the 4 

    chassis.pid_odom_set({{64, 43}, fwd, 110});
    chassis.pid_wait();

    // pick up the 2nd one
    chassis.pid_odom_set({{38, 15}, fwd, 70});
    chassis.pid_wait();
    // pick up the 3rd
    
    chassis.pid_odom_set({{37, -5}, fwd, 70});
    chassis.pid_wait();

    // pick up thelast ring

    chassis.pid_odom_set({{20, 9}, fwd, 70});
    chassis.pid_wait();

    // move to corner

    chassis.pid_odom_set({{18, -8}, rev, 70});
    chassis.pid_wait();

    intake.move(-127);
    pros::delay(160);
    intake.move(0);

    clampCylinder.set(false);
    chassis.pid_wait();

    intake.move(60);
    // lbSetPosition(1); // load
    //chassis.pid_wait();

    // move to ring
    chassis.pid_odom_set({{29, 30}, fwd, 110});
    chassis.pid_wait();

    intake.move(0);

    // move to mogo
    chassis.pid_odom_set({{60, 5}, rev, 60});
    chassis.pid_wait();

    intake.move(127); // score the ring we just picked up

    clampCylinder.set(true);
    chassis.pid_wait();
    // intake.move(0);

    pros::delay(200);

    chassis.pid_odom_set({{30, 49}, fwd, 110});
    pros::delay(200);
    chassis.pid_wait();
    lbSetPosition(1); // load

    // armControlCopy->setMaxVelocity(60);

    //lbSet(500); // hover
    pros::delay(300);
    intake.move(127);

    chassis.pid_odom_set({{13, 49}, fwd, 30});
    chassis.pid_wait();

    // armControlCopy->setMaxVelocity(200);

    // score on wallstake
    intake.move(127);
    pros::delay(100);
    intake.move(-30);
    pros::delay(70);
    intake.move(0);
    pros::delay(300); 

    lbSetPosition(2); // score
    pros::delay(700);

    // back up and arm down
    intake.move(127);

    lbSetPosition(0); // reset
    pros::delay(300);

    chassis.pid_odom_set({{26, 46}, rev, 110});
    chassis.pid_wait();

    // get the ring
    chassis.pid_odom_set({{26, 75}, fwd, 110});
    chassis.pid_wait();

    // get the mext ring
    chassis.pid_odom_set({{52, 85}, fwd, 110});
    chassis.pid_wait();

    // get the mext ring
    chassis.pid_odom_set({{23, 95}, fwd, 110});
    chassis.pid_wait();

    // get the ring after that
    chassis.pid_odom_set({{22, 108}, fwd, 80});
    chassis.pid_wait();

    // back up and get the ring after that
    chassis.pid_odom_set({{24, 98}, rev, 110});
    chassis.pid_wait();

    chassis.pid_odom_set({{5, 98}, fwd, 110});
    chassis.pid_wait();

    // back up and clear the corner
    chassis.pid_odom_set({{10, 75}, rev, 110});
    chassis.pid_wait();

    leftDoinker.set(true);

    // clear corner
    chassis.pid_odom_set({{8, 100}, fwd, 70});
    chassis.pid_wait();

    leftDoinker.set(false);

    // place mogo in corner
    chassis.pid_odom_set({{2, 102}, rev, 110});
    chassis.pid_wait();

    clampCylinder.set(false);
}