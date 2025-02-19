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
    chassis.pid_drive_constants_set(9.5, 0, 7);         // Fwd/rev constants, used for odom and non odom motions
    chassis.pid_heading_constants_set(9, 0.0, 7);        // Holds the robot straight while going forward without odom
    chassis.pid_turn_constants_set(0, 0, 0);     // Turn in place constants
    chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
    chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
    chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

    // Exit conditions
    chassis.pid_turn_exit_condition_set(50_ms, 4_deg, 250_ms, 7_deg, 500_ms, 500_ms);
    chassis.pid_swing_exit_condition_set(50_ms, 4_deg, 250_ms, 7_deg, 500_ms, 500_ms);
    chassis.pid_drive_exit_condition_set(70_ms, .5_in, 400_ms, 3_in, 200_ms, 200_ms);
    chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
    chassis.pid_odom_drive_exit_condition_set(90_ms, .5_in, 250_ms, 3_in, 500_ms, 750_ms);
    chassis.pid_turn_chain_constant_set(6_deg);
    chassis.pid_swing_chain_constant_set(5_deg);
    chassis.pid_drive_chain_constant_set(6_in);

    // Slew constants
    chassis.slew_turn_constants_set(3_deg, 70);
    chassis.slew_drive_constants_set(4_in, 40);
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
    chassis.pid_heading_constants_set(0, 0, 0);

    lbSetPosition(2);//score
    pros::delay(800);
    chassis.pid_drive_set(-11_in, DRIVE_SPEED/1.125);
    lbSetPosition(0);//reset arm as we drive
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(0_deg, TURN_SPEED);
    chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-20_in, DRIVE_SPEED);//approaching at full speed
    chassis.pid_wait_quick_chain();

    chassis.pid_drive_set(-4_in, DRIVE_SPEED/2);//approaching at half speed

    chassis.pid_wait_quick_chain();
    //chassis.pid_drive_set(-2_in, DRIVE_SPEED/1.5);//slow approach to mogo with 8 inches exccess
    chassis.pid_wait();
    clampCylinder.set(true);
    //pros::delay(50);//tune to see how low this can go without sacrificng consistency
    //chassis.pid_drive_set(_in, DRIVE_SPEED);//reverting 4 inchES excess 
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(135_deg, TURN_SPEED);
    chassis.pid_wait_quick_chain();
    intake.move(127);//preload scored
    chassis.pid_drive_set(14_in, DRIVE_SPEED);//appraoching ring 1
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(85, TURN_SPEED);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(10, DRIVE_SPEED);// approach to ring 2
    chassis.pid_wait_quick_chain();
    pros::delay(120);

    chassis.pid_swing_set(ez::RIGHT_SWING, -60_deg, SWING_SPEED, 5);//turning to ring 3 about the right side
   // intake.move(0);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(14_in, DRIVE_SPEED / 1.4);//getting ring 3
    intake.move(127);
    chassis.pid_wait_quick_chain();


    chassis.pid_drive_set(33_in, DRIVE_SPEED / 2); // drive straight after 
    chassis.pid_wait_quick_chain();

    chassis.pid_turn_set(-90_deg, TURN_SPEED);
    chassis.pid_wait_quick_chain();

    chassis.pid_drive_set(34_in, DRIVE_SPEED / 2);
    chassis.pid_wait_quick_chain();

    chassis.pid_turn_set(135_deg, TURN_SPEED);
    chassis.pid_wait_quick_chain();

    clampCylinder.set(false);
    chassis.pid_wait_quick_chain();

    chassis.pid_turn_set(0_deg, TURN_SPEED);
    chassis.pid_wait_quick_chain(); 

    intake.move(0);
    chassis.pid_wait_quick_chain();


    chassis.pid_drive_set(-10_in, DRIVE_SPEED);
    chassis.pid_wait_quick_chain(); 

    chassis.pid_drive_set(-4_in, DRIVE_SPEED/2);
    chassis.pid_wait_quick_chain(); 

    clampCylinder.set(true);
    chassis.pid_wait_quick_chain();

    chassis.pid_turn_set(-90_deg, TURN_SPEED);
    chassis.pid_wait_quick_chain(); 

    intake.move(127);
    chassis.pid_wait_quick_chain();
    
    chassis.pid_drive_set(18_in, DRIVE_SPEED);
    chassis.pid_wait_quick_chain();

    chassis.pid_turn_set(-75_deg, TURN_SPEED);
    chassis.pid_wait_quick_chain(); 

    chassis.pid_drive_set(-30_in, DRIVE_SPEED / 2);
    chassis.pid_wait_quick_chain();

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

    chassis.pid_odom_set({{118, 70}, rev, 50});
    chassis.pid_wait();
    lbSetPosition(1);
    pros::delay(700);

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
    chassis.pid_odom_set({{135, 70}, fwd, 35});

    chassis.pid_wait();

    intake.move(-30);
    pros::delay(70);
    intake.move(0);
    pros::delay(300);

    chassis.pid_odom_set({{136, 70}, fwd, 35});
    lbSetPosition(2);

    chassis.pid_wait();

    pros::delay(700);

    // back up and arm down
    lbSetPosition(0);
    chassis.pid_odom_set({{123, 70}, rev, 110});
    chassis.pid_wait();

    // turn towards the last 4 rings
    chassis.pid_odom_set({{123, 45}, fwd, 110});
    chassis.pid_wait();

    // pick up 2nd ring
    chassis.pid_odom_set({{123, 25}, fwd, 50});
    chassis.pid_wait();

    // pick up 3rd ring
    chassis.pid_odom_set({{123, 14}, fwd, 40});
    chassis.pid_wait();

    // pick up last ring
    chassis.pid_odom_set({{131, 24}, fwd, 50});
    chassis.pid_wait();

    // put in corner
    chassis.pid_odom_set({{136, 11}, rev, 90});
    chassis.pid_wait();

    intake.move(-127);

    pros::delay(160);
    intake.move(0);

    clampCylinder.set(false);
    chassis.pid_wait();
    
    chassis.pid_drive_constants_set(16, 0, 36); // increase kp for long movements

    chassis.pid_odom_set({{122, 95}, fwd, 127});
    intake.move(127);

    lbSetPosition(1);
    chassis.pid_wait();

    skills_constants(); // reset pid constants for short movements

    chassis.pid_odom_set({{122, 103}, fwd, 70});
    chassis.pid_wait();

    // get 2nd mogo
    chassis.pid_odom_set({{97, 123}, rev, 60});
    chassis.pid_wait();

    intake.move(0);

    clampCylinder.set(true);
    chassis.pid_wait();

    leftDoinker.set(true);
    
    chassis.pid_odom_set({{120, 122}, fwd, 110});
    chassis.pid_wait();

    chassis.pid_odom_set({{120, 115}, fwd, 110});
    chassis.pid_wait();

    leftDoinker.set(false);

    chassis.pid_odom_set({{130, 122}, rev, 70});
    chassis.pid_wait();

    clampCylinder.set(false);
    chassis.pid_wait();

    intake.move(-127);

    pros::delay(200);
    intake.move(0);

    // armControlCopy->setMaxVelocity(65);

    lbSet(500);
    pros::delay(300);

    chassis.pid_odom_set({{115, 105.5}, fwd, 110});
    chassis.pid_wait();

    chassis.pid_odom_set({{75.5, 105.5}, rev, 60});
    chassis.pid_wait();

    clampCylinder.set(true);
    pros::delay(200);
    chassis.pid_wait();

    chassis.pid_odom_set({{75.5, 119}, fwd, 60});
    chassis.pid_wait();

    chassis.pid_odom_set({{75.5, 113}, rev, 110});
    chassis.pid_wait();

    // armControlCopy->setMaxVelocity(200);

    lbSetPosition(2); // score
    pros::delay(700);

    intake.move(127);

    lbSetPosition(0); // reset
    pros::delay(200);

    // back up

    chassis.pid_odom_set({{76, 100}, rev, 110});
    chassis.pid_wait();

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
    chassis.pid_odom_set({{40, 15}, fwd, 70});
    chassis.pid_wait();
    // pick up the 3rd
    
    chassis.pid_odom_set({{38, -5}, fwd, 70});
    chassis.pid_wait();

    // pick up thelast ring

    chassis.pid_odom_set({{18, 8}, fwd, 70});
    chassis.pid_wait();

    // move to corner

    chassis.pid_odom_set({{18, -4}, rev, 70});
    chassis.pid_wait();

    intake.move(-127);
    pros::delay(160);
    intake.move(0);

    clampCylinder.set(false);
    chassis.pid_wait();

    intake.move(127);
    lbSetPosition(1); // load
    chassis.pid_wait();

    // move to ring
    chassis.pid_odom_set({{29, 30}, fwd, 110});
    chassis.pid_wait();

    // move to mogo
    chassis.pid_odom_set({{60, 5}, rev, 60});
    chassis.pid_wait();

    clampCylinder.set(true);
    chassis.pid_wait();
    intake.move(0);

    pros::delay(200);

    chassis.pid_odom_set({{30, 52}, fwd, 110});
    pros::delay(200);

    chassis.pid_wait();

    // armControlCopy->setMaxVelocity(60);

    lbSet(500); // hover
    pros::delay(300);
    intake.move(127);

    chassis.pid_odom_set({{15, 52}, fwd, 60});
    chassis.pid_wait();

    // armControlCopy->setMaxVelocity(200);

    // score on wallstake
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

    chassis.pid_odom_set({{26, 52}, rev, 110});
    chassis.pid_wait();

    // get the ring
    chassis.pid_odom_set({{26, 75}, fwd, 110});
    chassis.pid_wait();

    // get the mext ring
    chassis.pid_odom_set({{53, 85}, fwd, 110});
    chassis.pid_wait();

    // get the mext ring
    chassis.pid_odom_set({{26, 95}, fwd, 110});
    chassis.pid_wait();

    // get the ring after that
    chassis.pid_odom_set({{24, 105}, fwd, 60});
    chassis.pid_wait();

    // back up and get the ring after that
    chassis.pid_odom_set({{26, 98}, rev, 110});
    chassis.pid_wait();

    chassis.pid_odom_set({{5, 98}, fwd, 70});
    chassis.pid_wait();

    // back up and clear the corner
    chassis.pid_odom_set({{10, 75}, rev, 70});
    chassis.pid_wait();

    leftDoinker.set(true);

    // clear corner
    chassis.pid_odom_set({{10, 95}, fwd, 70});
    chassis.pid_wait();

    // place mogo in corner
    chassis.pid_odom_set({{4, 92}, rev, 110});
    chassis.pid_wait();
}