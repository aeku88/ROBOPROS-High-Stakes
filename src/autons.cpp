#include "autons.hpp"
#include "EZ-Template/util.hpp"
#include "intake.hpp"
#include "lbArm.hpp"
#include "main.h"
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
void default_constants() 
{
    // P, I, D, and Start I
    chassis.pid_drive_constants_set(12.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
    chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
    chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
    chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
    chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
    chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

    // Exit conditions
    chassis.pid_turn_exit_condition_set(50_ms, 4_deg, 250_ms, 7_deg, 500_ms, 500_ms);
    chassis.pid_swing_exit_condition_set(50_ms, 4_deg, 250_ms, 7_deg, 500_ms, 500_ms);
    chassis.pid_drive_exit_condition_set(50_ms, 2.5_in, 250_ms, 5_in, 500_ms, 500_ms);
    chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
    chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
    chassis.pid_turn_chain_constant_set(6_deg);
    chassis.pid_swing_chain_constant_set(5_deg);
    chassis.pid_drive_chain_constant_set(4_in);

    // Slew constants
    chassis.slew_turn_constants_set(3_deg, 70);
    chassis.slew_drive_constants_set(3_in, 70);
    chassis.slew_swing_constants_set(3_in, 80);

    // The amount that turns are prioritized over driving in odom motions
    // - if you have tracking wheels, you can run this higher.  1.0 is the max
    chassis.odom_turn_bias_set(0.9);

    chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
    chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
    chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

    chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
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