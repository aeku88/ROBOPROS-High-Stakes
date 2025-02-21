#include "intake.hpp"
#include <cstdint>
#include "EZ-Template/util.hpp"
#include "pros/colors.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

void intakeControl()
{
    optical.set_integration_time(10);
    pros::delay(100);
    optical.set_led_pwm(100);
    
    while (true)
    { 
        // Normal intake control
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            intake.move(127); 
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            intake.move(-127);
        else if (!pros::competition::is_autonomous())
            intake.move(0);

        if (optical.get_proximity() > detectionProximity && ((allianceColor == pros::Color::red 
            && optical.get_hue() > blueHue - hueThreshold 
            && optical.get_hue() < blueHue + hueThreshold) || (allianceColor == pros::Color::blue 
            && optical.get_hue() > redHue - hueThreshold 
            && optical.get_hue() < redHue + hueThreshold))) // Ring detected
        {
            sortRing(); // Sort the ring
        }

        pros::delay(ez::util::DELAY_TIME);
    }
}

void sortRing()
{
    pros::delay(60);
    intake.move(-127); // reverse the intake
    pros::delay(75); // Delay for 500 milliseconds
    intake.move(127);
}