#include "lbArm.hpp"

void lbSetPosition(int index)
{
    lbPID.target_set(heights[index]);
    positionIndex = index;
}

void lbSet(double position)
{
    lbPID.target_set(position);
}

void lbMoveUp()
{
    if (positionIndex < 3)
        lbSetPosition(positionIndex + 1);
}

void lbMoveDown()
{
    if (positionIndex > 0)
        lbSetPosition(positionIndex - 1);
}

void lbComputePID()
{   
    while (true)
    {
        lbMotors.move(lbPID.compute(lbMotors.get_position()));

        pros::delay(ez::util::DELAY_TIME);
    }
}

void lbWait() 
{
    while (lbPID.exit_condition(lb_motor, true) == ez::RUNNING) 
        pros::delay(ez::util::DELAY_TIME);
}