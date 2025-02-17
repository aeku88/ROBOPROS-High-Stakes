#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/rotation.hpp"

inline pros::MotorGroup lbMotors({18, -11});

inline pros::Motor lb_motor(18);

// inline pros::Rotation lbRotation(20);

inline ez::PID lbPID(.7, 0.0, 0.7, 0.0, "Lady Brown");

inline double heights[4] = {0.0, 155.0, 1600.0, 2000.0};
inline int positionIndex = 0;

void lbSetPosition(int index);

void lbMoveUp();

void lbMoveDown();

void lbComputePID();

void lbWait();