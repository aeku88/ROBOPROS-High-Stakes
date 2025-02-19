#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/rotation.hpp"

inline bool runLB = false;

inline pros::MotorGroup lbMotors({18, -11});

inline pros::Motor lb_motor(18);

// inline pros::Rotation lbRotation(20);

inline ez::PID lbPID(.5, 0.0, 0.5, 0.0, "Lady Brown");

inline double heights[4] = {0.0, 156, 1600.0, 2000.0};

// used to be 176

inline int positionIndex = 0;

void lbSetPosition(int index);

void lbSet(double position);

void lbMoveUp();

void lbMoveDown();

void lbComputePID();

void lbWait();