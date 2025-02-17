#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline ez::Piston clampCylinder('H');
inline ez::Piston leftDoinker('G');

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');