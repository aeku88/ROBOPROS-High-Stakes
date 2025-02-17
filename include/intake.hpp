#pragma once

#include <sys/types.h>
#include <cstdint>
#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/colors.hpp"

inline pros::Motor intake(-3); // Intake motor

inline pros::Optical optical(17); // Optical sensor used to "color sort" rings
inline pros::Color allianceColor = pros::Color::red; /* Color of alliance */
inline uint8_t redHue = 5, /* Hue value for red */
               blueHue = 230,/* Hue value for blue */
               detectionProximity = 210, /* Maximum distance used to start sensing rings — larger values indicate smaller distance*/
               hueThreshold = 10; /* Threshold for hue values */
void intakeControl();

void sortRing(); // Sorts a ring by reversing intake