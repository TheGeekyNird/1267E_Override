#pragma once

#include "api.h"

using namespace pros;

// Controller
extern pros::Controller Con1;

// Drive motors
extern Motor FL;
extern Motor ML;
extern Motor BL;
extern Motor FR;
extern Motor MR;
extern Motor BR;

// Generic sensors / state
extern pros::Imu imu;
extern bool color;
extern bool hilo;