#include "pros/optical.hpp"
using namespace pros;
//Using extern in C++ isn't great practice, but it is easier and what VEXCode uses.

// Controller
extern pros::Controller Con1;

// Drive Motors
extern Motor FL;
extern Motor ML;
extern Motor BL;
extern Motor FR;
extern Motor MR;
extern Motor BR;

// Intake + Scoring Motors
extern Motor Intake;
extern Motor Middle;
extern Motor Top;

// Pneumatics 
extern pros::adi::Pneumatics antler_l;
extern pros::adi::Pneumatics antler_r;
extern pros::adi::Pneumatics loader;
extern pros::adi::Pneumatics triangle;

// Sensors
extern pros::Imu imu;
extern adi::DigitalIn load_button;
extern pros::Optical colory;

//Variables
extern bool color;
extern bool hilo;