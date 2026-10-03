#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/optical.hpp"
using namespace pros;

// VEXCode Style Robot Configuration

// Controller
pros::Controller Con1(pros::E_CONTROLLER_MASTER);

// Drive Motors
// NOTE: negative port number for reverse

// port numbers are placeholders
Motor FL (-7,v5::MotorGears::blue);
Motor ML (-11,v5::MotorGears::blue);
Motor BL (-14,v5::MotorGears::blue);

Motor FR (13,v5::MotorGears::blue);
Motor MR (12,v5::MotorGears::blue);
Motor BR (16,v5::MotorGears::blue);

// Intake + Scoring Motors

// also placeholder port numbers
Motor Intake (5,v5::MotorGears::green);
Motor Middle (21,v5::MotorGears::green);
Motor Top (-4,v5::MotorGears::green);

// Pneumatics 
pros::adi::Pneumatics antler_l('f', false); 
pros::adi::Pneumatics antler_r('e', false); 
pros::adi::Pneumatics loader('g', false);
pros::adi::Pneumatics triangle('h', false);

// Sensors
pros::Imu imu(3);
pros::adi::DigitalIn load_button ('d');
pros::Optical colory(3);

//Variables
bool color;
bool hilo;