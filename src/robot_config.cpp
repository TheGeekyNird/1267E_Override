#include "main.h"

using namespace pros;

// Controller
pros::Controller Con1(pros::E_CONTROLLER_MASTER);

// Drive motor definitions.
// Ports are placeholders only; assign the real wiring when the new robot is ready.
Motor FL(1, v5::MotorGears::blue);
Motor ML(2, v5::MotorGears::blue);
Motor BL(3, v5::MotorGears::blue);

Motor FR(4, v5::MotorGears::blue);
Motor MR(5, v5::MotorGears::blue);
Motor BR(6, v5::MotorGears::blue);

// Generic hardware state
pros::Imu imu(7);
bool color = false;
bool hilo = false;