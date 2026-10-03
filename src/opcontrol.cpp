#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/screen.hpp"
#include <cstdio>

void opcontrol() {
	// Define opcontrol variables and pre-opcontrol code here
	uint32_t sleep_time = millis();
	int count=0;
	int ic=0;

	double current_color;
	//Brake Types
	Intake.set_brake_mode(MOTOR_BRAKE_COAST);
	Middle.set_brake_mode(MOTOR_BRAKE_COAST);
	Top.set_brake_mode(MOTOR_BRAKE_HOLD);

	//Set Default States
	//lift_piston.extend();
	loader.retract();
	triangle.extend();
	colory.set_led_pwm(100);
	
	// Active opcontrol code
	while (true){
	
		// Drive code
		int deadzone=10;
		int leftX = 0.5*joystick_math(Con1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X),deadzone);
		int leftY = joystick_math(Con1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),deadzone);
		int rightX = joystick_math(Con1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X),deadzone);
		int rightY = joystick_math(Con1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y),deadzone);

		int leftVal = rightY+leftY+leftX;
		int rightVal = +rightY+leftY-leftX;

		// currently set split arcade
		FL.move(leftVal);
		ML.move(leftVal);
		BL.move(leftVal);

		FR.move(rightVal);
		MR.move(rightVal);
		BR.move(rightVal);

		//Intake
			if (Con1.get_digital(E_CONTROLLER_DIGITAL_R1)){
				Intake.move(127);
				Middle.move(127);
			}
			else if (Con1.get_digital(E_CONTROLLER_DIGITAL_R2)){
				Intake.move(-127);
				Middle.move(-127);
			}
			else {
				Intake.brake();
				Middle.brake();
			}
			if (Con1.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
				Top.move(127);
			}
			else if (Con1.get_digital(E_CONTROLLER_DIGITAL_L2)){
				Top.move(-127);
			}
			else {
				Top.brake();
			}


		
			

		//Antler extension toggler
		if (Con1.get_digital_new_press(DIGITAL_A)){
			antler_l.toggle();
			antler_r.toggle();
		}

		//Triangle or Loader
		if (Con1.get_digital_new_press(DIGITAL_DOWN)){
			if (!loader.is_extended()&&triangle.is_extended()){
				triangle.retract();
				loader.extend();
			}
			else if (loader.is_extended()&&!triangle.is_extended()){
				loader.retract();
			}
		}
		
		if (load_button.get_value() && !loader.is_extended()){triangle.extend();}
		else if (loader.is_extended()){triangle.retract();}

		/*
		if (Con1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
			triangle.toggle();
			loader.toggle();
		}
		*/
		
		if (within(colory.get_hue(),0,10)){
			//Red
			screen::set_eraser(0xd22630);
			screen::erase();
			Con1.print(0,0,"Red.");
		}
		else if (within(colory.get_hue(),225,50)){
			//Blue
			screen::set_eraser(0x0077c8);
			screen::erase();
			Con1.print(0,0,"Blue");
		}
		else {
			//Else
			screen::set_eraser(0x004e38);
			screen::erase();
			Con1.print(0,0,"None");
		}


		//10 msec loop
		pros::Task::delay_until(&sleep_time, 10);
		count++;
	}
}