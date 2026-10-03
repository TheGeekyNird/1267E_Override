#include "main.h"
#include <string>
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    delay(1500);
    colory.set_led_pwm(100);
    Con1.clear();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void button_update(){
    screen_touch_status_s_t status = screen::touch_status();
    int touch_x = status.x;
    int touch_y = status.y;
    //button::poll(touch_x,touch_y);
}

int auto_select = 0;

//Define Buttons
ScreenButton color_type(15,15,220,55,0xD22630,0x0077c8,"Red","Blue",0x000000,0x000000);
ScreenButton auton_side(15,85,220,55,0xCFFF04,0xCFFF04,"NA","NA",0x000000,0x000000);
ScreenButton skills_tst(15,155,220,55,0xD29DF8,0xD29DF8,"Skills","Skills");
ScreenButton high_butn(250,15,220,55,0x004e38,0x004e38,"High","High");
ScreenButton low_butn(250,85,220,55,0x004e38,0x004e38,"Low","Low",0x000000,0xFFFFFF,1);
ScreenButton bonk_butn(250,155,220,55,0x004e38,0x004e38,"Bonk", "Bonk");

// competition_initaialize is the same as pre_auton.
void competition_initialize() {
    //Initalize Buttons
    color_type.enabled(true);
    auton_side.enabled(true);
    skills_tst.enabled(true);
    high_butn.enabled(true);
    low_butn.enabled(true);
    bonk_butn.enabled(true);
    pros::screen_touch_status_s_t status;
    int x;
    int y;
	int c=0;
	std::string str;
    bool wait = false;
    bool hold = true;
    while (1==1){
        //Track Touches & Update Buttons
        status = pros::screen::touch_status();
        //Get touchscreen status
        if (status.touch_status==TOUCH_RELEASED and wait==false){
            //If screen is pressed and not locked
            x = status.x; //Get x value
            y = status.y; //Get y value
            color_type.poll(x,y);
            high_butn.poll(x,y);
            bonk_butn.poll(x,y);
            low_butn.poll(x,y);
            skills_tst.poll(x,y);
            auton_side.poll(x,y);
            wait=true; //Lock input
        }
        else if (status.touch_status!=TOUCH_RELEASED){
            wait=false; //Release lock
        }

        //Set Variables
        //This just sets variables for auton
        if (skills_tst.toggled()){
            auto_select = 0;
			high_butn.change(0);
			bonk_butn.change(0);
			low_butn.change(0);
        }
        else if (high_butn.toggled()){
            auto_select = 1;
			skills_tst.change(0);
			bonk_butn.change(0);
			low_butn.change(0);
        }
        else if (low_butn.toggled()){
            auto_select = 2;
			high_butn.change(0);
			bonk_butn.change(0);
			skills_tst.change(0);
        }
        else if (bonk_butn.toggled()){
            auto_select = 3;
        }

        color=color_type.toggled();
        hilo=high_butn.toggled();

		if (!auto_select){str="Skills";}
		else if (auto_select==1){
			if (auton_side.toggled()){str="3 Low";}
			else {str="3 High";}
		}
		else if (auto_select==2){
			if (auton_side.toggled()){str="1 Low";}
			else {str="1 High";}
		}
		else if (auto_select==3){str="NOTHING";}
		//Controller Printing
		if (!c%50){Con1.print(0,0,"%s",str);}
        
        pros::Task::delay(10); //Delay before repeat
		c++;

    }
}

/*
 void competition_initialize() {
	std::string str;
    bool lock = false;
	while (1==1){
		if (auto_selector.get_value() and !lock){
            lock=true;
			auto_select+=1;
			auto_select%=4;
			switch (auto_select){
				case 0:
					str="High 3";break;
				case 1:
					str="Low 3";break;
				case 2:
					str="High 1";break;
				case 3:
					str="Low 1";break;
			}
			screen::print(pros::E_TEXT_MEDIUM,2,"%s",str);
		}
        else if (!auto_selector.get_value()){lock=false;}
		pros::Task::delay(10); //Delay before repeat
		color = auto_select%2;	
        hilo=color;
	}
}
*/

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	logo();
	switch (auto_select){
		case 0:
			 skills_auton(); break; //Skills
		case 1:
			high_auton(); break;
		case 2:
			low_auton(); break;
		case 3:
			Top.move(127); delay(250); Top.move(0); break; //Skills
            //move_drive_motors(64, 64);delay(150);move_drive_motors(0, 0);break; //Match

	}
}
