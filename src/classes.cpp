#include "main.h"
#include <cmath>

//Drivetrain PID
DrivePID::DrivePID(double kp_fb,double ki_fb,double kd_fb, double kp_tu,double ki_tu,double kd_tu, double dt){
    this->kp_fb = kp_fb;
    this->ki_fb = ki_fb;
    this->kd_fb = kd_fb;

    this->kp_tu = kp_tu;
    this->ki_tu = ki_tu;
    this->kd_tu = kd_tu;

    this->finish=false;
    this->dt=dt;

    this->li=0;
    this->ri=0;
    this->loe=0;
    this->roe=0;

    //R
    this->s=500;
    //R
}

void DrivePID::prepare(double distangle, bool turn, bool rev){
    imu.tare_rotation();
    this->turn=turn;
    this->finish = false;
    this->breakpoint=!this->turn?30:1.5;
    this->minimum=!this->turn?35:35;
    if (this->turn == false){
        this->target = (360*distangle*60)/(3.25*3.1416*36);
        //360 - Convert into revolutions
        //distance (inches)
        //wheel diameter (inches)
        //pi
        //gear ratio (motor/wheel)
    }
    else {
        this->target=(distangle+imu.get_rotation())*(1-2*hilo);
    }
    //Set correct k constants
    this->kp=this->turn==0?this->kp_fb:this->kp_tu;
    this->ki=this->turn==0?this->ki_fb:this->ki_tu;
    this->kd=this->turn==0?this->kd_fb:this->kd_tu;

	//Set correct encoder units (degrees)
	FL.set_encoder_units(MOTOR_ENCODER_DEGREES);
    ML.set_encoder_units(MOTOR_ENCODER_DEGREES);
	BL.set_encoder_units(MOTOR_ENCODER_DEGREES);
	FR.set_encoder_units(MOTOR_ENCODER_DEGREES);
    MR.set_encoder_units(MOTOR_ENCODER_DEGREES);
	BR.set_encoder_units(MOTOR_ENCODER_DEGREES);

	//Set motor position to 0
	FL.tare_position();
    ML.tare_position();
	BL.tare_position();
	FR.tare_position();
    MR.tare_position();
	BR.tare_position();
    delay(20);
    //pros::screen::print(TEXT_LARGE_CENTER, 1, "%f", imu.get_rotation());
    //pros::screen::print(TEXT_LARGE_CENTER, 3, "%f", target);

    //Set time
	this->time = millis();
    this->t = 0;

    //R
    this->settle=0;
    //R
}

void DrivePID::hmove(double distance){
    this->l_encode = (FL.get_position()+ML.get_position()+BL.get_position())/3;
    this->r_encode = (FR.get_position()+MR.get_position()+BR.get_position())/3;
    //This averages all three motors, which is more accurate
    //Error for both sides
    this->l_error = this->target-this->l_encode;
    this->r_error = this->target-this->r_encode;
    //Proportional for both sides
    this->lp = this->kp*this->l_error;
    this->rp = this->kp*this->r_error;
    //Integral for both sides
	float arb_num = 100;
	this->li = this->l_error>std::abs(arb_num)?0:this->l_error==0?0:this->ki*(this->li+this->dt*this->l_error);
	this->ri = this->r_error>std::abs(arb_num)?0:this->r_error==0?0:this->ki*(this->ri+this->dt*this->r_error);
	//Derivative for both sides
	this->ld = this->kd*(this->l_error-this->loe)/this->dt;
	this->rd = this->kd*(this->r_error-this->roe)/this->dt;
	//Set previous error to error
	this->loe = this->l_error;
	this->roe = this->r_error;
    //Motor stuff
    this->l_add = this->lp+this->li+this->ld;
    this->r_add = this->rp+this->ri+this->rd;
    if (fabs(this->l_add)<=this->minimum and fabs(this->r_add)<=this->minimum and 1==0){
        this->l_motor=30*get_sign(this->target);
        this->r_motor=this->l_motor;
    }
    else{
        this->l_motor=this->l_add;
        this->r_motor=this->r_add;
    }
    int max=64;
    l_motor=fabs(l_motor)<max?l_motor:get_sign(l_motor)*max;
    r_motor=fabs(r_motor)<max?r_motor:get_sign(r_motor)*max;
    move_drive_motors(this->l_motor,this->r_motor);
    //Check if loop is done
    //R
    if (within(this->l_error,0,this->breakpoint) and within(this->r_error,0,this->breakpoint)){this->finish=true;}
    /*
    if (within(this->l_error,0,this->breakpoint) and within(this->r_error,0,this->breakpoint)){
        if (this->settle<this->s){this->settle+=1;}
        else if (settle>=this->s){this->finish=true;}
        else {settle=0;}
    }
    */
    //R
    if (this->finish){
        FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        ML.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        MR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    
        FL.brake();
        BL.brake();
        FR.brake();
        BR.brake();
        ML.brake();
        MR.brake();
    }
    //Delay for 10ms
    Task::delay_until(&this->time,this->dt);
    }


void DrivePID::hturn(double angle){  
    //We only need one variable but I only have
    //l & r variables. I'm using l in name only 
    //Calculatons
    this->l_error=10*(this->target-imu.get_rotation()); 
    this->lp = this->kp*this->l_error;
	float arb_num = 100;
	this->li = this->l_error>std::abs(arb_num)?0:this->l_error==0?0:this->ki*(this->li+this->dt*this->l_error);
	this->ld = this->kd*(this->l_error-this->loe)/this->dt;
	this->loe = this->l_error;
    //Motor Stuff
    this->l_add = this->lp+this->li+this->ld;
    if (fabs(this->l_add)<=this->minimum and fabs(this->r_add)<=this->minimum and 1==0){
        this->l_motor=30*get_sign(this->target);
        this->r_motor=-this->l_motor;
    }
    else{
        this->l_motor=this->l_add;
        this->r_motor=-this->l_add;
    }
    int max=40;
    l_motor=fabs(l_motor)<max?l_motor:get_sign(l_motor)*max;
    r_motor=fabs(r_motor)<max?r_motor:get_sign(r_motor)*max;
    move_drive_motors(this->l_motor,this->r_motor);
    //Check if loop is done    
    //R
    //if (within(this->l_error,0,this->breakpoint)){this->finish=true;}
    
    if (within(this->l_error,0,this->breakpoint)){
        //pros::screen::print(TEXT_LARGE,5,"1");
        if (this->settle<this->s+1){this->settle+=1;}
        else if (settle>=this->s){this->finish=true;}
        else {settle=0;/*pros::screen::print(TEXT_LARGE,5,"0");*/}
    }
    //pros::screen::print(TEXT_LARGE,0,"%i,%i,%f",this->settle,this->l_error);
    

    //R
    if (this->finish){
        FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        ML.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        MR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    
        FL.brake();
        BL.brake();
        FR.brake();
        BR.brake();
        ML.brake();
        MR.brake();


    //Delay for 10ms
    Task::delay_until(&this->time,this->dt);

    //Print stuff
    }
}

void DrivePID::go(){
    if (this->turn){this->hturn(this->target);}
    else{this->hmove(this->target);}
}

bool DrivePID::is_finished(){
    return this->finish;
}

void DrivePID::timeout(int wait){
    if (wait!=0){
        this->t+=this->dt;
        if (this->t>=wait){this->finish=true;}
    }
}

void DrivePID::q_go(double distangle, bool  , int t){
    this->prepare(distangle,turn);
    while (!this->is_finished()){
        this->go();
        this->timeout(t);
    }
}

/*
How to use PIDmove:
    prepare(distance, turn {0/1});
    while(!is_finished()){
        other_stuff_first();
        move();
    }
*/

//Buttons On Screen
ScreenButton::ScreenButton(int x, int y, int length, int height, int color_one, int color_two, std::string str_one, std::string str_two, int border_one, int border_two, bool toggled){
    this->x=x;
    this->y=y;
    this->length=length;
    this->height=height;
    this->color_one=color_one;
    this->color_two=color_two;
    this->toggle=toggled;
    this->enable=false;
    this->str_one=str_one;
    this->str_two=str_two;
    this->border_one=border_one;
    this->border_two=border_two;
}

void ScreenButton::draw_button(){
    int margin = 3;
    std::string str_use;
    int color;
    double scale;
    if (enable){ //Enabled Button
        //Draw toggled outline if needed
        //Also set other things
        if (toggle){
            screen::set_pen(this->border_two);
            str_use = this->str_two;
            color = this->color_two;
            //Border, string, text
        }
        else {
            screen::set_pen(this->border_one);
            str_use = this->str_one;
            color = this->color_one;
            //Border, string, text
        }
        screen::fill_rect(this->x-margin,this->y-margin,
        this->x+this->length+margin,this->y+this->height+margin);
        //Draw actual button
        screen::set_pen(color);
        screen::fill_rect(this->x,this->y,this->x
        +this->length,this->y+this->height);
        //Draw text
        screen::set_pen(0xfffffff);
        scale = 1.0/int(str_use.size());
        screen::print(TEXT_LARGE,int(this->x+scale*this->length)
        ,int(this->y+scale*this->height),"%s",str_use);        
    }
    else {
        //Disabled Button
        screen::set_pen(0x000000);
        screen::fill_rect(this->x-margin,this->y-margin,
        this->x+this->length+margin,this->y+this->height+margin);
    }
}

bool ScreenButton::toggled(){
    return this->toggle;
}

void ScreenButton::change(bool value){
    if (this->toggle!=value){
        this->toggle=value;
        if (this->enable){this->draw_button();}
    }
}

void ScreenButton::poll(int touch_x, int touch_y){
    //The reason for using parameters rather than
    //accessing the touch data directly is that
    //accessing the data only once is faster 

    if (this->enable){
        if (touch_x >= this->x and touch_x <= this->x+this->length
         and touch_y >= this->y and touch_y <= this->y+this->height){
            this->toggle = !this->toggle;
            this->draw_button();
        }
    }
}

void ScreenButton::enabled(bool enable){
    //Enable whenever you want button to 
    //display and allow touch, and
    //disable otherwise
    this->enable=enable;
    this->draw_button();
}