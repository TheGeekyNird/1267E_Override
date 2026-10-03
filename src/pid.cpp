#include "main.h"
#include "pros/rtos.h"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

PID::PID(double kp_fb,double ki_fb,double kd_fb, double kp_tu,double ki_tu,double kd_tu, double dt){
    this->kp_fb = kp_fb;
    this->ki_fb = ki_fb;
    this->kd_fb = kd_fb;

    this->kp_tu = kp_tu;
    this->ki_tu = ki_tu;
    this->kd_tu = kd_tu;

    this->dt=dt;
    this->ratio=1040/180.0;
    this->minimum=22;
    this->fb_max=72;
    this->lw_max=30;
    this->tu_max=60;
    this->settle=3;
    this->e_break=12;
    this->a_break=2;

}

void PID::tare_prepare(){
    delay(50);
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

    //Set IMU to 0
    imu.tare_rotation();
}

void PID::set_finished(){
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

        delay(150);
}

void PID::go(double distangle, bool turn, int timeout){
    int loops=0;
    double target;
    //Do Math and Prepare
    if (!turn){
        //Forwards/Backwards
        target = (360*distangle*60)/(3.25*3.1416*36);
    }
    else {
        //Turning
        target = distangle*this->ratio;
    }
    double l_error,r_error,loe,roe,lp,rp,li,ri,ld,rd,l_motor,r_motor,e_motor,l_encode,r_encode=0;
    bool finish=false;
    this->tare_prepare();
    FL.tare_position();
    uint32_t time,start=millis();

    //Actual Loop
    while (!finish){
        if (!turn){
            //Forwards and Backwards
            l_encode = (FL.get_position()+ML.get_position()+BL.get_position())/3.0;
            r_encode = (FR.get_position()+MR.get_position()+BR.get_position())/3.0;
            //This averages all three motors, which is more accurate
            //Error for both sides
            l_error = target-l_encode;
            r_error = target-r_encode;
            //Proportional for both sides
            lp = this->kp_fb*l_error;
            rp = this->kp_fb*r_error;
            //Integral for both sides
            float arb_num = 100;
            li = l_error>std::abs(100)?0:l_error==0?0:this->ki_fb*(li+this->dt*l_error);
            ri = r_error>std::abs(100)?0:r_error==0?0:this->ki_fb*(ri+this->dt*r_error);
            //Derivative for both sides
            ld = this->kd_fb*(l_error-loe)/dt;
            rd = this->kd_fb*(r_error-roe)/dt;
            //Set previous error to error
            loe = l_error;
            roe = r_error;
            //Value before max and min
            l_motor = lp+li+ld;
            r_motor = rp+ri+rd;
            //Bound the values
            if (std::fabs(l_motor)>this->fb_max){l_motor=this->fb_max*get_sign(l_motor);}
            else if (std::fabs(l_motor)<this->minimum){l_motor=this->minimum*get_sign(l_motor);}

            if (std::fabs(r_motor)>this->fb_max){r_motor=this->fb_max*get_sign(r_motor);}
            else if (std::fabs(r_motor)<this->minimum){r_motor=this->minimum*get_sign(r_motor);}

            if (std::fabs(l_error)<std::fabs(target*0.3) && l_motor>lw_max){l_motor=lw_max*get_sign(l_motor);}
            if (std::fabs(r_error)<std::fabs(target*0.3) && r_motor>lw_max){r_motor=lw_max*get_sign(r_motor);}

            
            //screen::print(pros::E_TEXT_LARGE,2,"%f",FL.get_position());

        }
        else {
            //Turning

            //Encoder Math


            l_encode = (FL.get_position()+ML.get_position()+BL.get_position())/3.0;
            r_encode = -(FR.get_position()+MR.get_position()+BR.get_position())/3.0;
            l_error = target-l_encode;
            r_error = target-r_encode;
            lp = this->kp_fb*l_error;
            rp = this->kp_fb*r_error;
            li = l_error>std::abs(100)?0:l_error==0?0:this->ki_fb*(li+this->dt*l_error);
            ri = r_error>std::abs(100)?0:r_error==0?0:this->ki_fb*(ri+this->dt*r_error);
            ld = this->kd_fb*(l_error-loe)/dt;
            rd = this->kd_fb*(r_error-roe)/dt;
            loe = l_error;
            roe = r_error;
            e_motor = (lp+li+ld+rp+ri+rd)/2.0;
            if (std::fabs(e_motor)>this->tu_max){e_motor=this->tu_max*get_sign(e_motor);}
            else if (std::fabs(e_motor)<this->minimum){e_motor=this->minimum*get_sign(e_motor);}

            l_motor=e_motor;
            r_motor=-l_motor;

        }

        move_drive_motors(l_motor, r_motor);

        //Breakpoints
        if (std::fabs((l_error+r_error)/2)<=this->e_break){
            if (loops<this->settle){loops++;}
            else{finish=1;}
        }
        else{loops=0;}
        if (millis()>=start+timeout && timeout!=-1){finish=1;}
        Task::delay_until(&time,this->dt);
    }
    this->set_finished();

}