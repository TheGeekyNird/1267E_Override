#include "main.h"
#include "pros/motors.h"

void skills_auton(){
        PID drive(0.4,0,0.1,1.1,0,0.7);
        //Go to match loader
        Top.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        drive.go(-32, 0);
        drive.go(-90,1,1500);
        //Red High
        loader.extend();
        delay(400);
        Intake.move(127);
        Middle.move(96);
        Top.brake();
        drive.go(-8,0,1000);
        delay(500);
        drive.go(-0.5,0,250);
        delay(750);
        Intake.move(32);
        Middle.move(-64);
        drive.go(12,0);
        Middle.move(0);
        loader.retract();
        delay(250);
        triangle.extend();
        drive.go(20,0,1200);
        Intake.move(127);
        Middle.move(-127);
        Top.move(127);
        delay(250);
        Middle.move(127);
        delay(2000);
        Top.move(0);
        triangle.retract();
        //Blue Low
        Intake.move(-127);
        Middle.move(-127);
        Top.move(-127);
        drive.go(-8,0);
        drive.go(-90,1);
        drive.go(16,0);
        drive.go(85,1);
        drive.go(80,0);
        drive.go(-90,1);
        drive.go(-12,0);
        drive.go(-90,1);
        loader.extend();
        delay(250);
        Intake.move(127);
        Middle.move(64);
        drive.go(-12,0,1500);
        drive.go(1,0);
        drive.go(-1,0);
        delay(3000);
        drive.go(10,0);
        loader.retract();
        delay(250);
        triangle.extend();
        drive.go(20,0,2000);
        Intake.move(127);
        Middle.move(127);
        Top.move(127);
}