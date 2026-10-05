#include "main.h"

void opcontrol() {
    uint32_t sleep_time = millis();
    const int deadzone = 10;

    FL.set_brake_mode(E_MOTOR_BRAKE_COAST);
    ML.set_brake_mode(E_MOTOR_BRAKE_COAST);
    BL.set_brake_mode(E_MOTOR_BRAKE_COAST);
    FR.set_brake_mode(E_MOTOR_BRAKE_COAST);
    MR.set_brake_mode(E_MOTOR_BRAKE_COAST);
    BR.set_brake_mode(E_MOTOR_BRAKE_COAST);

    while (true) {
        const int left_x = joystick_math(Con1.get_analog(E_CONTROLLER_ANALOG_LEFT_X), deadzone);
        const int left_y = joystick_math(Con1.get_analog(E_CONTROLLER_ANALOG_LEFT_Y), deadzone);
        const int right_y = joystick_math(Con1.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y), deadzone);

        const int left_value = right_y + left_y + left_x;
        const int right_value = right_y + left_y - left_x;

        FL.move(left_value);
        ML.move(left_value);
        BL.move(left_value);
        FR.move(right_value);
        MR.move(right_value);
        BR.move(right_value);

        pros::Task::delay_until(&sleep_time, 10);
    }
}