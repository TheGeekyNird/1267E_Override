#include "main.h"
#include <cmath>

PID::PID(double kp_fb, double ki_fb, double kd_fb, double kp_tu, double ki_tu, double kd_tu, double dt) {
    this->kp_fb = kp_fb;
    this->ki_fb = ki_fb;
    this->kd_fb = kd_fb;

    this->kp_tu = kp_tu;
    this->ki_tu = ki_tu;
    this->kd_tu = kd_tu;

    this->dt = dt;
    this->ratio = 1040.0 / 180.0;
    this->minimum = 22;
    this->fb_max = 72;
    this->lw_max = 30;
    this->tu_max = 60;
    this->settle = 3;
    this->e_break = 12.0;
    this->a_break = 2.0;
}

void PID::tare_prepare() {
    delay(50);

    FL.set_encoder_units(MOTOR_ENCODER_DEGREES);
    ML.set_encoder_units(MOTOR_ENCODER_DEGREES);
    BL.set_encoder_units(MOTOR_ENCODER_DEGREES);
    FR.set_encoder_units(MOTOR_ENCODER_DEGREES);
    MR.set_encoder_units(MOTOR_ENCODER_DEGREES);
    BR.set_encoder_units(MOTOR_ENCODER_DEGREES);

    FL.tare_position();
    ML.tare_position();
    BL.tare_position();
    FR.tare_position();
    MR.tare_position();
    BR.tare_position();

    imu.tare_rotation();
}

void PID::set_finished() {
    FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    ML.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    MR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    move_drive_motors(0, 0);
    delay(150);
}

void PID::go(double distangle, bool turn, int timeout) {
    int loops = 0;
    double target = turn ? distangle * this->ratio
                         : (360.0 * distangle * 60.0) / (3.25 * 3.141592653589793 * 36.0);

    double left_error = 0.0;
    double right_error = 0.0;
    double left_prev = 0.0;
    double right_prev = 0.0;
    double left_integral = 0.0;
    double right_integral = 0.0;
    double left_derivative = 0.0;
    double right_derivative = 0.0;
    double left_motor = 0.0;
    double right_motor = 0.0;
    double left_encode = 0.0;
    double right_encode = 0.0;

    bool finish = false;
    this->tare_prepare();

    uint32_t start = millis();
    uint32_t time = millis();

    while (!finish) {
        left_encode = (FL.get_position() + ML.get_position() + BL.get_position()) / 3.0;
        right_encode = (FR.get_position() + MR.get_position() + BR.get_position()) / 3.0;

        if (!turn) {
            left_error = target - left_encode;
            right_error = target - right_encode;

            left_integral = this->ki_fb * (left_integral + this->dt * left_error);
            right_integral = this->ki_fb * (right_integral + this->dt * right_error);

            left_derivative = this->kd_fb * (left_error - left_prev) / this->dt;
            right_derivative = this->kd_fb * (right_error - right_prev) / this->dt;

            left_prev = left_error;
            right_prev = right_error;

            left_motor = (this->kp_fb * left_error) + left_integral + left_derivative;
            right_motor = (this->kp_fb * right_error) + right_integral + right_derivative;
        } else {
            const double turn_target = (distangle * this->ratio + imu.get_rotation()) * 0.5;
            double turn_error = turn_target - imu.get_rotation();
            left_error = turn_error;
            right_error = -turn_error;

            left_integral = this->ki_tu * (left_integral + this->dt * left_error);
            right_integral = this->ki_tu * (right_integral + this->dt * right_error);

            left_derivative = this->kd_tu * (left_error - left_prev) / this->dt;
            right_derivative = this->kd_tu * (right_error - right_prev) / this->dt;

            left_prev = left_error;
            right_prev = right_error;

            left_motor = (this->kp_tu * left_error) + left_integral + left_derivative;
            right_motor = (this->kp_tu * right_error) + right_integral + right_derivative;
        }

        if (std::fabs(left_motor) > this->fb_max) {
            left_motor = this->fb_max * get_sign(left_motor);
        } else if (std::fabs(left_motor) < this->minimum) {
            left_motor = this->minimum * get_sign(left_motor);
        }

        if (std::fabs(right_motor) > this->fb_max) {
            right_motor = this->fb_max * get_sign(right_motor);
        } else if (std::fabs(right_motor) < this->minimum) {
            right_motor = this->minimum * get_sign(right_motor);
        }

        move_drive_motors(left_motor, right_motor);

        const double average_error = std::fabs((left_error + right_error) / 2.0);
        if (average_error <= this->e_break) {
            if (loops < this->settle) {
                loops++;
            } else {
                finish = true;
            }
        } else {
            loops = 0;
        }

        if (timeout != -1 && (millis() - start) >= static_cast<uint32_t>(timeout)) {
            finish = true;
        }

        Task::delay_until(&time, this->dt);
    }

    this->set_finished();
}