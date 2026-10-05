#include "main.h"
#include <cmath>

int joystick_math(int joystick_value, int deadzone) {
    if (std::abs(joystick_value) < deadzone) {
        return 0;
    }

    const double scaled = std::pow(std::fabs(static_cast<double>(joystick_value)) / 50.0, 5.0) + 20.0;
    return get_sign(joystick_value) * static_cast<int>(scaled);
}

int get_sign(double value) {
    if (value == 0.0) {
        return 1;
    }
    return value < 0.0 ? -1 : 1;
}

bool within(double number, double target, double range) {
    return number >= target - range && number <= target + range;
}

void move_drive_motors(float left_value, float right_value) {
    FL.move(left_value);
    ML.move(left_value);
    BL.move(left_value);

    FR.move(right_value);
    MR.move(right_value);
    BR.move(right_value);
}