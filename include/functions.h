using namespace pros;

int joystick_math(int joystick_value, int deadzone);
int get_sign(double value);
bool within(double number, double target, double range);
void move_drive_motors(float Left_value, float Right_value);
void logo();