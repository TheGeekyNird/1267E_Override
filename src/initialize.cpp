#include "main.h"

int auto_select = 0;

void initialize() {
    Con1.clear();
    Con1.set_text(0, 0, "Robot ready");
    delay(250);
}

void competition_initialize() {
    // Keep this function as a no-op shell for a new robot project.
    // Add autonomous selector code here when the real robot is ready.
    auto_select = 0;
}

void autonomous() {
    // Replace with the new robot's autonomous routine.
    move_drive_motors(0, 0);
    delay(50);
}
