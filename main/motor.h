#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "driver/mcpwm.h"

void setup_motor_gpio();

void control_motor_forward();

void control_motor_backward();

void control_motor_stop();

void setup_mcpwm();