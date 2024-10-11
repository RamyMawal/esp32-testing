#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "motor.h"

#define GPIO_MOTOR_FORWARD_PIN 5
#define GPIO_MOTOR_BACKWARD_PIN 6

void app_main() {
    // Setup GPIO for H-Bridge control
    setup_motor_gpio();

    // Setup MCPWM for motor speed control
    setup_mcpwm();

    while (1) {
        control_motor_forward();
        vTaskDelay(pdMS_TO_TICKS(1000));

        control_motor_stop();
        vTaskDelay(pdMS_TO_TICKS(500));

        control_motor_backward();
        vTaskDelay(pdMS_TO_TICKS(1000));

        control_motor_stop();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
