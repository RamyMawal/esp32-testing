#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"


// Define GPIO pins for H-Bridge control
#define GPIO_MOTOR_FORWARD_PIN 5
#define GPIO_MOTOR_BACKWARD_PIN 6
#define PWM_GPIO_PIN 18 // PWM pin for controlling motor speed

// PWM Configuration
#define PWM_FREQUENCY 1000 // 1kHz PWM frequency
#define PWM_DUTY_CYCLE 75  // 75% duty cycle for speed control

void setup_mcpwm() {
    // Initialize MCPWM on unit 0, timer 0
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_GPIO_PIN);
    
    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQUENCY;  // Set PWM frequency
    pwm_config.cmpr_a = PWM_DUTY_CYCLE;    // Set duty cycle for PWM0A (0-100%)
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);  // Initialize MCPWM
}

void control_motor_forward() {
    gpio_set_level(GPIO_MOTOR_FORWARD_PIN, 1); // Forward ON
    gpio_set_level(GPIO_MOTOR_BACKWARD_PIN, 0); // Backward OFF
}

void control_motor_backward() {
    gpio_set_level(GPIO_MOTOR_FORWARD_PIN, 0); // Forward OFF
    gpio_set_level(GPIO_MOTOR_BACKWARD_PIN, 1); // Backward ON
}

void control_motor_stop() {
    gpio_set_level(GPIO_MOTOR_FORWARD_PIN, 0); // Stop forward
    gpio_set_level(GPIO_MOTOR_BACKWARD_PIN, 0); // Stop backward
}

void app_main() {
    // Setup GPIO for H-Bridge control
    esp_rom_gpio_pad_select_gpio(GPIO_MOTOR_FORWARD_PIN);
    esp_rom_gpio_pad_select_gpio(GPIO_MOTOR_BACKWARD_PIN);
    gpio_set_direction(GPIO_MOTOR_FORWARD_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_BACKWARD_PIN, GPIO_MODE_OUTPUT);

    // Setup MCPWM for motor speed control
    setup_mcpwm();

    while (1) {
        // Run motor forward for 5 seconds
        control_motor_forward();
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Stop motor for 1 second
        control_motor_stop();
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Run motor backward for 5 seconds
        control_motor_backward();
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Stop motor for 1 second
        control_motor_stop();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
