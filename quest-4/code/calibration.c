/* servo motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

void calibrateESC() {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 18);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 19);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    //mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config)

    while (1) {
    // calibrating run
    printf("Calibrating forward backwards\n");
    vTaskDelay(4000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 2100); // HIGH signal in microseconds
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 700);  // LOW signal in microseconds
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400); // NEUTRAL signal in microseconds
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400); // reset the ESC to neutral (non-moving) value

    printf("Calibrating steering\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2500); // HIGH signal in microseconds
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 600);  // LOW signal in microseconds
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1500); // NEUTRAL signal in microseconds

    printf("Finish calibration\n");
  }
// uint32_t angle, count;
//
// while (1) {
//         for (count = 0; count < SERVO_MAX_DEGREE; count++) {
//             printf("Angle of rotation: %d\n", count);
//             angle = servo_per_degree_init(count);
//             printf("pulse width: %dus\n", angle);
//             mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
//             vTaskDelay(10);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
//         }
//       }
}


/**
 * @brief Configure MCPWM module
 */
// void mcpwm_example_servo_control(void *arg)
// {
//     uint32_t angle, count;
//     //1. mcpwm gpio initialization
//     //mcpwm_example_gpio_initialize();
//
//     //2. initial mcpwm configuration
//     printf("Configuring Initial Parameters of mcpwm......\n");
//     mcpwm_config_t pwm_config;
//     pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
//     pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
//     pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
//     pwm_config.counter_mode = MCPWM_UP_COUNTER;
//     pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
//     mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
//     while (1) {
//         for (count = 0; count < SERVO_MAX_DEGREE; count++) {
//             printf("Angle of rotation: %d\n", count);
//             angle = servo_per_degree_init(count);
//             printf("pulse width: %dus\n", angle);
//             mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
//             vTaskDelay(10);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
//         }
//     }
// }

void app_main(void)
{
    calibrateESC();
    //printf("Testing servo motor.......\n");
    //xTaskCreate(mcpwm_example_servo_control, "mcpwm_example_servo_control", 4096, NULL, 5, NULL);
}
