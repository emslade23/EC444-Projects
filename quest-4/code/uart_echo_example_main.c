#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_types.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#define BLINK_RED 25
#define BLINK_GREEN 4
#define BLINK_BLUE 12

#define ECHO_TEST_TXD  16//(GPIO_NUM_4) white in RX
#define ECHO_TEST_RXD  17//(GPIO_NUM_5) green in TX
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload



// Flag for dt
int dt_complete = 0;

uint16_t distance = 0;

// Define timer interrupt handler
// void IRAM_ATTR timer_isr(void* arg)
// {
//     // Clear interrupt
//     TIMERG0.int_clr_timers.t0 = 1;
//     // Indicate timer has fired
//     dt_complete = 1;
//     //printf("timer is triggered\n");
// }

// static void periodic_timer_init()
// {
//     // Basic parameters of the timer
//     timer_config_t config;
//     //...
//     config.divider = TIMER_DIVIDER;
//     config.counter_dir = TIMER_COUNT_UP;
//     config.counter_en = TIMER_PAUSE;
//     config.alarm_en = TIMER_ALARM_EN;
//     config.intr_type = TIMER_INTR_LEVEL;
//     config.auto_reload = true;
//     timer_init(TIMER_GROUP_0, TIMER_0, &config);
//
//     // Timer's counter will initially start from value below
//     timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
//
//     // Configure the alarm value and the interrupt on alarm.
//     timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 0.1 * TIMER_SCALE);
//     timer_enable_intr(TIMER_GROUP_0, TIMER_0);
//     timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_isr,
//         NULL, ESP_INTR_FLAG_IRAM, NULL);
//
//     //start timer
//     timer_start(TIMER_GROUP_0, TIMER_0);
// }

double dt = 0.1;
int previous_error = 0;		// Set up PID loop
int integral = 0;
int setpoint = 50;
int error = 0;
int derivative = 0;
int output = 0;
int Kp = 1;
int Ki = 0;
int Kd = 0;


static void PID() {
  error = setpoint - distance;
  integral = integral + error * dt;
  derivative = (error - previous_error) / dt;
  output = Kp * error + Ki * integral + Kd * derivative;
  //printf("output = %d\n",output);
  previous_error = error;
  //printf("Error is %d\n", error);
  if(error == 0) {
    gpio_set_level(BLINK_GREEN, 1);
    gpio_set_level(BLINK_BLUE, 0);
    gpio_set_level(BLINK_RED, 0);
  } else if(error > 0) {
    gpio_set_level(BLINK_GREEN, 0);
    gpio_set_level(BLINK_BLUE, 1);
    gpio_set_level(BLINK_RED, 0);
  } else if(error < 0) {
    gpio_set_level(BLINK_GREEN, 0);
    gpio_set_level(BLINK_BLUE, 0);
    gpio_set_level(BLINK_RED, 1);
  }

}

static void microLidar(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(11);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1, data, 9, 20 / portTICK_RATE_MS);

        if(data[0] == 0x59 && data[1] == 0x59) {
          distance = (data[3]<<8)|(data[2]);
          //printf("distance: %d cm\n", distance);
        }
        // Write data back to the UART
        //uart_write_bytes(UART_NUM_1, (const char *) data, len);
        //vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    gpio_pad_select_gpio(BLINK_BLUE);
    gpio_set_direction(BLINK_BLUE, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(BLINK_GREEN);
    gpio_set_direction(BLINK_GREEN, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(BLINK_RED);
    gpio_set_direction(BLINK_RED, GPIO_MODE_OUTPUT);

    //periodic_timer_init();
    xTaskCreate(microLidar, "micro_lidar", 1024, NULL, 10, NULL);
    //Timer tiggers PID control every 100ms
    while (1) {
        //if (dt_complete == 1) {
          PID();
        //  dt_complete = 0;
          // Re-enable alarm
        //  TIMERG0.hw_timer[TIMER_0].config.alarm_en = 1;
        // }
       vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
