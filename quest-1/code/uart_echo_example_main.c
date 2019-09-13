/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_vfs_dev.h"

/**
 * This is an example which echos any data it receives on UART1 back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: UART1
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below
 */

#define ECHO_TEST_TXD  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_RXD  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)

#define LED 13

static void echo_task(void *arg)
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
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    //esp_vfs_dev_uart_use_driver(UART_NUM_0);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        uart_write_bytes(UART_NUM_0, (const char *) data, len);
    }
}

// function for counting length of buf
int inputlength(char* buf) {
    int m;
    int charcount;

    charcount = 0;
    for (m = 0; buf[m]; m++) {
        charcount ++;
    }

    return charcount;
}

void app_main(void)
{
    //xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);

    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) );
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    int switchmodes = 1;
    char buf[5];

    int on = 0;
    gpio_pad_select_gpio(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);

    printf("Mode 1: LED toggle, Mode 2: echo, Mode 3: hexadecimal display - 's' to toggle modes\n");
    printf("Mode 1: Enter 't' to toggle on-board led on and off\n");

    while(1) {

        switch (switchmodes) {
            case 1:
                // toggles led light on and off based with 't' as input
                gets(buf);
                if (buf[0] == 't') {
                    if (on == 0) {
                        printf("t, led turns on\n");
                        gpio_set_level(LED, 1);
                        on = 1;
                    } else {
                        printf("t, led turns off\n");
                        gpio_set_level(LED, 0);
                        on = 0;
                    }
                } else if (buf[0] == 's') {
                    printf("Mode 2: Enter anything and it will be echoed back\n");
                    switchmodes = 2;
                } else {
                    printf("String is not t\n");
                }
                vTaskDelay(50 / portTICK_RATE_MS);
                break;
            case 2:
                gets(buf);
                int charcount;
                charcount = inputlength(buf);

                if (buf[0] == 's' && charcount == 1) {
                    printf("Mode 3: some display dexadecimal stuff\n");
                    switchmodes = 3;
                } else {
                    printf("echo: %s\n", buf);
                }
                vTaskDelay(50 / portTICK_RATE_MS);
                break;
            case 3:
                gets(buf);
                if (buf[0] == 'u') {
                    printf("executing mode 3 functions\n");
                } else if (buf[0] == 's') {
                    printf("Mode 1: Enter 't' to toggle on-board led on and off\n");
                    switchmodes = 1;
                } else {
                    printf("lalla\n");
                }
                vTaskDelay(50 / portTICK_RATE_MS);
                break;
        }

    }

}
