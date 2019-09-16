#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_vfs_dev.h"


#define BUF_SIZE (1024)
#define LED 13


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
                    printf("Mode 3: Dex to hex mode\n");
                    switchmodes = 3;
                } else {
                    printf("echo: %s\n", buf);
                }
                vTaskDelay(50 / portTICK_RATE_MS);
                break;
            case 3:
                printf("Enter an integer: \n");
                gets(buf);
                int charcountt, num;
                charcountt = inputlength(buf);

                if (buf[0] == 's' && charcountt == 1) {
                    printf("Mode 1: LED toggle with 't'\n");
                    switchmodes = 1;
                } else {
                    num = atoi(buf);
                    char hexnum[100];
                    int i = 0;
                    long int remainder = 0;
                    while(num != 0) {
                        remainder = num % 16;
                        if(remainder < 10) {
                            remainder = remainder + 48;
                        } else {
                            remainder = remainder + 55;
                        }
                        hexnum[i++] = remainder;
                        num = num / 16;
                    }
                    printf("Hex:");
                    for(int j = i - 1; j >= 0; j--) {
                        printf("%c", hexnum[j]);
                    }
                    printf("\n");
                }
                vTaskDelay(50 / portTICK_RATE_MS);
                break;
        }

    }

}
