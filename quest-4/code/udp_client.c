//library includes
#include <string.h>
#include <sys/param.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "esp_adc_cal.h"
#include "esp_vfs_dev.h"
#include "esp_adc_cal.h"
#include <math.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define GPIO_INPUT_IO_0     4 //GPIO input for vibration sensor
#define GPIO_INPUT_IO_1     5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0
#define WATER_GPIO 26 //A0  //Blue led that blinks when water reminder is triggered
#define FINDDEVICE_GPIO 25 //A1 //Red led that blinks when find device is on
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

#define DEFAULT_VREF    1100

#define HOST_IP_ADDR "192.168.1.144" //ip address for communicating with the node.js server
#define PORT 3030

static const char *TAG = "example";
char *payload; //message being sent from the ESP32 to the server
size_t payloadSize;



static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 18);    //Set GPIO 18 as PWM0A, to which servo is connected
}
//Global variables
uint32_t battery_voltage;

int timePassed = 0; //time elapsed
int flag = 0; //interrupt for the vibration sensor
int stepCount = 0; // number of steps
int waterCount = 10; // the number of seconds thats scheduled for the water reminder to be triggered. Default is 10 seconds.

//Turning on and off functions
int waterOn = 0;
int locateDeviceOn = 0;


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
}
//establishing udp socket between the ESP32 and the node.js server
static void udp_client_task(void *pvParameters)
{
    char command[128];
    char addr_str[128];
    char previousCommand[128];
    int newCommand = 1;
    int addr_family;
    int ip_protocol;

    while (1) {
      //configuring socket
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

        // establishing socket
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1) {

            //Recieveing data from node.js
            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, command, sizeof(command) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                command[len] = 0; // Null-terminate whatever we received and treat like a string

                //Making changes to the states of the functions depending on the command
                if(strcmp(previousCommand,command) == 0) {
                    strcpy(command,"none");
                } else {
                    strcpy(previousCommand,command);
                }
                if (strncmp(command,"Start",8) == 0) { //
                   printf("Start!!! Zoom Zoom\n");
                   mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1800); // NEUTRAL signal in microseconds
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                } else if(strcmp(command,"Stop") == 0) {
                    waterOn = 0;
                    printf("Stop!!!\n");
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400); // NEUTRAL signal in microseconds
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
                } else {
                    printf("command processing...\n");
                }
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
}




static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

/**
 * @brief Configure MCPWM module
 */



void app_main(void)
{
  //initializing
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
     calibrateESC();

//creating tasks
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, configMAX_PRIORITIES, NULL);

}