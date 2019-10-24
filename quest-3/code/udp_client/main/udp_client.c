//Author: Amy Dong, Quianna Mortimer, Elizabeth Slade

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

#define DEFAULT_VREF    1100

#define HOST_IP_ADDR "192.168.1.146" //ip address for communicating with the node.js server
#define PORT 3030

static const char *TAG = "example";
char *payload; //message being sent from the ESP32 to the server
size_t payloadSize;

//Setting up ADCs for thermistor and battery
static esp_adc_cal_characteristics_t *adc_chars_thermistor;
static esp_adc_cal_characteristics_t *adc_chars_battery;
static const adc_channel_t channel_thermistor = ADC_CHANNEL_6; //A2
static const adc_channel_t channel_battery = ADC_CHANNEL_7; //A10
static const adc_atten_t atten_thermistor = ADC_ATTEN_DB_0;
static const adc_atten_t atten_battery = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;
static const adc_unit_t unit2 = ADC_UNIT_2;

//Global variables
float temperature = 0;
uint32_t battery_voltage;

int timePassed = 0; //time elapsed
int flag = 0; //interrupt for the vibration sensor
int stepCount = 0; // number of steps
int waterCount = 10; // the number of seconds thats scheduled for the water reminder to be triggered. Default is 10 seconds.

//Turning on and off functions
int thermistorOn = 1;
int batteryOn = 1;
int waterOn = 0;
int locateDeviceOn = 0;

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

            //Sending ESP32 data to node.js
            asprintf(&payload,"%f,%d,%d,%d",temperature, battery_voltage, stepCount,timePassed);

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

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
                if(newCommand == 1) {
                    strcpy(previousCommand,command);
                    newCommand = 0;
                }
                if (strncmp(command,"water on",8) == 0) { //
                    waterOn = 1;
                    if(strcmp(command,"water on") == 0) {// no numbers in water command
                        printf("You have turned on the water scheduling function and the set time to alert is %d seconds.\n", waterCount);
                    } else { //there is a number in the water command
                        if (1 == sscanf(command,"%*[^0123456789]%d", &waterCount)) {
                        printf("You have turned on the water scheduling function and set the alert time to %d seconds.\n", waterCount);
                        }
                    }
                } else if(strcmp(command,"water off") == 0) {
                    waterOn = 0;
                    printf("Water reminder is off.\n");
                } else if(strcmp(command,"find device on") == 0) {
                    locateDeviceOn = 1;
                    printf("Find device is on.\n");
                } else if(strcmp(command,"find device off") == 0) {
                    locateDeviceOn = 0;
                    printf("Find device is off.\n");
                } else if(strcmp(command,"none") == 0) {
                    printf("Waiting for new commands...\n");
                } else if(strcmp(command,"reset steps") == 0) {
                    printf("reset steps to 0\n");
                    stepCount = 0;
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

// Task for reading the thermistor at ADC1
static void thermistor()
{
    //Configure ADC
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel_thermistor, atten_thermistor);

    //Characterize ADC
    adc_chars_thermistor = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten_thermistor, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars_thermistor);

    double thermometerResistance = 0.0;
    double a = 5, b = 1000, c = 216;
    double r_zero = 10000, t_zero = 298.15, beta = 3435, ktoc = 273.15;

    //Continuously sample ADC1
    while (1) {
        if(thermistorOn == 1) {
            uint32_t adc_reading = 0;
            //Multisampling
            for (int i = 0; i < 10; i++) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel_thermistor);
            }
            adc_reading /= 10;

            //Convert adc_reading to voltage in mV
            uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars_thermistor);

            thermometerResistance = a * c / (voltage/b) - c;
            temperature = (t_zero * beta)/(t_zero * log(thermometerResistance/r_zero) + beta) - ktoc - 2;
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// Task for reading the battery at ADC2
static void voltageDivider()
{
    //Configure ADC 2
    adc2_config_channel_atten((adc2_channel_t)channel_battery, atten_battery);

    //Characterize ADC 2
    adc_chars_battery = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit2, atten_battery, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars_battery);

    //Continuously sample ADC2
    while (1) {
        int adc_reading2 = 0;
        //Multisampling
        for (int i = 0; i < 2; i++) {
            adc_reading2 += adc2_get_raw(channel_battery,ADC_WIDTH_BIT_12,&adc_reading2);
        }
        adc_reading2 /= 2;

        //Convert adc_reading to voltage in mV
        battery_voltage = esp_adc_cal_raw_to_voltage(adc_reading2, adc_chars_battery);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//Interrupt for the vibration sensor
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    flag ^= 1;
}

//Records number of steps taken by the user
static void stepCounter(void* arg)
{
    while(1) {
        if (flag == 1) {
            stepCount++;
            flag = 0;
            vTaskDelay(300 / portTICK_RATE_MS);
        }
        flag = 0;
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

//initializing the vibration sensor
static void initializeVibrationSensor() {
    gpio_config_t io_conf;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_POSEDGE);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
}

// task for setting the water reminder
static void waterReminder() {
    gpio_pad_select_gpio(WATER_GPIO);
    gpio_set_direction(WATER_GPIO, GPIO_MODE_OUTPUT);
    int cnt = 0;

    while (1){
        if (waterOn == 1) { //water on command
            cnt++;
            if (cnt == waterCount) {
                cnt = 4; // when the reminder is triggered, the blue LED blinks three times
                gpio_set_level(WATER_GPIO, 1);
                vTaskDelay(1000 / portTICK_RATE_MS);
                gpio_set_level(WATER_GPIO, 0);
                vTaskDelay(1000 / portTICK_RATE_MS);
                gpio_set_level(WATER_GPIO, 1);
                vTaskDelay(1000 / portTICK_RATE_MS);
                gpio_set_level(WATER_GPIO, 0);
                vTaskDelay(1000 / portTICK_RATE_MS);
                gpio_set_level(WATER_GPIO, 1);
            }
            vTaskDelay(1000 / portTICK_RATE_MS);
            gpio_set_level(WATER_GPIO, 0);
        } else if(waterOn == 0){ // LED is off if water off command is receieved
            gpio_set_level(WATER_GPIO, 0);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }
}

// task for finding the device, turns the red led on and off
static void findDevice() {
    gpio_pad_select_gpio(FINDDEVICE_GPIO);
    gpio_set_direction(FINDDEVICE_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        if (locateDeviceOn == 1) { // find device on command makes the red LED blink
            gpio_set_level(FINDDEVICE_GPIO, 1);
            vTaskDelay(1000 / portTICK_RATE_MS);
            gpio_set_level(FINDDEVICE_GPIO, 0);
            vTaskDelay(1000 / portTICK_RATE_MS);
        } else if(locateDeviceOn == 0) { // find device off command turns off the red LED
            gpio_set_level(FINDDEVICE_GPIO, 0);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }
}

void app_main(void)
{
  //initializing
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    initializeVibrationSensor();
//creating tasks
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(voltageDivider, "voltageDivider", 4096, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(stepCounter, "stepCounter", 2048, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(thermistor,"thermistor", 4096, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(waterReminder, "waterReminder", 4096, NULL, configMAX_PRIORITIES-3, NULL);
    xTaskCreate(findDevice, "findDevice", 4096, NULL, configMAX_PRIORITIES-4, NULL);

//saving elapsed time
    while(1) {
        timePassed++;
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
