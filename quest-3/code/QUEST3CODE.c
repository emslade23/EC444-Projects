#include <stdio.h>
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

#define GPIO_INPUT_IO_0     4 //input
#define GPIO_INPUT_IO_1     5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0
#define WATER_GPIO 26 //A0
#define FINDDEVICE_GPIO 25 //A1

#define DEFAULT_VREF    1100

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR "192.168.1.146"
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif
#define PORT 8080

static const char *TAG = "example";
char *payload;
size_t payloadSize;


static esp_adc_cal_characteristics_t *adc_chars_thermistor;
static esp_adc_cal_characteristics_t *adc_chars_battery;
static const adc_channel_t channel_thermistor = ADC_CHANNEL_6; //A2
static const adc_channel_t channel_battery = ADC_CHANNEL_7; //A10
static const adc_atten_t atten_thermistor = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;
static const adc_unit_t unit2 = ADC_UNIT_2;

float temperature = 0;
uint32_t battery_voltage;

int flag = 0;
int stepCount = 0;
int waterCount = 10;

int thermistorOn = 1;
int batteryOn = 1;
int waterOn = 1;
int locateDeviceOn = 1;

static void udp_client_task(void *pvParameters)
{
    char command[128];
    char addr_str[128];
    char previousCommand[128];
    int isFirstCommand = 1;
    int addr_family;
    int ip_protocol;

    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 dest_addr;
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1) {
            //printf("still working\n");
            //printf("temp: %f,battery: %d,steps: %d\n",temperature, battery_voltage, stepCount);
            asprintf(&payload,"%f,%d,%d",temperature, battery_voltage, stepCount);

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            //ESP_LOGI(TAG, "Message sent");

            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, command, sizeof(command) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            if(isFirstCommand == 1) {
                isFirstCommand = 0;
                strcpy(previousCommand,command);
            }

            if(strcmp(previousCommand,command) != 0) { //aren't the same
                strcpy(command,"none");
            }



            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                //printf("im here");
                command[len] = 0; // Null-terminate whatever we received and treat like a string
            //    ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
            //    ESP_LOGI(TAG, "%s", rx_buffer);
                printf("Message: %s\n", command);
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    //vTaskDelete(NULL);
}

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

static void voltageDivider()
{
    //Configure ADC 2
    adc2_config_channel_atten((adc2_channel_t)channel_battery, atten_thermistor);

    //Characterize ADC 2
    adc_chars_battery = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit2, atten_thermistor, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars_battery);

    //Continuously sample ADC2
    while (1) {
        int adc_reading = 0;
        //Multisampling
        for (int i = 0; i < 10; i++) {
            adc_reading += adc2_get_raw(channel_battery,ADC_WIDTH_BIT_12,&adc_reading);
        }
        adc_reading /= 10;

        //Convert adc_reading to voltage in mV
        battery_voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars_battery);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    flag ^= 1;
}

static void stepCounter(void* arg)
{
    while(1) {
        if (flag == 1) {
            stepCount++;
            //printf("%d tap\n", stepCount);
            flag = 0;
           // gpio_set_level(BLINK_GPIO, 1);
            vTaskDelay(300 / portTICK_RATE_MS);
        }
        flag = 0;
        vTaskDelay(10 / portTICK_RATE_MS);
       // gpio_set_level(BLINK_GPIO, 0);
    }
}

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

static void waterReminder() {
    gpio_pad_select_gpio(WATER_GPIO);
    gpio_set_direction(WATER_GPIO, GPIO_MODE_OUTPUT);
    int cnt = 0;

    while (1){
        if (waterOn == 1) {
            cnt++;
            //printf("water: %d\n",waterCount);
            if (cnt == waterCount) {
                cnt = 4;
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
        } else if(waterOn == 0){
            gpio_set_level(WATER_GPIO, 0);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }
}

static void findDevice() {
    gpio_pad_select_gpio(FINDDEVICE_GPIO);
    gpio_set_direction(FINDDEVICE_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        if (locateDeviceOn == 1) {
            gpio_set_level(FINDDEVICE_GPIO, 1);
            vTaskDelay(1000 / portTICK_RATE_MS);
            gpio_set_level(FINDDEVICE_GPIO, 0);
            vTaskDelay(1000 / portTICK_RATE_MS);
        } else if(locateDeviceOn == 0) {
            gpio_set_level(FINDDEVICE_GPIO, 0);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    initializeVibrationSensor();

    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(stepCounter, "stepCounter", 2048, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(thermistor,"thermistor", 4096, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(voltageDivider, "voltageDivider", 4096, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(waterReminder, "waterReminder", 4096, NULL, configMAX_PRIORITIES-3, NULL);
    xTaskCreate(findDevice, "findDevice", 4096, NULL, configMAX_PRIORITIES-4, NULL);


    while(1) {
        //payloadSize = snprintf(NULL,0,"temp: %f,battery: %d,steps: %d\n",temperature, battery_voltage, stepCount);
        //payload = (char *)malloc(payloadSize + 1);
        //snprintf(payload, payloadSize + 1,"temp: %f,battery: %d,steps: %d\n",temperature, battery_voltage, stepCount);

       //int length = asprintf(&payload,"what");

        //printf("temp: %f,battery: %d,steps: %d\n",temperature, battery_voltage, stepCount);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
