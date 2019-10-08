#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "esp_adc_cal.h"
#include "esp_vfs_dev.h"
#include "esp_adc_cal.h"
#include <math.h>

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling


static esp_adc_cal_characteristics_t *adc_chars_rangefinder;
static esp_adc_cal_characteristics_t *adc_chars_ultrasound;
static esp_adc_cal_characteristics_t *adc_chars_thermistor;
static esp_adc_cal_characteristics_t *adc_chars_battery;
static const adc_channel_t channel_rangefinder = ADC_CHANNEL_6; //A2
static const adc_channel_t channel_ultrasound = ADC_CHANNEL_0; //A4
static const adc_channel_t channel_thermistor = ADC_CHANNEL_3; //A3
static const adc_channel_t channel_battery = ADC_CHANNEL_7; //A10
static const adc_atten_t atten_rangefinder = ADC_ATTEN_DB_11;
static const adc_atten_t atten_ultrasound = ADC_ATTEN_DB_0;
static const adc_atten_t atten_thermistor = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;
static const adc_unit_t unit2 = ADC_UNIT_2;

float rangefinder_distance = 0;
float ultrasound_distance = 0;
float temperature = 0;
uint32_t battery_voltage;

static void voltageDivider()
{
    //Configure ADC
    adc2_config_channel_atten((adc2_channel_t)channel_battery, atten_ultrasound);

    //Characterize ADC
    adc_chars_battery = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit2, atten_ultrasound, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars_battery);
    //print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) {
        int adc_reading = 0;
        //Multisampling
        for (int i = 0; i < 10; i++) {
            adc_reading += adc2_get_raw(channel_battery,ADC_WIDTH_BIT_12,&adc_reading);
        }
        adc_reading /= 10;
        //Convert adc_reading to voltage in mV
        battery_voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars_battery);
        //battery_voltage_V = battery_voltage/1000.00;
//        printf("Raw: %d\tVoltage: %dmV\n", adc_reading, battery_voltage);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void rangefinder()
{
    //Configure ADC
        adc1_config_width(ADC_WIDTH_BIT_10);
        adc1_config_channel_atten(channel_rangefinder, atten_rangefinder);

    //Characterize ADC
    adc_chars_rangefinder = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten_rangefinder, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_rangefinder);


    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel_rangefinder);
        }
        adc_reading /= NO_OF_SAMPLES;

        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars_rangefinder);
        if(voltage <= 480) {
            rangefinder_distance = 49151.8275 * pow(voltage,-0.92265)/100;
        } else if(voltage <= 1495 && voltage > 480) {
            rangefinder_distance = 174026.07143 * pow(voltage,-1.14652)/100;
        } else if(voltage > 1495) {
            rangefinder_distance = (-0.02061 * voltage + 70.17877)/100;
        }

        //rangefinder_distance = voltage;

        //printf("rangefinder distance: %f m \n", distance);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void thermistor()
{
    //Configure ADC
        adc1_config_width(ADC_WIDTH_BIT_10);
        adc1_config_channel_atten(channel_thermistor, atten_thermistor);

    //Characterize ADC
    adc_chars_thermistor = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten_thermistor, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_thermistor);

    double thermometerResistance = 0.0;
    double a = 5, b = 1000, c = 216;
    double r_zero = 10000, t_zero = 298.15, beta = 3435, ktoc = 273.15;

    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < 1; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel_thermistor);
        }
        adc_reading /= 1;

        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars_thermistor);

        thermometerResistance = a * c / (voltage/b) - c;
        temperature = (t_zero * beta)/(t_zero * log(thermometerResistance/r_zero) + beta) - ktoc -4;
        //printf("Temperature: %f degrees celcius\n", temp);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void ultrasound()
{
    //Configure ADC
        adc1_config_width(ADC_WIDTH_BIT_10);
        adc1_config_channel_atten(channel_ultrasound, atten_ultrasound);

    //Characterize ADC
    adc_chars_ultrasound = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten_ultrasound, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_ultrasound);


    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel_ultrasound);
        }
        adc_reading /= NO_OF_SAMPLES;

        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars_ultrasound);
        //ultrasound_distance = adc_reading/1024 * 5;
        ultrasound_distance =  voltage / 6.4 * 2.54 / 100;
        //printf("Ultrasound distance: %f m \n", distance);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{

    xTaskCreate(rangefinder,"rangefinder", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(ultrasound,"ultrasound", 4096, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(thermistor,"thermistor", 4096, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(voltageDivider, "voltageDivider", 4096, NULL, configMAX_PRIORITIES-3, NULL);
int count = 0;

    while(1) {
        count++;
        //printf("Rangefinder distance: %f\n", rangefinder_distance);
      //  printf("Ultrasound distance: %f\n", sultrasound_distance);
    //    printf("Thermistor: %f\n", temperature);
        printf(" %d,%f,%f,%d,%f\n",count,rangefinder_distance,ultrasound_distance,battery_voltage,temperature);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}
