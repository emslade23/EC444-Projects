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

// 14-Segment Display
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

//display functions
static void i2c_example_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK) {printf("- initialized: yes\n\n");}

    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

////////////////////////////////////////////////////////////////////////////////

// Alphanumeric Functions //////////////////////////////////////////////////////

// Turn on oscillator for alpha display
int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set blink rate to off
int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

static void test_alpha_display() {
    // Debug
    uint16_t fonttable[10];
    uint16_t map[5];
    char buf[5];
    fonttable[0] = 0b0000110000111111; // 0
    fonttable[1] = 0b0000000000000110; // 1
    fonttable[2] = 0b0000000011011011; // 2
    fonttable[3] = 0b0000000010001111; // 3
    fonttable[4] = 0b0000000011100110; // 4
    fonttable[5] = 0b0010000001101001; // 5
    fonttable[6] = 0b0000000011111101; // 6
    fonttable[7] = 0b0000000000000111; // 7
    fonttable[8] = 0b0000000011111111; // 8
    fonttable[9] = 0b0000000011101111; // 9

    int ret;

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}

    while (1) {
      sprintf(buf,"%d",battery_voltage);

        for (int i = 0; buf[i]; i++) {
            int num = (int) buf[i];
            map[i] = fonttable[num-48];
        }

        // Write to characters to bufferASDFGHJKLs
        uint16_t displaybuffer[8];

        displaybuffer[0] = map[0];
        displaybuffer[1] = map[1];
        displaybuffer[2] = map[2];
        displaybuffer[3] = map[3];

      // Send commands characters to display over I2C
      i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
      i2c_master_start(cmd4);
      i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
      i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
      for (uint8_t i=0; i<8; i++) {
        i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
      }
      i2c_master_stop(cmd4);
      ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
      i2c_cmd_link_delete(cmd4);

    }
}

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
        vTaskDelay(pdMS_TO_TICKS(10));
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
        vTaskDelay(pdMS_TO_TICKS(10));
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
        temperature = (t_zero * beta)/(t_zero * log(thermometerResistance/r_zero) + beta) - ktoc - 5;
        //printf("Temperature: %f degrees celcius\n", temp);
        vTaskDelay(pdMS_TO_TICKS(10));
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
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    i2c_example_master_init();

    xTaskCreate(rangefinder,"rangefinder", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(ultrasound,"ultrasound", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(thermistor,"thermistor", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(voltageDivider, "voltageDivider", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(test_alpha_display,"test_alpha_display", 4096, NULL, configMAX_PRIORITIES-1, NULL);

    while(1) {
        //printf("Rangefinder distance: %f\n", rangefinder_distance);
      //  printf("Ultrasound distance: %f\n", sultrasound_distance);
    //    printf("Thermistor: %f\n", temperature);
        printf(" %f,%f,%f,%d\n",rangefinder_distance,ultrasound_distance,temperature,battery_voltage);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

}
