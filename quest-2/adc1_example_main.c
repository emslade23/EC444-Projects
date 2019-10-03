// defines for displays
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_vfs_dev.h"
#include "esp_adc_cal.h"

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

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   10         //Multisampling

uint32_t voltage;
uint32_t distance;

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
    uint16_t fonttable[127];
    uint16_t map[5];
    char buf[5];
    fonttable[0] = 0b0000000000000001;
    fonttable[1] = 0b0000000000000010;
    fonttable[2] = 0b0000000000000100;
    fonttable[3] = 0b0000000000001000;
    fonttable[4] = 0b0000000000010000;
    fonttable[5] = 0b0000000000100000;
    fonttable[6] = 0b0000000001000000;
    fonttable[7] = 0b0000000010000000;
    fonttable[8] = 0b0000000100000000;
    fonttable[9] = 0b0000001000000000;
    fonttable[10] = 0b0000010000000000;
    fonttable[11] = 0b0000100000000000;
    fonttable[12] = 0b0001000000000000;
    fonttable[13] = 0b0010000000000000;
    fonttable[14] = 0b0100000000000000;
    fonttable[15] = 0b1000000000000000;
    fonttable[16] = 0b0000000000000000;
    fonttable[17] = 0b0000000000000000;
    fonttable[18] = 0b0000000000000000;
    fonttable[19] = 0b0000000000000000;
    fonttable[20] = 0b0000000000000000;
    fonttable[21] = 0b0000000000000000;
    fonttable[22] = 0b0000000000000000;
    fonttable[23] = 0b0000000000000000;
    fonttable[24] = 0b0001001011001001;
    fonttable[25] = 0b0001010111000000;
    fonttable[26] = 0b0001001011111001;
    fonttable[27] = 0b0000000011100011;
    fonttable[28] = 0b0000010100110000;
    fonttable[29] = 0b0001001011001000;
    fonttable[30] = 0b0011101000000000;
    fonttable[31] = 0b0001011100000000;
    fonttable[32] = 0b0000000000000000; //
    fonttable[33] = 0b0000000000000110; // !
    fonttable[34] = 0b0000001000100000; // "
    fonttable[35] = 0b0001001011001110; // #
    fonttable[36] = 0b0001001011101101; // $
    fonttable[37] = 0b0000110000100100; // %
    fonttable[38] = 0b0010001101011101; // &
    fonttable[39] = 0b0000010000000000; // '
    fonttable[40] = 0b0010010000000000; // (
    fonttable[41] = 0b0000100100000000; // )
    fonttable[42] = 0b0011111111000000; // *
    fonttable[43] = 0b0001001011000000; // +
    fonttable[44] = 0b0000100000000000; // ,
    fonttable[45] = 0b0000000011000000; // -
    fonttable[46] = 0b0000000000000000; // .
    fonttable[47] = 0b0000110000000000; // /
    fonttable[48] = 0b0000110000111111; // 0
    fonttable[49] = 0b0000000000000110; // 1
    fonttable[50] = 0b0000000011011011; // 2
    fonttable[51] = 0b0000000010001111; // 3
    fonttable[52] = 0b0000000011100110; // 4
    fonttable[53] = 0b0010000001101001; // 5
    fonttable[54] = 0b0000000011111101; // 6
    fonttable[55] = 0b0000000000000111; // 7
    fonttable[56] = 0b0000000011111111; // 8
    fonttable[57] = 0b0000000011101111; // 9
    fonttable[58] = 0b0001001000000000; // :
    fonttable[59] = 0b0000101000000000; // ;
    fonttable[60] = 0b0010010000000000; // <
    fonttable[61] = 0b0000000011001000; // =
    fonttable[62] = 0b0000100100000000; // >
    fonttable[63] = 0b0001000010000011; // ?
    fonttable[64] = 0b0000001010111011; // @
    fonttable[65] = 0b0000000011110111; // A
    fonttable[66] = 0b0001001010001111; // B
    fonttable[67] = 0b0000000000111001; // C
    fonttable[68] = 0b0001001000001111; // D
    fonttable[69] = 0b0000000011111001; // E
    fonttable[70] = 0b0000000001110001; // F
    fonttable[71] = 0b0000000010111101; // G
    fonttable[72] = 0b0000000011110110; // H
    fonttable[73] = 0b0001001000000000; // I
    fonttable[74] = 0b0000000000011110; // J
    fonttable[75] = 0b0010010001110000; // K
    fonttable[76] = 0b0000000000111000; // L
    fonttable[77] = 0b0000010100110110; // M
    fonttable[78] = 0b0010000100110110; // N
    fonttable[79] = 0b0000000000111111; // O
    fonttable[80] = 0b0000000011110011; // P
    fonttable[81] = 0b0010000000111111; // Q
    fonttable[82] = 0b0010000011110011; // R
    fonttable[83] = 0b0000000011101101; // S
    fonttable[84] = 0b0001001000000001; // T
    fonttable[85] = 0b0000000000111110; // U
    fonttable[86] = 0b0000110000110000; // V
    fonttable[87] = 0b0010100000110110; // W
    fonttable[88] = 0b0010110100000000; // X
    fonttable[89] = 0b0001010100000000; // Y
    fonttable[90] = 0b0000110000001001; // Z
    fonttable[91] = 0b0000000000111001; // [
    fonttable[92] = 0b0010000100000000; //
    fonttable[93] = 0b0000000000001111; // ]
    fonttable[94] = 0b0000110000000011; // ^
    fonttable[95] = 0b0000000000001000; // _
    fonttable[96] = 0b0000000100000000; // `
    fonttable[97] = 0b0001000001011000; // a
    fonttable[98] = 0b0010000001111000; // b
    fonttable[99] = 0b0000000011011000; // c
    fonttable[100] = 0b0000100010001110; // d
    fonttable[101] = 0b0000100001011000; // e
    fonttable[102] = 0b0000000001110001; // f
    fonttable[103] = 0b0000010010001110; // g
    fonttable[104] = 0b0001000001110000; // h
    fonttable[105] = 0b0001000000000000; // i
    fonttable[106] = 0b0000000000001110; // j
    fonttable[107] = 0b0011011000000000; // k
    fonttable[108] = 0b0000000000110000; // l
    fonttable[109] = 0b0001000011010100; // m
    fonttable[110] = 0b0001000001010000; // n
    fonttable[111] = 0b0000000011011100; // o
    fonttable[112] = 0b0000000101110000; // p
    fonttable[113] = 0b0000010010000110; // q
    fonttable[114] = 0b0000000001010000; // r
    fonttable[115] = 0b0010000010001000; // s
    fonttable[116] = 0b0000000001111000; // t
    fonttable[117] = 0b0000000000011100; // u
    fonttable[118] = 0b0010000000000100; // v
    fonttable[119] = 0b0010100000010100; // w
    fonttable[120] = 0b0010100011000000; // x
    fonttable[121] = 0b0010000000001100; // y
    fonttable[122] = 0b0000100001001000; // z
    fonttable[123] = 0b0000100101001001; // {
    fonttable[124] = 0b0001001000000000; // |
    fonttable[125] = 0b0010010010001001; // }
    fonttable[126] = 0b0000010100100000; // ~
    fonttable[127] = 0b0011111111111111;

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
      sprintf(buf,"%d",voltage);

        for (int i = 0; buf[i]; i++) {
            int num = (int) buf[i];
            map[i] = fonttable[num];
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

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void volt_div()
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_10);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        ///convert raw to distance
        distance = 5461 /(voltage - 17) -2 ;
        printf("Raw: %d\tVoltage: %dmV\t Distance: %dcm\n", adc_reading, voltage, distance);
          vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    i2c_example_master_init();

    xTaskCreate(volt_div, "volt_div", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(test_alpha_display,"test_alpha_display", 4096, NULL, configMAX_PRIORITIES-1, NULL);
}
