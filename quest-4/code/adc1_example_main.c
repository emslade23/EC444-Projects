#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include "esp_attr.h"
#include "driver/mcpwm.h"
///#include "soc/mcpwm_periph.h"
#include "driver/i2c.h"

#include "driver/gpio.h"
#include "esp_vfs_dev.h"
#include "alpha display.h"

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

#define SLAVE_ADDR_2                       0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// ADXL343
#define SLAVE_ADDR                         0x62  //default value of slave address 0x62

#define REG1                               0x00 //initiate ranging Register
#define VAL1                               0x04 //intiate range value
#define REG2                               0x8f //register to read for high and low
#define REG3                               0x01 //register to repeatedly read for LSB

//microlidar defines
#define ECHO_TEST_TXD  16//(GPIO_NUM_4) white in RX
#define ECHO_TEST_RXD  17//(GPIO_NUM_5) green in TX
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
#define BUF_SIZE (1024)

#define LEFT_LIDAR_TWO_TXD  12//(GPIO_NUM_4) white in RX
#define LEFT_LIDAR_TWO_RXD  27//(GPIO_NUM_5) green in TX
#define LEFT_LIDAR_TWO_RTS  (UART_PIN_NO_CHANGE)
#define LEFT_LIDAR_TWO_CTS  (UART_PIN_NO_CHANGE)

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   1          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34(A2) if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;
uint32_t voltage;

double rpm = 0;
double speed = 0;

uint16_t leftDistanceOne = 0;
uint16_t leftDistanceTwo = 0;

// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init(){
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
   if (err == ESP_OK) {printf("- initialized: yes\n");}

   // Data in MSB mode
   i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
 }

 // Alphanumeric Functions /////////////////////////////////////////////////////
// Turn on oscillator for alpha display
/* int alpha_oscillator() {
   int ret;
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, ( SLAVE_ADDR_2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
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


// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
i2c_cmd_handle_t cmd = i2c_cmd_link_create();
i2c_master_start(cmd);
i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
i2c_master_stop(cmd);
int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
i2c_cmd_link_delete(cmd);
return err;
}*/


// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {

i2c_cmd_handle_t cmd = i2c_cmd_link_create();
i2c_master_start(cmd);
i2c_master_write_byte(cmd, (SLAVE_ADDR<< 1)| WRITE_BIT, ACK_CHECK_EN);
i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
i2c_master_stop(cmd);
int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
i2c_cmd_link_delete(cmd);
return ret;
}

// Read register
uint8_t readRegister(uint8_t reg) {
// YOUR CODE HERE
uint8_t data;
i2c_cmd_handle_t cmd = i2c_cmd_link_create();
i2c_master_start(cmd);
i2c_master_write_byte(cmd, (SLAVE_ADDR << 1)| WRITE_BIT, ACK_CHECK_EN);
i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
i2c_master_start(cmd);
i2c_master_write_byte(cmd, (SLAVE_ADDR << 1)| READ_BIT, ACK_CHECK_EN);
i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS);
//i2c_master_write_byte(cmd, reg, NACK_VAL);
i2c_master_stop(cmd);
i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
i2c_cmd_link_delete(cmd);
return data;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {

uint8_t d1, d2;

i2c_cmd_handle_t cmd = i2c_cmd_link_create();
i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
i2c_master_start(cmd);
i2c_master_write_byte(cmd, (SLAVE_ADDR << 1)|WRITE_BIT, ACK_CHECK_EN);
i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
i2c_master_stop(cmd); //for this device, it myst first stop the condition before a new start condition
i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
i2c_master_start(cmd2);
i2c_master_write_byte(cmd2, (SLAVE_ADDR << 1)| READ_BIT, ACK_CHECK_EN);
i2c_master_read_byte(cmd2, &d1, ACK_VAL);
i2c_master_read_byte(cmd2, &d2, ACK_VAL);
i2c_master_stop(cmd2);
i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 100 / portTICK_RATE_MS);
i2c_cmd_link_delete(cmd);
i2c_cmd_link_delete(cmd2);
int16_t dtemp = d1 << 8;
int16_t data = dtemp| d2;
return data;
}

int16_t frontDistance = 0;
int collision = 0;

// Task to continuously poll lidar
void read_lidar() {
  while (1){
    writeRegister(REG1, VAL1);

    frontDistance = read16(REG2);
    if (frontDistance < 19) {
      collision = 1;
    } else {
      collision = 0;
    }
    vTaskDelay(200 / portTICK_RATE_MS);
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

    //left lidar 2
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, LEFT_LIDAR_TWO_TXD, LEFT_LIDAR_TWO_RXD, LEFT_LIDAR_TWO_RTS, LEFT_LIDAR_TWO_CTS);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    uint8_t *dataOne = (uint8_t *) malloc(11);
    uint8_t *dataTwo = (uint8_t *) malloc(11);

    while (1) {
        // Read data from the UART
        uart_read_bytes(UART_NUM_1, dataOne, 9, 10 / portTICK_RATE_MS);
        uart_read_bytes(UART_NUM_2, dataTwo, 9, 10 / portTICK_RATE_MS);

        if(dataOne[0] == 0x59 && dataOne[1] == 0x59) {
          leftDistanceOne = (dataOne[3]<<8)|(dataOne[2]);
        }
        if(dataTwo[0] == 0x59 && dataTwo[1] == 0x59) {
          leftDistanceTwo = (dataTwo[3]<<8)|(dataTwo[2]);
        }
    }
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
    //
    // printf("Calibrating steering\n");
    // vTaskDelay(3000 / portTICK_PERIOD_MS);
    // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2500); // HIGH signal in microseconds
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 600);  // LOW signal in microseconds
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1300); // NEUTRAL signal in microseconds
    // vTaskDelay(2000 / portTICK_PERIOD_MS);

    printf("Finish calibration\n");

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1300);
}

static void encoder() {
  //Configure ADC
      adc1_config_width(ADC_WIDTH_BIT_10);
      adc1_config_channel_atten(channel, atten);

  //Characterize ADC
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars);
  //print_char_val_type(val_type);

  //Continuously sample ADC1
  while (1) {
      float adc_reading = 0;
      //Multisampling
      for (int i = 0; i < NO_OF_SAMPLES; i++) {
          adc_reading += adc1_get_raw((adc1_channel_t)channel);
      }
      adc_reading /= NO_OF_SAMPLES;
      voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

      //printf("Voltage: %d\n", voltage);
      vTaskDelay(pdMS_TO_TICKS(10));
  }
}

double pulseCount = 0;

static void pulseCounting() {
  while (1) {
    if (voltage > 500) {
      pulseCount++;
      while(voltage > 500) {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    } else if (voltage < 500) {
      pulseCount++;
      while(voltage < 500) {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }
  }
}

static void straightLineError() {
  int difference = leftDistanceOne - leftDistanceTwo;
  while (1) {
    if (collision == 0) {
      if(difference > 5) { //turn left a little
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1900);
      } else if (difference < -5) { //turn right a little
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
      } else if (leftDistanceOne < 15) {
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
      } else {
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1450);
      }
    } else if (collision == 1) {
      mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1450);
      mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

  int setspeed = 1310;

static void setSpeedController() {
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, setspeed);
  while (1) {
    if (collision == 0) {
      if(speed > 0.413328) {
        setspeed++;
      } else if (speed < 0.413328) {
        setspeed--;
      } else {
        setspeed = setspeed;
      }
    } else if (collision == 1) {
      //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, setspeed);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void app_main(void)
{
      i2c_master_init();
      calibrateESC();

      vTaskDelay(1000 / portTICK_PERIOD_MS);

      xTaskCreate(setSpeedController, "set_speed_controller", 1024, NULL, configMAX_PRIORITIES, NULL);
      xTaskCreate(read_lidar,"read_lidar", 4096, NULL, 5, NULL);
      xTaskCreate(encoder,"encoder", 4096, NULL, configMAX_PRIORITIES, NULL);
      xTaskCreate(pulseCounting,"pulse_counting", 4096, NULL, configMAX_PRIORITIES, NULL);
      xTaskCreate(microLidar, "micro_lidar", 1024, NULL, configMAX_PRIORITIES, NULL);
      xTaskCreate(straightLineError, "straight_Line_Error", 1024, NULL, configMAX_PRIORITIES, NULL);

      while (1) {
        speed = pulseCount * 0.051666;
        printf("Pulse Count = %f\n", pulseCount);
        pulseCount = 0;
            printf("Speed = %f m/s\n", speed);

        char speed_char[5]; // = NULL;
        gcvt(s, 5, speed_char);
        alpha_display(speed_char);


        printf("left One Distance = %d cm\n", leftDistanceOne);
        printf("Left Two Distance = %d cm\n", leftDistanceTwo);
        printf("set speed = %d \n", setspeed);
        printf("Front Distance: %d cm\n", frontDistance);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
}
