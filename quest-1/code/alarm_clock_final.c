#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_vfs_dev.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

//defines for display
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
/////////////////////////////////////////////////////

#define BLINK 26

// Defines for the seconds servo
#define SERVO_MIN_PULSEWIDTH 550 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2700 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate

// Defines for the hour servo
#define SERVO_MIN_PULSEWIDTH_HOUR 550 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_HOUR 2700 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE_HOUR 180 //Maximum angle in degree upto which servo can rotate

uint16_t font_table(int num) {
    uint16_t fonttable[10];
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
    return fonttable[num];
}

char hour[2];
char minute[2];
int int_hour = 0;
int int_minute = 0;
uint32_t counth = 0;
char alarmset[1];
char hralarmtime[2];
char mialarmtime[2];
int int_hralarmtime = 0;
int int_mialarmtime = 0;

bool isitdigit(char str[2])
{
    int i = 0;
    int counter = 0;
    for(i = 0; i < 2 ; i++) {
        if (str[i] >= '0' && str[i] <= '9'){
            counter++;
        }
    }
    if(counter == 2) {
        return true;
    } else {
        return false;
    }
}

// sets the initial time for the display
static void set_time() {

    // Set up routines
    // Turn on alpha oscillator

    // Asks the user to enter the hour
    printf(">> Enter the hour, if one digit add 0 before it: \n");
    gets(hour);
    printf("%s\n", hour);
    while(isitdigit(hour) == false || atoi(hour) > 23) { // Error check the input
        if(isitdigit(hour) == false) {
            printf("Error: Please enter only numbers:\n");
            gets(hour);
            printf("%s\n", hour);
        } else if(atoi(hour) > 23) {
            printf("Error: Please enter a number under 24:\n");
            gets(hour);
            printf("%s\n", hour);
        }
    }
    printf("Hour is set to %s.\n", hour);
    int_hour = atoi(hour);

    // Asks the user to enter the minute
    printf(">> Enter the minute, if one digit add 0 before it: \n");
    gets(minute);
    printf("%s\n", minute);
    while(isitdigit(minute) == false || atoi(minute) > 59) { // Error check the input
        if(isitdigit(minute) == false) {
            printf("Error: Please enter only numbers:\n");
            gets(minute);
            printf("%s\n", minute);
        } else if(atoi(minute) > 59) {
            printf("Error: Please enter a number under 60:\n");
            gets(minute);
            printf("%s\n", minute);
        }
    }
    printf("Minute is set to %s.\n", minute);
    int_minute = atoi(minute);
    counth = int_minute * 3;

    printf("Time is set to %d:%d.\n", int_hour, int_minute);

    // set alarm
    printf(">> Do you want to set an alarm? Type 1 for yes and any other key for no: \n");
    gets(alarmset);
    if(alarmset[0] == '1') {
        printf(">> Enter alarm hour time: \n");
        gets(hralarmtime);
        printf("%s\n", hralarmtime);
        while(isitdigit(hralarmtime) == false || atoi(hralarmtime) > 59) { // Error check the input
            if(isitdigit(hralarmtime) == false) {
                printf("Error: Please enter only numbers:\n");
                gets(hralarmtime);
                printf("%s\n", hralarmtime);
            } else if(atoi(hralarmtime) > 59) {
                printf("Error: Please enter a number under 60:\n");
                gets(hralarmtime);
                printf("%s\n", hralarmtime);
            }
        }
        printf("Alarm hour is set to %s.\n", hralarmtime);
        int_hralarmtime = atoi(hralarmtime);

        printf(">> Enter alarm minute time: \n");
        gets(mialarmtime);
        printf("%s\n", mialarmtime);
        while(isitdigit(mialarmtime) == false || atoi(mialarmtime) > 59) { // Error check the input
            if(isitdigit(mialarmtime) == false) {
                printf("Error: Please enter only numbers:\n");
                gets(mialarmtime);
                printf("%s\n", mialarmtime);
            } else if(atoi(mialarmtime) > 59) {
                printf("Error: Please enter a number under 60:\n");
                gets(mialarmtime);
                printf("%s\n", mialarmtime);
            }
        }
        printf("Alarm hour is set to %s.\n", mialarmtime);
        int_mialarmtime = atoi(mialarmtime);
    }
    printf("Alarm time is set to %d:%d.\n", int_hralarmtime, int_mialarmtime);

}

// Code for seconds servo
static void servo_initialize_seconds(void)
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 18);    //Set GPIO 18 as PWM0A, to which seconds servo is connected
}

static void servo_initialize_minutes(void)
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 19);    //Set GPIO 19 as PWM0A, to which minute is connected

}


// Calculate pulse width for seconds servo
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

uint32_t count;

// Code for seconds servo
void seconds_servo_control(void *arg)
{
    uint32_t angle;
    //1. mcpwm gpio initialization
    servo_initialize_seconds();

    //2. initial mcpwm configuration
    //printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    while (1) {
        for (count = 0; count < SERVO_MAX_DEGREE; count = count + 3) {
            printf("Angle of rotation: %d\n", count);
            angle = servo_per_degree_init(count);
            //printf("pulse width: %dus\n", angle);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
            vTaskDelay(100);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        }
    }
}

void minutes_servo_control(void *arg)
{
    uint32_t angle;
    //1. mcpwm gpio initialization
    servo_initialize_minutes();

    //2. initial mcpwm configuration
    //printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    while (1) {
        for (counth = counth; counth < SERVO_MAX_DEGREE; counth = counth + 3) {
            printf("hour count: %d\n", counth);
            angle = servo_per_degree_init(counth);
            //printf("pulse width: %dus\n", angle);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
            vTaskDelay(6000);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        }
        counth = 0;
    }
}

// init for display
static void i2c_example_master_init(){
    // Debug
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
    //if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    //if (err == ESP_OK) {printf("- initialized: yes\n\n");}

    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

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
// Set blink rate to off for display
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
// Set Brightness for display
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

// Minute increment function for the display
int min_increment(int minute, char min[2])
{
    if(minute == 59) {
        minute = 0;
        min[0] = '0';
        min[1] = '0';
    } else {
        minute++;
        if(min[1] == '9') {
            min[1] ='0';
            min[0]++;
        } else {
            min[1]++;
        }
    }
    return minute;
}

int hr_increment(int min, int hr, char hour[2])
{
    if (min == 59){
        if(hr == 23){
            hr = 0;
            hour[0] = '0';
            hour[1] = '0';
        }else{
            hr++;
            if(hour[1]=='9'){
                hour[1] = '0';
                hour[0]++;
            }else{
                hour[1]++;
            }
        }
    }

    return hr;
}



static void test_alpha_display() {
    int ret;
    int currenttime[] = {0,0,0,0};
    ret = alpha_oscillator();
    ret = no_blink();
    ret = set_brightness_max(0xF);

    uint16_t displaybuffer[8];

    while(1) {
        //printf("%d\n",count);
        if(count == 177) {
            int_hour = hr_increment(int_minute, int_hour, hour);
            int_minute = min_increment(int_minute, minute);
        }
        //if(int_hour == int_hralarmtime && int_minute == int_mialarmtime) {
        //    printf("The alarm is triggered!");
            //int_hralarmtime == 99;
            //int_mialarmtime == 99;
        //}
            //show numbers on display
            currenttime[0] = hour[0] - '0';
            currenttime[1] = hour[1] - '0';
            currenttime[2] = minute[0] - '0';
            currenttime[3] = minute[1] - '0';

            for (int i = 0; i <= 3; i++) {
                displaybuffer[i] = font_table(currenttime[i]);
            }

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
            vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

static void led() {
    gpio_pad_select_gpio(BLINK);
    gpio_set_direction(BLINK, GPIO_MODE_OUTPUT);

    while(1) {
        if(int_hour == int_hralarmtime && int_minute == int_mialarmtime) {
            printf("The alarm is triggered!");
            gpio_set_level(BLINK, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            gpio_set_level(BLINK, 1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            //int_hralarmtime == 99;
            //int_mialarmtime == 99;
        }
        vTaskDelay(10 / portTICK_RATE_MS);
        gpio_set_level(BLINK, 1);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) );
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    set_time();
    i2c_example_master_init();
    xTaskCreate(test_alpha_display,"test_alpha_display", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(seconds_servo_control, "seconds_servo_control", 4096, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(minutes_servo_control, "minutes_servo_control", 4096, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(led, "led", 4096, NULL, configMAX_PRIORITIES-3, NULL);

}
