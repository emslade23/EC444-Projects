//Author: Amy Dong, Quianna Mortimer, Elizabeth Slade
//Date: 12/10/2019

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
//#include "soc/mcpwm_periph.h"
#include "driver/i2c.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "esp_adc_cal.h"
#include "esp_vfs_dev.h"
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
#include "esp_attr.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "esp_types.h"


//Defines for the timer
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

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

//  Defines for alphanumeric display
#define SLAVE_ADDR_2                       0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Defines for Lidar
#define SLAVE_ADDR                         0x62  //default value of slave address 0x62
#define REG1                               0x00 //initiate ranging Register
#define VAL1                               0x04 //intiate range value
#define REG2                               0x8f //register to read for high and low
#define REG3                               0x01 //register to repeatedly read for LSB

// Defines for microlidar
#define ECHO_TEST_TXD  16//(GPIO_NUM_4) white in RX
#define ECHO_TEST_RXD  17//(GPIO_NUM_5) green in TX
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
#define BUF_SIZE (1024)

#define LEFT_LIDAR_TWO_TXD  12//(GPIO_NUM_4) white in RX
#define LEFT_LIDAR_TWO_RXD  27//(GPIO_NUM_5) green in TX
#define LEFT_LIDAR_TWO_RTS  (UART_PIN_NO_CHANGE)
#define LEFT_LIDAR_TWO_CTS  (UART_PIN_NO_CHANGE)

//reciever
#define RECIEVER_TXD  (UART_PIN_NO_CHANGE)
#define RECIEVER_RXD  34
#define RECIEVER_RTS  (UART_PIN_NO_CHANGE)
#define RECIEVER_CTS  (UART_PIN_NO_CHANGE)

// Defines for client communication
#define HOST_IP_ADDR "192.168.1.111" //ip address for communicating with the node.js server
#define PORT 3030
static const char *TAG = "example";

// Encoder defines and constants
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   1          //Multisampling
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_0;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

//  Global variables
uint32_t voltage;
double speed = 0;
int leftDistanceOne = 0;
int leftDistanceTwo = 0;
int16_t frontDistance = 0;
uint8_t *data;
//uint8_t *id;
//uint8_t *color;
int elapsedtime = 0;


// Flags
int collision = 0; // Collision Flag
int commandStop = 0; // Command Stop Flag
int dt_complete = 0; // Timer Flag
int slowdown = 0;
int manual = 0;
int redlight = 0;

//Define timer interrupt handler
void IRAM_ATTR timer_isr(void* arg)
{
    // Clear interrupt
    TIMERG0.int_clr_timers.t0 = 1;
    // Indicate timer has fired
    dt_complete = 1;
}

// Timer initializaton
static void periodic_timer_init()
{
    // Basic parameters of the timer
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = true;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    // Configure the alarm value and the interrupt on alarm.
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 0.1 * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_isr,
        NULL, ESP_INTR_FLAG_IRAM, NULL);

    //start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}


// Font table for alpha displays
uint16_t font_table(int num) {
    uint16_t fonttable[58];
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
    return fonttable[num];
}

int sock;
int len;
char* payload = "hello";
struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
// Task for remotely starting and stopping the crawler
static void udp_client_task(void *pvParameters)
{
  //char rx_buffer[128];
  char addr_str[128];
  int addr_family;
  int ip_protocol;
  char command[128];
  //char rx_buffer[128];


    while (1) {
      //configuring socket
      #ifdef CONFIG_EXAMPLE_IPV4
              struct sockaddr_in dest_addr;
              dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
              dest_addr.sin_family = AF_INET;
              dest_addr.sin_port = htons(PORT);
              addr_family = AF_INET;
              ip_protocol = IPPROTO_IP;
              inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
      #else // IPV6
              struct sockaddr_in6 dest_addr;
              bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
              dest_addr.sin6_family = AF_INET6;
              dest_addr.sin6_port = htons(PORT);
              addr_family = AF_INET6;
              ip_protocol = IPPROTO_IPV6;
              inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
      #endif

      // establishing socket
      sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
      ESP_LOGI(TAG, "Socket created");

      int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
       if (err < 0) {
           ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
       }
       ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        while (1) {

          ESP_LOGI(TAG, "Waiting for data");
           //struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
           socklen_t socklen = sizeof(source_addr);
           len = recvfrom(sock, command, sizeof(command) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

          if (len < 0) {
              ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
              break;
          } else {   // Data received
              // Get the sender's ip address as string
              if (source_addr.sin6_family == PF_INET) {
                  inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
              } else if (source_addr.sin6_family == PF_INET6) {
                  inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
              }

              command[len] = 0; // Null-terminate whatever we received and treat like a string...
              ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
              ESP_LOGI(TAG, "%s", command);


              if (strncmp(command, "auto",4) == 0){
                  printf("Autonomous driving on\n");
                  commandStop = 0;
                  slowdown = 0;
              } else if (strncmp(command, "manual",6) == 0){
                  printf("Manual driving on\n");
                  commandStop = 1;
                  slowdown = 0;
                  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
              } else {
                  printf("Waiting for valid command...\n");
              }

              if (commandStop == 1) {
                if (strncmp(command,"straight",8) == 0) {
                  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1450);
                  //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1250);
                } else if (strncmp(command,"backward",8) == 0) {
                  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1450);
                  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1500);
                } else if (strncmp(command,"left",4) == 0) {
                  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2400);
                  //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1300);
                } else if (strncmp(command,"right",5) == 0) {
                  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 800);
                  //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1300);
                } else if (strncmp(command,"stop",4) == 0) {
                  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
                } else if (strncmp(command,"speedup",6) == 0) {
                  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1300);
                }
             }


              // int err = sendto(sock, payload, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
              // if (err < 0) {
              //     ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
              //     break;
              // }
          }

               //vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
  }
}

 //Function to initiate i2c components
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


// Utility  Functions //////////////////////////////////////////////////////////

// Turn on oscillator for alpha display
int alpha_oscillator() {
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
// Set blink rate to off for display
int no_blink() {
    int ret;
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, ( SLAVE_ADDR_2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
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
    i2c_master_write_byte(cmd3, ( SLAVE_ADDR_2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
    i2c_master_stop(cmd3);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd3);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

// Function that sends information to the display and makes it display the wheel speed
static void alpha_display(char buffer[5]) {

    int currenttime[] = {0,0,0,0};
    uint16_t displaybuffer[8];

            currenttime[0] = buffer[0];
            currenttime[1] = buffer[1];
            currenttime[2] = buffer[2];
            currenttime[3] = buffer[3];

            // gets the correct mapping from the font table
        for (int i = 0; i <= 3; i++) {
            displaybuffer[i] = font_table(currenttime[i]);
        }

            // Send commands characters to display over I2C
            i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
            i2c_master_start(cmd4);
            i2c_master_write_byte(cmd4, ( SLAVE_ADDR_2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
            for (uint8_t i=0; i<8; i++) {
                i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
                i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
            }
            i2c_master_stop(cmd4);
            i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
            i2c_cmd_link_delete(cmd4);
            vTaskDelay(1000 / portTICK_RATE_MS);
}

// Write one byte to register of Lidar
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

// Read register of Lidar
uint8_t readRegister(uint8_t reg) {
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

// read 16 bits (2 bytes) of Lidar
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

// Task to continuously read lidar data
void read_lidar() {
  while (1){
    writeRegister(REG1, VAL1);
    frontDistance = read16(REG2);
    vTaskDelay(200 / portTICK_RATE_MS);
  }
}

// Task to get data from the micro lidars
static void microLidar(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200, //1200
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    //left lidar 2
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, LEFT_LIDAR_TWO_TXD, LEFT_LIDAR_TWO_RXD, LEFT_LIDAR_TWO_RTS, LEFT_LIDAR_TWO_CTS);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);


    // Configure a temporary buffer for the incoming data
    uint8_t *dataOne = (uint8_t *) malloc(11);
    uint8_t *dataTwo = (uint8_t *) malloc(11);

    while (1) {
        // Read data from the UART
        uart_read_bytes(UART_NUM_0, dataOne, 9, 10 / portTICK_RATE_MS);
        uart_read_bytes(UART_NUM_2, dataTwo, 9, 10 / portTICK_RATE_MS);

        if(dataOne[0] == 0x59 && dataOne[1] == 0x59) {
          leftDistanceOne = (dataOne[3]<<8)|(dataOne[2]);
        }
            if(dataTwo[0] == 0x59 && dataTwo[1] == 0x59) {
              leftDistanceTwo = (dataTwo[3]<<8)|(dataTwo[2]);
            }
    }
}

// Calibrating motors and testing servo on the crawler
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

    printf("Finish calibration\n");

        printf("testing steering\n");

        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2500); // HIGH signal in microseconds
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700);  // LOW signal in microseconds
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1450); // NEUTRAL signal in microsecondsA
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1450); // reset the ESC

        vTaskDelay(2000 / portTICK_PERIOD_MS);

        printf("test done\n");

}

// Task for using adc with optical encoder to detect wheel speed
static void encoder() {
  //Configure ADC
      adc1_config_width(ADC_WIDTH_BIT_10);
      adc1_config_channel_atten(channel, atten);

  //Characterize ADC
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars);

  //Continuously sample ADC1
  while (1) {
      float adc_reading = 0;
      //Multisampling
      for (int i = 0; i < NO_OF_SAMPLES; i++) {
          adc_reading += adc1_get_raw((adc1_channel_t)channel);
      }
      adc_reading /= NO_OF_SAMPLES;
      voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

      vTaskDelay(pdMS_TO_TICKS(10));
  }
}

double pulseCount = 0;

// Function for detecting pulses with the encoder
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

int difference = 0;

// Task for steering
static void straightLineError() {
  while (1) {
      if(commandStop == 0) {
        difference = leftDistanceOne - leftDistanceTwo;
        if (redlight == 0) {
          if (collision == 0) {
            if (leftDistanceOne <= 35) {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700);
            } else if (difference >= 3 && difference < 8) { //turn left a little
              //printf("diff more than 5\n");
              mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1800);
            } else if (difference <= -3 && difference > -8) { //turn right a little
              mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1200);
            } else if (difference >= 10) {
              mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2400);
            } else if (difference <= -10) {
              mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 800);
            } else {
              mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1450);
            }
          } else if (collision == 1) {
            //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1450);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
          }
        } else if (redlight == 1) {
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
      } else if(commandStop == 1) {
        vTaskDelay(pdMS_TO_TICKS(100));
      }
  }
}

int setspeed = 1265;
double output = 0;
int error = 0;
int setpoint = 0.258330;
int integral = 0;
double dt = 0.1;
double derivative = 0;
int previous_error = 0;
double Kp = 40;
double Ki = 1;
double Kd = 0.1;

// PID for controlling speed of crawler
void PID() {
    if (frontDistance < 40) {
      collision = 1;
    } else {
      collision = 0;
    }

      if(commandStop == 0) {
        if (redlight == 0) {
          if (collision == 0) {
              error = setpoint - speed;
              integral = integral + error * dt;
              derivative = (error - previous_error) / dt;
              output = Kp * error + Ki * integral + Kd * derivative;
              previous_error = error;
            if (slowdown == 0) {
              mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, setspeed);
            } else if (slowdown == 1) {
              mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, setspeed);
            }
          } else if (collision == 1) {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
          }
        } else if (redlight == 1) {
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
        }
      } else if(commandStop == 1) {
          vTaskDelay(pdMS_TO_TICKS(100));
      }
}

void setspeedcontroller() {
    if (collision == 0 && commandStop == 0 && redlight == 0) {
      if (slowdown == 0) {
        if(speed > 0.206664 && speed < 0.35) {
          setspeed++;
        } else if (speed < 0.206664 && speed > 0.03) {
          setspeed--;
        } else if (speed > 0.37) {
          setspeed = setspeed + 5;
        } else if (speed < 0.03) {
          setspeed = setspeed - 5;
        } else {
          setspeed = setspeed;
        }
      } else if (slowdown == 1){
        if(speed > 0.1 && speed < 0.21) {
          setspeed++;
        } else if (speed < 0.1 && speed > 0.03) {
          setspeed--;
        } else if (speed < 0.03) {
          setspeed = setspeed - 5;
        } else if (speed > 0.37) {
          setspeed = setspeed + 5;
        } else {
          setspeed = setspeed;
        }
      }
    }
}

int beaconid;
char color[5];
char previouscolor[5];
int previousbeaconid;

void sendsplittime() {
  asprintf(&payload,"%d,%s,%d",elapsedtime, color, beaconid);

  if(previousbeaconid != beaconid) {
    printf("message is: %s\n",payload);

    char splittime_char[50]; // = NULL;
    sprintf(splittime_char,"%d", elapsedtime);
    alpha_display(splittime_char);

    // int err = sendto(sock, payload, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
    // if (err < 0) {
    //     ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    //     //break;
    // }
  }
  strcpy(previouscolor,color);
  previousbeaconid = beaconid;
}


static void recieveLight(void *arg)
{
  data = (uint8_t *) malloc(20);


    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config_reciever = {
        .baud_rate = 1200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config_reciever);
    uart_set_pin(UART_NUM_1, RECIEVER_TXD, RECIEVER_RXD, RECIEVER_RTS, RECIEVER_CTS);
    uart_set_line_inverse(UART_NUM_1, UART_INVERSE_RXD);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    //char *msg = "data transmitted";

    while (1) {
      // sending info back to the fob
        //uart_write_bytes(UART_NUM_1, (char *) msg, 20);
        if (commandStop == 0) {
        int len = uart_read_bytes(UART_NUM_1, data, 20, 20 / portTICK_RATE_MS);

        //if (len >= 1) {
          //printf(" length: %d \n", len);
          //printf("%x %x %x %x\n", data[0], data[1], data[2], data[3]);
          for (int i = 0; i < 4; i++) {
            if(data[i] == 0x1B) {
              if (i != 3) {
                if(data[i+1] == 0x52) { //"R"
                  printf("Light is red\n");
                  redlight = 1;
                  slowdown = 0;
                  strcpy(color,"R");
                } else if (data[i+1] == 0x59) { //"Y"
                  redlight = 0;
                  slowdown = 1;
                  printf("Light is yellow\n");
                  strcpy(color,"Y");
                } else if (data[i+1] == 0x47) { //"G'"
                  redlight = 0;
                  slowdown = 0;
                  printf("Light is green\n");
                  strcpy(color,"G");
                }
                if (i == 0 || i == 1) {
                  beaconid = data[i+2];
                } else if (i == 2) {
                  beaconid = data[0];
                }
              } else if (i == 3) {
                if(data[0] == 0x52) { //"R"
                  printf("Light is red\n");
                  redlight = 1;
                  slowdown = 0;
                  strcpy(color,"R");
                } else if (data[0] == 0x59) { //"Y"
                  redlight = 0;
                  slowdown = 1;
                  printf("Light is yellow\n");
                  strcpy(color,"Y");
                } else if (data[0] == 0x47) { //"G'"
                  redlight = 0;
                  slowdown = 0;
                  printf("Light is green\n");
                  strcpy(color,"G");
                }
                beaconid = data[1];
              }
              //printf("%d\n", beaconid);
              sendsplittime();
            }
          }
          data[0] = 0;
          data[1] = 0;
          data[2] = 0;
          data[3] = 0;
          //*color = data[1];
          //*id = data[2];
          /*char color, id;
          //int id;
          sprintf(color, "%x"， data[1]);
          sprintf(id, "%x", data[2]);
          printf("Color: %c, ID: %c\n", color, id);*/

        //printf("imreading\n");
        //}
      }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
  // Initializations and task for sending commands to the crawler
      ESP_ERROR_CHECK(nvs_flash_init());
      tcpip_adapter_init();
      ESP_ERROR_CHECK(esp_event_loop_create_default());
      ESP_ERROR_CHECK(example_connect());
      xTaskCreate(udp_client_task, "udp_client", 4096, NULL, configMAX_PRIORITIES, NULL);

// Initializations for the alphanumeric display
    i2c_master_init();
    alpha_oscillator();
    no_blink();
    set_brightness_max(0xF);

// Calibrate ESC
    calibrateESC();

      xTaskCreate(read_lidar,"read_lidar", 4096, NULL, configMAX_PRIORITIES, NULL);
      //printf("1\n");
      xTaskCreate(encoder,"encoder", 4096, NULL, configMAX_PRIORITIES, NULL);
      //printf("2\n");
      xTaskCreate(pulseCounting,"pulse_counting", 4096, NULL, configMAX_PRIORITIES, NULL);
      //printf("3\n");
      xTaskCreate(microLidar, "micro_lidar", 4096, NULL, configMAX_PRIORITIES, NULL);
      //printf("4\n");
      xTaskCreate(straightLineError, "straight_Line_Error", 4096, NULL, configMAX_PRIORITIES, NULL);
      //printf("5\n");
      xTaskCreate(recieveLight, "recieve_light", 4096, NULL, configMAX_PRIORITIES, NULL);


// Initialization for timer
    periodic_timer_init();

    int count = 0;
      while (1) {

          // Run PID every 100 ms (when timer is triggered)
          if (dt_complete == 1) {
              count++;
              if (count == 10) {
                  setspeedcontroller();
                  count = 0;
                  elapsedtime++;
                  printf("%d\n",elapsedtime);
                  speed = pulseCount * 0.051666;

                          printf("Speed = %f m/s\n", speed);
                          printf("left One Distance = %d cm\n", leftDistanceOne);
                          printf("Left Two Distance = %d cm\n", leftDistanceTwo);
                          printf("set speed = %d \n", setspeed);
                          printf("Front Distance: %d cm\n", frontDistance);
                          printf("collision = %d, slowdown = %d, commandstop = %d\n", collision, slowdown, commandStop);
                            pulseCount = 0;
              }

            PID();
            dt_complete = 0;
            // Re-enable alarm
            TIMERG0.hw_timer[TIMER_0].config.alarm_en = 1;
              //printf("working timer: %d\n", count);
           }
          vTaskDelay(pdMS_TO_TICKS(50));
      }
}
