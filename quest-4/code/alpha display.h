#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_vfs_dev.h"

// 14-Segment Display
#define SLAVE_ADDR_1                        0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_buffer_DISABLE  0    // i2c master no bufferfer needed
#define I2C_EXAMPLE_MASTER_RX_buffer_DISABLE  0    // i2c master no bufferfer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// Function to initiate i2c -- note the MSB declaration!
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
                       I2C_EXAMPLE_MASTER_RX_buffer_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_buffer_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK) {printf("- initialized: yes\n\n");}

    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
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
}

// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
        printf("- No I2C devices found!" "\n");
    printf("\n");
}

////////////////////////////////////////////////////////////////////////////////

// Alphanumeric Functions //////////////////////////////////////////////////////

// Turn on oscillator for alpha display
int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR_1 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
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
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR_1 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
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
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR_1 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

////////////////////////////////////////////////////////////////////////////////

//char *mapping(char* buffer) {
//    if(buffer[0] == 't') {
//        char *map = "0b0000110000111111";
//    }
//return map;
//}


static void test_alpha_display(char buffer[5]) {
    // Debug
    //char buffer[5];
    uint16_t fonttable[40];
    uint16_t map[5];
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
    fonttable[10] = 0b0000000000000000; //
    fonttable[11] = 0b0000000011110111; // A
    fonttable[12] = 0b0001001010001111; // B
    fonttable[13] = 0b0000000000111001; // C
    fonttable[14] = 0b0001001000001111; // D
    fonttable[15] = 0b0000000011111001; // E
    fonttable[16] = 0b0000000001110001; // F
    fonttable[17] = 0b0000000010111101; // G
    fonttable[18] = 0b0000000011110110; // H
    fonttable[19] = 0b0001001000000000; // I
    fonttable[20] = 0b0000000000011110; // J
    fonttable[21] = 0b0010010001110000; // K
    fonttable[22] = 0b0000000000111000; // L
    fonttable[23] = 0b0000010100110110; // M
    fonttable[24] = 0b0010000100110110; // N
    fonttable[25] = 0b0000000000111111; // O
    fonttable[26] = 0b0000000011110011; // P
    fonttable[27] = 0b0010000000111111; // Q
    fonttable[28] = 0b0010000011110011; // R
    fonttable[29] = 0b0000000011101101; // S
    fonttable[30] = 0b0001001000000001; // T
    fonttable[31] = 0b0000000000111110; // U
    fonttable[32] = 0b0000110000110000; // V
    fonttable[33] = 0b0010100000110110; // W
    fonttable[34] = 0b0010110100000000; // X
    fonttable[35] = 0b0001010100000000; // Y
    fonttable[36] = 0b0000110000001001; // Z

    while (1) {
      /*  printf(">> Enter a string of 4 to display: \n");
    gets(buffer);
    printf("%s\n",buffer);*/

    char num;
    //char letter = 'a';

int i ;
for (i = 0; buffer[i]; i++) {
	//check numbers
	for (num ='0'; num>='9'; num++){
        	if(buffer[i] == num) {
            		map[i] = fonttable[i];
        	}
	}
}

      //}  //check letters
        if(buffer[i] == ' ') {
            map[i] = fonttable[10];
        }
        if(buffer[i] == 'a') {
            map[i] = fonttable[11];
        }
        if(buffer[i] == 'b') {
            map[i] = fonttable[12];
        }
        if(buffer[i] == 'c') {
            map[i] = fonttable[13];
        }
        if(buffer[i] == 'd') {
            map[i] = fonttable[14];
        }
        if(buffer[i] == 'e') {
            map[i] = fonttable[15];
        }
        if(buffer[i] == 'f') {
            map[i] = fonttable[16];
        }
        if(buffer[i] == 'g') {
            map[i] = fonttable[17];
        }
        if(buffer[i] == 'h') {
            map[i] = fonttable[18];
        }
        if(buffer[i] == 'i') {
            map[i] = fonttable[19];
        }
        if(buffer[i] == 'j') {
            map[i] = fonttable[20];
        }
        if(buffer[i] == 'k') {
            map[i] = fonttable[21];
        }
        if(buffer[i] == 'l') {
            map[i] = fonttable[22];
        }
        if(buffer[i] == 'm') {
            map[i] = fonttable[23];
        }
        if(buffer[i] == 'n') {
            map[i] = fonttable[24];
        }
        if(buffer[i] == 'o') {
            map[i] = fonttable[25];
        }
        if(buffer[i] == 'p') {
            map[i] = fonttable[26];
        }
        if(buffer[i] == 'q') {
            map[i] = fonttable[27];
        }
        if(buffer[i] == 'r') {
            map[i] = fonttable[28];
        }
        if(buffer[i] == 's') {
            map[i] = fonttable[29];
        }
        if(buffer[i] == 't') {
            map[i] = fonttable[30];
        }
        if(buffer[i] == 'u') {
            map[i] = fonttable[31];
        }
        if(buffer[i] == 'v') {
            map[i] = fonttable[32];
        }
        if(buffer[i] == 'w') {
            map[i] = fonttable[33];
        }
        if(buffer[i] == 'x') {
            map[i] = fonttable[34];
        }
        if(buffer[i] == 'y') {
            map[i] = fonttable[35];
        }
        if(buffer[i] == 'z') {
            map[i] = fonttable[36];
        }
    //}


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

    // Write to characters to bufferfer
    uint16_t displaybufferfer[8];

    displaybufferfer[0] = map[0];
    displaybufferfer[1] = map[1];
    displaybufferfer[2] = map[2];
    displaybufferfer[3] = map[3];

    // Continually writes the same command
    //while (1) {

      // Send commands characters to display over I2C
      i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
      i2c_master_start(cmd4);
      i2c_master_write_byte(cmd4, ( SLAVE_ADDR_1 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
      i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
      for (uint8_t i=0; i<8; i++) {
        i2c_master_write_byte(cmd4, displaybufferfer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displaybufferfer[i] >> 8, ACK_CHECK_EN);
      }
      i2c_master_stop(cmd4);
      ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
      i2c_cmd_link_delete(cmd4);

      if(ret == ESP_OK) {
          printf("NO\n");
      }
    }


}

void alpha_display(char buffer [5])
{
  ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) );
  esp_vfs_dev_uart_use_driver(UART_NUM_0);

  i2c_example_master_init();
  i2c_scanner();

  test_alpha_display(buffer);
}

/*void app_main() {
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) );
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    i2c_example_master_init();
    i2c_scanner();

    xTaskCreate(test_alpha_display,"test_alpha_display", 4096, NULL, 5, NULL);


}*/
