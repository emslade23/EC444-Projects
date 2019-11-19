

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "esp_log.h"

 #define RMT_TX_CHANNEL RMT_CHANNEL_0
 #define RMT_TX_GPIO 26

 #define ECHO_TEST_TXD  25
 #define ECHO_TEST_RXD  34
 #define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
 #define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

 #define BUF_SIZE (1024)//1024

 //define for button
 #define BUTTON 4

 #define BLINK_RED 33
 #define BLINK_GREEN 32
 #define BLINK_BLUE 14

 int push_button_state = 0;
 int flag = 0;

 static void IRAM_ATTR button_isr_handler(void* arg) {
     flag ^= 1;
 }

 static void button() {
     gpio_pad_select_gpio(BUTTON);
     gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
     gpio_set_intr_type(BUTTON, GPIO_INTR_POSEDGE);
     gpio_intr_enable(BUTTON);
     gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
     gpio_isr_handler_add(BUTTON, button_isr_handler,(void*) BUTTON);

     while(1) {
        if(flag == 1 && push_button_state != 2) {
          push_button_state++;
        } else if (flag == 1 && push_button_state == 2) {
          push_button_state = 0;
        }
        flag = 0;
         vTaskDelay(500/portTICK_PERIOD_MS);
     }
 }


  //Convert uint8_t type of data to rmt format data.
  static void IRAM_ATTR u8_to_rmt(const void* src, rmt_item32_t* dest, size_t src_size,
                           size_t wanted_num, size_t* translated_size, size_t* item_num)
  {
      if(src == NULL || dest == NULL) {
          *translated_size = 0;
          *item_num = 0;
          return;
      }
      const rmt_item32_t bit0 = {{{ 32767, 1, 15000, 0 }}}; //Logical 0
      const rmt_item32_t bit1 = {{{ 32767, 1, 32767, 0 }}}; //Logical 1
      size_t size = 0;
      size_t num = 0;
      uint8_t *psrc = (uint8_t *)src;
      rmt_item32_t* pdest = dest;
      while (size < src_size && num < wanted_num) {
          for(int i = 0; i < 8; i++) {
              if(*psrc & (0x1 << i)) {
                  pdest->val =  bit1.val;
              } else {
                  pdest->val =  bit0.val;
              }
              num++;
              pdest++;
          }
          size++;
          psrc++;
      }
      *translated_size = size;
      *item_num = num;
  }

 static void rmt_tx_int(void)
 {
     rmt_config_t config;
     config.rmt_mode = RMT_MODE_TX;
     config.channel = RMT_TX_CHANNEL;
     config.gpio_num = RMT_TX_GPIO;
     config.mem_block_num = 1;
     config.tx_config.loop_en = 0;
     // enable the carrier to be able to hear the Morse sound
     // if the RMT_TX_GPIO is connected to a speaker
     config.tx_config.carrier_en = 1;
     config.tx_config.idle_output_en = 1;
     config.tx_config.idle_level = 1;
     config.tx_config.carrier_duty_percent = 50;

     config.tx_config.carrier_freq_hz = 38000;
     config.tx_config.carrier_level = 1;
     // set the maximum clock divider to be able to output
     // RMT pulses in range of about one hundred milliseconds
     config.clk_div = 255;

     ESP_ERROR_CHECK(rmt_config(&config));
     ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
     ESP_ERROR_CHECK(rmt_translator_init(config.channel, u8_to_rmt));
 }

uint8_t *data;
//uint8_t *data;

static void echo_task(void *arg)
{
  data = (uint8_t *) malloc(20);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 2400,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_set_line_inverse(UART_NUM_1, UART_INVERSE_RXD);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data

    char *msg = "0";

    while (1) {
      if (push_button_state == 0) {
        msg = "0";
      } else if (push_button_state == 1) {
        msg = "1";
      } else if (push_button_state == 2) {
        msg = "2";
      }
      //printf("%s\n", msg);

      //uart_write_bytes(UART_NUM_1, (char *) msg, 1);

        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1, data, 20, 20 / portTICK_RATE_MS);
        printf(" length: %d \n", len);
        printf("%c \n", data[0]);
       //  printf("%c ", data[1]);
       //  printf("%c ", data[2]);
       //  printf("%c ", data[3]);
       //  printf("%c ", data[4]);
       // printf("%c ", data[5]);
       // printf("%c ", data[6]);
       // printf("%c ", data[7]);
       // printf("%c \n", data[8]);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
  gpio_pad_select_gpio(BLINK_BLUE);
  gpio_set_direction(BLINK_BLUE, GPIO_MODE_OUTPUT);

  gpio_pad_select_gpio(BLINK_GREEN);
  gpio_set_direction(BLINK_GREEN, GPIO_MODE_OUTPUT);

  gpio_pad_select_gpio(BLINK_RED);
  gpio_set_direction(BLINK_RED, GPIO_MODE_OUTPUT);

    rmt_tx_int();
    xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
    xTaskCreate(button,"button", 1024*2, NULL, configMAX_PRIORITIES, NULL);

    while (1) {
      if (data[0] == '0') {
        gpio_set_level(BLINK_RED, 1);
        gpio_set_level(BLINK_GREEN, 0);
        gpio_set_level(BLINK_BLUE, 0);
      } else if (data[0] == '1') {
        gpio_set_level(BLINK_GREEN, 1);
        gpio_set_level(BLINK_BLUE, 0);
        gpio_set_level(BLINK_RED, 0);
      } else if (data[0] == '2') {
        gpio_set_level(BLINK_BLUE, 1);
        gpio_set_level(BLINK_RED, 0);
        gpio_set_level(BLINK_GREEN, 0);
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
