//CODE FOR KEYFOB 1

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "esp_log.h"
#include <stdlib.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
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

 #define RMT_TX_CHANNEL RMT_CHANNEL_0
 #define RMT_TX_GPIO 26

 #define ECHO_TEST_TXD  25
 #define ECHO_TEST_RXD  34
 #define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
 #define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

 #define BUF_SIZE (1024)//1024

 //define for button
 #define BUTTON 4

 // #define BLINK_RED 33
  #define BLINK_GREEN 32
 // #define BLINK_BLUE 14

 #define HOST_IP_ADDR "192.168.1.131" //ip address for communicating with the node.js server
 #define PORT 3030

 static const char *TAG = "example";

 int push_button_state = 0;
 int flag = 0;

//static void udp_client_task(void *pvParameters)
static void udp_client_task()
 {
     //char rx_buffer[128];
     char addr_str[128];
     int addr_family;
     int ip_protocol;
     char command[128];
     char* payload = "192.168.1.111";

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
         //break;
     }
     ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
     int index = 0;

    // while (1) {

           //printf("index： %d\n", index);

          // index = index + 1;

           if (push_button_state == 1) {
             // printf("hi!!\n");

             int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
             if (err < 0) {
                 ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                  //break;
             }

             //Recieveing data from node.js
             struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
             socklen_t socklen = sizeof(source_addr);
             int len = recvfrom(sock, command, sizeof(command) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

               // Error occurred during receiving
               if (len < 0) {
                   ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                   //break;
               } else {
                   command[len] = 0;

                   // Recieved command handling
                   if (strcmp(command, "yes") == 0){
                       printf("Access granted\n");
                       gpio_set_level(BLINK_GREEN, 1);
                       vTaskDelay(2000/portTICK_PERIOD_MS);
                       gpio_set_level(BLINK_GREEN, 0);
                   } else if (strcmp(command, "no") == 0){
                       printf("Access denied\n");
                   } else {
                       printf("Waiting for valid command...\n");
                   }
               }
           }
           vTaskDelay(100 / portTICK_PERIOD_MS);
      // }

         // if (sock != -1) {
         //     ESP_LOGE(TAG, "Shutting down socket and restarting...");
         //     shutdown(sock, 0);
         //     close(sock);
         // }
     //}
 }

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
        if(flag == 1) {
          push_button_state = 1;
          flag = 0;
        } else {
          push_button_state = 0;
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
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

    char *msg = "1,smart004";

    while (1) {
      if (push_button_state == 1) {
        uart_write_bytes(UART_NUM_1, (char *) msg, 10);

          // Read data from the UART
        //int len = uart_read_bytes(UART_NUM_1, data, 20, 20 / portTICK_RATE_MS);
        //printf("length: %d \n", len);
        //printf("data: %s\n", data);

      }
      //printf("%s\n", msg);

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
  gpio_pad_select_gpio(BLINK_GREEN);
  gpio_set_direction(BLINK_GREEN, GPIO_MODE_OUTPUT);
  //
  ESP_ERROR_CHECK(nvs_flash_init());
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());
  //xTaskCreate(udp_client_task, "udp_client", 4096, NULL, configMAX_PRIORITIES, NULL);

    rmt_tx_int();
    //button();
    xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
    xTaskCreate(button,"button", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    //printf("initialized\n");

    while (1) {
      if (push_button_state == 1) {
        udp_client_task();
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
