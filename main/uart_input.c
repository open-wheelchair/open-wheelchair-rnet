#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "rom/crc.h"
#include "esp_timer.h"

#include "cobs.h"
#include "pal.h"

static const char* TAG = "uart_input";


#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart2_queue;


uint8_t msg_buf_encoded[256];
size_t msg_buf_encoded_idx = 0;

uint8_t msg_buf_decoded[256];

#define COMM_TYPE_JOYSTICK (1)

static void handle_msg(const uint8_t* buffer, const size_t len) {
    if (len < 1) {
        return;
    }
    switch (buffer[0]) {
        case COMM_TYPE_JOYSTICK: {
            if (len < 5) {
                return;
            }
            pal_serial_input_t pal_input = {
                .right = buffer[1],
                .fwd = buffer[2],
                .is_seating_mode = buffer[3] != 0,
                .seating_axis = buffer[4],

                .timestamp_us = esp_timer_get_time(),
            };
            pal_push_serial(&pal_input);

            break;
        }
        default: {
            ESP_LOGE(TAG, "unknown comms packet");
        }
    }
}


static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart2_queue, (void *)&event, (TickType_t)1000)) {
            bzero(dtmp, RD_BUF_SIZE);
            // ESP_LOGI(TAG, "uart[%d] event:", UART_NUM_2);
            switch (event.type) {
            //Event of UART receiving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                int len = uart_read_bytes(UART_NUM_2, dtmp, event.size, portMAX_DELAY);
                for (int i = 0; i < len; i++) {
                    msg_buf_encoded[msg_buf_encoded_idx] = dtmp[i];
                    msg_buf_encoded_idx++;
                    if (dtmp[i] == 0) {
                        size_t decoded_len = 0;
                        cobs_ret_t const result = cobs_decode(msg_buf_encoded, msg_buf_encoded_idx, msg_buf_decoded, sizeof(msg_buf_decoded), &decoded_len);
                        if (result == COBS_RET_SUCCESS) {
                            uint32_t crc_result = crc32_le(0, msg_buf_decoded, decoded_len);
                            if (crc_result != 0xffffffff) {
                                ESP_LOGE(TAG, "CRC mismatch!");
                            } else if (decoded_len >= 4) {
                                handle_msg(msg_buf_decoded, decoded_len - 4);
                            }
                        } else {
                            // decoding failed, look to 'result' for details.
                            ESP_LOGE(TAG, "failed to decode COBS");
                        }
                        msg_buf_encoded_idx = 0;
                    }
                    if (msg_buf_encoded_idx >= sizeof(msg_buf_encoded)) {
                        msg_buf_encoded_idx = 0;
                        ESP_LOGE(TAG, "UART COBS overflow");
                    }

                }

                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGE(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_2);
                xQueueReset(uart2_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGE(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_2);
                xQueueReset(uart2_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        } else {
            char* message = "hello there!\n";
            uart_write_bytes(UART_NUM_2, message, strlen(message));
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void setup_uart() {
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config0 = {
        .baud_rate = 922190,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config0);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_disable_pattern_det_intr(UART_NUM_0);

    // UART 2
    uart_config_t uart_config2 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart2_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config2));

    //Set UART pins (using UART0 default pins ie no changes.)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uart_disable_pattern_det_intr(UART_NUM_2);

    xTaskCreate(uart_event_task, "uart_event_task", 3072, NULL, 12, NULL);
}

