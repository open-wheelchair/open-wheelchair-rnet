#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "can.h"
#include "interface.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_task_wdt.h"

#include "rom/ets_sys.h"

#include "can.h"
#include "interface.h"
#include "wireless.h"
#include "pal.h"

// TX pin on the CAN board
#define PIN_CAN_TX (26)
// RX pin on the CAN board
#define PIN_CAN_RX (27)

volatile bool has_seen_ack = false;

TaskHandle_t task_handle_node_sw = NULL;
TaskHandle_t task_handle_node_hw = NULL;
TaskHandle_t logging_task_handle = NULL;

#define ESPNOW_MAXDELAY 512

QueueHandle_t logging_queue = NULL;
typedef struct {
    uint64_t timestamp_us;
    can_frame_t frame;
    uint8_t receiver;
    uint8_t slot;
} can_frame_log_entry_t;


static const char *TAG = "joyride";

void log_can_frame(uint8_t receiver, uint8_t slot, can_frame_t frame) {
    can_frame_log_entry_t entry = {
        .receiver = receiver,
        .slot = slot,
        .frame = frame,
        .timestamp_us = esp_timer_get_time()
    };
    bool sent = xQueueSendToBack(logging_queue, &entry, 0);
    if (!sent) {
        ets_printf("log queue fail\n");
    }
}
static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    can_hardware_ctx_t* ctx = (can_hardware_ctx_t*) user_ctx;
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };
    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
        can_frame_t frame = {0};
        frame.can_dlc = rx_frame.header.dlc;
        memcpy(frame.data, rx_frame.buffer, frame.can_dlc);
        frame.can_id = rx_frame.header.id;
        if (rx_frame.header.ide) {
            frame.can_id |= CAN_EFF_FLAG;
        }
        if (rx_frame.header.rtr) {
            frame.can_id |= CAN_RTR_FLAG;
        }
        log_can_frame(ctx->id, 0, frame);
        eventqueue_item_t item = {
            .kind = EVKIND_FRAME,
            .d.frame = frame
        };
        bool success = xQueueSendToBack(ctx->opposing_event_queue, &item, 0);
        if (!success) {
            ets_printf("rx overflow!\n");
            abort();
        } else {
            xSemaphoreGive(ctx->opposing_semaphore);
        }
    }
    return false;
}

static bool twai_tx_done_cb(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
    can_hardware_ctx_t* ctx = (can_hardware_ctx_t*) user_ctx;
    if (edata->is_tx_success) {
        free(edata->done_tx_frame->buffer);
        free((void*) edata->done_tx_frame);
        if (!has_seen_ack) {
            has_seen_ack = true;
            eventqueue_item_t item = {.kind = EVKIND_SIGNAL_FIRST_ACK};
            bool success = xQueueSendToBack(ctx->opposing_event_queue, &item, 0);
            if (!success) {
                ets_printf("rx ACK overflow!\n");
                abort();
            } else {
                xSemaphoreGive(ctx->opposing_semaphore);
            }
        }
    } else {
        ets_printf("retransmit!\n");
        twai_node_transmit(handle, edata->done_tx_frame, 0);
    }
    return false;
}

void can_hardware_task(void *pvParam) {
    can_hardware_ctx_t* ctx = (can_hardware_ctx_t*) pvParam;

    twai_node_handle_t node_hdl = NULL;
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = PIN_CAN_TX,             // TWAI TX GPIO pin
        .io_cfg.rx = PIN_CAN_RX,             // TWAI RX GPIO pin
        .bit_timing.bitrate = 125000,  // 125 kbps bitrate
        .tx_queue_depth = 16,        // Transmit queue depth set to 5
        .fail_retry_cnt = 15 // retransmit 15 times in hardware, then fail
    };
    // Create a new TWAI controller driver instance
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));
    twai_event_callbacks_t user_cbs = {
        .on_rx_done = twai_rx_cb,
        .on_tx_done = twai_tx_done_cb,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl, &user_cbs, ctx));

    // Start the TWAI controller
    ESP_ERROR_CHECK(twai_node_enable(node_hdl));

    while (true) {
        // wait for ISR (or for 10 second to have passed, just to be sure)
        xSemaphoreTake(ctx->incoming_semaphore, pdMS_TO_TICKS(10*1000));

        eventqueue_item_t item;
        while (xQueueReceive(ctx->incoming_event_queue, &item, 0)) {
            if (item.kind == EVKIND_FRAME) {
                log_can_frame(20, 0, item.d.frame);
                // contains a frame, so queue it
                uint8_t* send_buffer = malloc(8);
                memcpy(send_buffer, item.d.frame.data, item.d.frame.can_dlc);

                twai_frame_t* tx_msg = calloc(1, sizeof(twai_frame_t));
                tx_msg->header.id = item.d.frame.can_id & CAN_EFF_MASK;
                tx_msg->header.ide = (item.d.frame.can_id & CAN_EFF_FLAG) ? 1 : 0; // extended
                tx_msg->header.rtr = (item.d.frame.can_id & CAN_RTR_FLAG) ? 1 : 0;
                tx_msg->buffer = send_buffer;        // Pointer to data to transmit
                tx_msg->buffer_len = item.d.frame.can_dlc;  // Length of data to transmit
                ESP_ERROR_CHECK(twai_node_transmit(node_hdl, tx_msg, 0));  // Timeout = 0: returns immediately if queue is full
            }
        }
    }
}

void logging_task(void* pvParam) {
    while (true) {
        can_frame_log_entry_t entry = {0};
        if (!xQueueReceive(logging_queue, &entry, portMAX_DELAY)) {
            continue;
        }
        if (entry.receiver == 2) {
            printf("RX ");
        } else if (entry.receiver == 20) {
            printf("TX ");
        } else {
            printf("UNK_%d ", entry.receiver);
        }

        if (entry.frame.can_id & CAN_ERR_FLAG) {
            printf("ERROR ");
        }
        if (entry.frame.can_id & CAN_RTR_FLAG) {
            printf("RTR ");
        }

        if (entry.frame.can_id & CAN_EFF_FLAG) {
            printf("FRAME %lld %d: %08lx", entry.timestamp_us, entry.slot, entry.frame.can_id & CAN_EFF_MASK);
        } else {
            printf("FRAME %lld %d: %03lx", entry.timestamp_us, entry.slot, entry.frame.can_id & CAN_SFF_MASK);
        }

        for (int i = 0; i < entry.frame.can_dlc; i++) {
            printf(" %02x", entry.frame.data[i]);
        }
        printf(" END\n");
    }
}

extern void setup_uart();

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    pal_init();
    setup_uart();

    vTaskDelay(pdMS_TO_TICKS(1000));

    logging_queue = xQueueCreate(32, sizeof(can_frame_log_entry_t));

    QueueHandle_t tx_queue_1 = xQueueCreate(16, sizeof(eventqueue_item_t));
    QueueHandle_t tx_queue_2 = xQueueCreate(16, sizeof(eventqueue_item_t));
    SemaphoreHandle_t semaphore_1 = xQueueCreateCountingSemaphore(1, 0);
    SemaphoreHandle_t semaphore_2 = xQueueCreateCountingSemaphore(1, 0);

    can_hardware_ctx_t ctx_1 = {
        .id = 1,
        .hw = (void*) (0xDEADBEEF),
        .incoming_event_queue = tx_queue_1,
        .opposing_event_queue = tx_queue_2,
        .incoming_semaphore = semaphore_1,
        .opposing_semaphore = semaphore_2,
    };
    can_hardware_ctx_t ctx_2 = {
        .id = 2,
        .hw = NULL,
        .incoming_event_queue = tx_queue_2,
        .opposing_event_queue = tx_queue_1,
        .incoming_semaphore = semaphore_2,
        .opposing_semaphore = semaphore_1,
    };

    xTaskCreatePinnedToCore(
        logging_task,         // Task function
        "logger",       // Task name
        15000,             // Stack size (bytes)
        NULL,              // Parameters
        1,                 // Priority
        &logging_task_handle,  // Task handle
        0                  // Core 0
    );

    xTaskCreatePinnedToCore(
        can_hardware_task,         // Task function
        "can_hw",       // Task name
        15000,             // Stack size (bytes)
        &ctx_2,              // Parameters
        configMAX_PRIORITIES  - 2,                 // Priority
        &task_handle_node_hw,  // Task handle
        0                  // Core 0
    );

    xTaskCreatePinnedToCore(
        comms_task, // Task function
        "comms",       // Task name
        15000,             // Stack size (bytes)
        &ctx_1,              // Parameters
        9,                 // Priority
        &task_handle_node_sw,  // Task handle
        0                  // Core 0
    );

    ESP_LOGI(TAG, "Ready for capture");
    vTaskDelay(pdMS_TO_TICKS(5000));

    wifi_init();
    espnow_init();

    // prevent local variables going out of scope
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000*100));
    }
}
