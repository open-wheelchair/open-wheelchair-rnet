#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "mcp2515.h"
#include "can.h"
#include "interface.h"
#include "wireless.h"
#include "pal.h"

volatile bool has_seen_ack = false;

#define PIN_1_MCP2515_INT  (13)
#define PIN_1_MCP2515_CLK  (12)
#define PIN_1_MCP2515_MOSI (11)
#define PIN_1_MCP2515_MISO (10)
#define PIN_1_MCP2515_CS    (9)
#define MCP2515_1_SPI_INSTANCE (SPI2_HOST)

#define PIN_2_MCP2515_INT  (15)
#define PIN_2_MCP2515_CLK  (7)
#define PIN_2_MCP2515_MOSI (6)
#define PIN_2_MCP2515_MISO (5)
#define PIN_2_MCP2515_CS    (4)
#define MCP2515_2_SPI_INSTANCE (SPI3_HOST)


TaskHandle_t mcp2515_message_task_1 = NULL;
TaskHandle_t mcp2515_message_task_2 = NULL;
TaskHandle_t logging_task_handle = NULL;

#define ESPNOW_MAXDELAY 512

QueueHandle_t logging_queue = NULL;
typedef struct {
    uint64_t timestamp_us;
    can_frame_t frame;
    uint8_t receiver;
    uint8_t slot;
} can_frame_log_entry_t;


static const char *TAG = "robob";

void log_can_frame(uint8_t receiver, uint8_t slot, can_frame_t frame) {
    can_frame_log_entry_t entry = {
        .receiver = receiver,
        .slot = slot,
        .frame = frame,
        .timestamp_us = esp_timer_get_time()
    };
    bool sent = xQueueSendToBack(logging_queue, &entry, 0);
    if (!sent) {
        ESP_LOGE(TAG, "unable to queue log item");
    }
}

// Handles interrupts, returns whether this function should be called again shortly
static bool handle_interrupts(mcp2515_ctx_t* ctx) {
    uint8_t interrupts = MCP2515_getInterrupts(ctx->mcp_hardware_instance);
    if (!interrupts) {
        return false;
    }
    can_frame_t frame;
    if (interrupts & CANINTF_RX0IF) {
        // ESP_LOGI(TAG, "RX0 interrupt");
        if (MCP2515_readMessage(ctx->mcp_hardware_instance, RXB0, &frame) == ERROR_OK) {
            log_can_frame(ctx->id, 0, frame);
            eventqueue_item_t item = {
                .kind = EVKIND_FRAME,
                .d.frame = frame
            };
            bool success = xQueueSendToBack(ctx->opposing_event_queue, &item, 0);
            if (!success) {
                ESP_LOGE(TAG, "RX queue overflow %d", ctx->id);
            } else {
                xSemaphoreGive(ctx->opposing_semaphore);
            }
        }
    }
    if (interrupts & CANINTF_RX1IF) {
        // ESP_LOGI(TAG, "RX1 interrupt");
        if (MCP2515_readMessage(ctx->mcp_hardware_instance, RXB1, &frame) == ERROR_OK) {
            log_can_frame(ctx->id, 1, frame);
            eventqueue_item_t item = {
                .kind = EVKIND_FRAME,
                .d.frame = frame
            };
            bool success = xQueueSendToBack(ctx->opposing_event_queue, &item, 0);
            if (!success) {
                ESP_LOGE(TAG, "RX queue overflow %d", ctx->id);
            } else {
                xSemaphoreGive(ctx->opposing_semaphore);
            }
        }
    }
    if (interrupts & (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF)) {
        if (interrupts & CANINTF_TX0IF) {
            uint8_t reg = MCP2515_readRegister(ctx->mcp_hardware_instance, MCP_TXB0CTRL);
            // ESP_LOGI(TAG, "%d TX=%02x", ctx->id, reg);
            if ((reg & TXB_TXERR) == 0 && !has_seen_ack) {
                has_seen_ack = true;
                xSemaphoreGive(ctx->opposing_semaphore);
            }
        }
        if (interrupts & (CANINTF_TX1IF | CANINTF_TX2IF)) {
            ESP_LOGE(TAG, "TX INT on unhandled interface");
        }
    }


    if (interrupts & CANINTF_ERRIF) {
        uint8_t errors = MCP2515_getErrorFlags(ctx->mcp_hardware_instance);
        MCP2515_modifyRegister(ctx->mcp_hardware_instance, MCP_EFLG, errors, 0);
        ESP_LOGE(TAG, "CAN ERRIF %d err=%02x", ctx->id, errors);
        if (errors & (EFLG_RX0OVR | EFLG_RX1OVR)) {
            ESP_LOGE(TAG, "RX OVERFLOW %d", ctx->id);
        }
    }
    if (interrupts & CANINTF_MERRF) {
        uint8_t reg = MCP2515_readRegister(ctx->mcp_hardware_instance, MCP_TXB0CTRL);
        bool logged = false;
        if (reg & TXB_ABTF) {
            ESP_LOGE(TAG, "MERR ABTF %d", ctx->id);
            logged = true;
        }
        if (reg & TXB_MLOA) {
            ESP_LOGE(TAG, "MERR MLOA %d", ctx->id);
            logged = true;
        }
        if (reg & TXB_TXERR) {
            //ESP_LOGE(TAG, "MERR TXERR %d", ctx->id);
            logged = true;
        }
        if (!logged) {
            ESP_LOGE(TAG, "unknown MERR %d txb=%02x", ctx->id, reg);
        }

    }
    // clear interrupts
    MCP2515_modifyRegister(ctx->mcp_hardware_instance, MCP_CANINTF, interrupts, 0);

    return true;
}

// Handles transmission, returns whether this function should be called again shortly
bool handle_transmit(mcp2515_ctx_t* ctx) {
    // TODO should we call FromISR methods from a regular task?
    if (0 == uxQueueMessagesWaiting(ctx->incoming_event_queue)) {
        return false;
    }

    eventqueue_item_t item;

    assert(xQueuePeek(ctx->incoming_event_queue, &item, 0));
    // this queue should only get frames, no other kind of events
    assert(item.kind == EVKIND_FRAME);

    // There are messages waiting in the queue, check if we have space for them
    uint8_t ctrlval = MCP2515_readRegister(ctx->mcp_hardware_instance, MCP_TXB0CTRL);
    if ((ctrlval & TXB_TXREQ) != 0) {
        // instead we just drop the frames for now
        // ESP_LOGW(TAG, "still transmitting, waiting for slot to become free");
        return true;
    }
    log_can_frame(20, 0, item.d.frame);
    ERROR_t err = MCP2515_sendMessage(ctx->mcp_hardware_instance, TXB0, &(item.d.frame));
    if (err == ERROR_OK) {
        // ESP_LOGI(TAG, "TX slot now contains %03lx", item.d.frame.can_id);
        assert(xQueueReceive(ctx->incoming_event_queue, &item, 0));
        return true;
    } else {
        xQueueReceive(ctx->incoming_event_queue, &item, 0);
        ESP_LOGE(TAG, "sendMessage %d err %d", ctx->id, err);
        return true;
    }
}

void mcp2515_handle_interrupt_task(void *pvParam) {
    mcp2515_ctx_t* ctx = (mcp2515_ctx_t*) pvParam;
    assert(ctx->mcp_hardware_instance != NULL);

    bool initialized_fully = false;
    while (true) {
        // wait for ISR
        xSemaphoreTake(ctx->incoming_semaphore, pdMS_TO_TICKS(40));
        bool interrupt_has_work, tx_has_work;
        int serviced = 0;
        do {
            int64_t begin = esp_timer_get_time();
            interrupt_has_work = handle_interrupts(ctx);
            tx_has_work = handle_transmit(ctx);
            int64_t end = esp_timer_get_time();
            serviced++;
            if (serviced > 10) {
                ESP_LOGE(TAG, "serviced %d times without returning", serviced);
            }
            if (end - begin > 50*1000 /*50 ms*/) {
                ESP_LOGE(TAG, "servicing took %lld us", end - begin);
            }
        } while (interrupt_has_work || tx_has_work);

        if (has_seen_ack && !initialized_fully) {
            initialized_fully = true;
            ESP_LOGW(TAG, "first succesful transmission with ack on %d", ctx->id);
            ESP_LOGI(TAG, "adding watchdog timer");
            ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
            ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
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

static void IRAM_ATTR mcp2515_isr(void *arg) {
    mcp2515_ctx_t* ctx = (mcp2515_ctx_t*) arg;

    int higher_priority_task_woken = 0;
    xSemaphoreGive(ctx->incoming_semaphore);
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

bool SPI_Init(MCP2515 MCP2515_Object, spi_host_device_t spi, int miso, int mosi, int clk, int cs)
{
	esp_err_t ret;
	//Configuration for the SPI bus
	spi_bus_config_t bus_cfg={
		.miso_io_num=miso,
		.mosi_io_num=mosi,
		.sclk_io_num=clk,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz = 0 // no limit
	};

	// Define MCP2515 SPI device configuration
	spi_device_interface_config_t dev_cfg = {
		.mode = 0, // (0,0)
		.clock_speed_hz = 10000000, // 10 MHz max SPI clock speed on MCP2515
		.spics_io_num = cs,
		.queue_size = 128
	};

	// Initialize SPI bus
	ret = spi_bus_initialize(spi, &bus_cfg, SPI_DMA_CH_AUTO);
	ESP_ERROR_CHECK(ret);

	// Add MCP2515 SPI device to the bus
	ret = spi_bus_add_device(spi, &dev_cfg, &(MCP2515_Object->spi));
	ESP_ERROR_CHECK(ret);

	return true;
}

bool CAN_selftest(MCP2515 MCP2515_Object) {
    MCP2515_setLoopbackMode(MCP2515_Object);
    can_frame_t can_frame_tx = {0};
    can_frame_tx.can_id = (0x12344321) | CAN_EFF_FLAG;
	can_frame_tx.can_dlc = 8;
	can_frame_tx.data[0] = 0x01;
	can_frame_tx.data[1] = 0x02;
	can_frame_tx.data[2] = 0x03;
	can_frame_tx.data[3] = 0x04;
	can_frame_tx.data[4] = 0x05;
	can_frame_tx.data[5] = 0x06;
	can_frame_tx.data[6] = 0x07;
	can_frame_tx.data[7] = 0x08;

    if(MCP2515_sendMessageAfterCtrlCheck(MCP2515_Object, &can_frame_tx) != ERROR_OK){
        ESP_LOGE(TAG, "Couldn't send message.");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!MCP2515_checkReceive(MCP2515_Object)) {
        ESP_LOGE(TAG, "sent message, but not received");
        return false;
    } else {
        can_frame_t can_frame_rx = {0};
        ERROR_t ret = MCP2515_readMessageAfterStatCheck(MCP2515_Object, &can_frame_rx);
        if (ret != ERROR_OK) {
            ESP_LOGE(TAG, "Queued, but unable to pop");
            return false;
        }
        if (can_frame_rx.can_dlc != can_frame_tx.can_dlc || can_frame_rx.can_id != can_frame_tx.can_id || memcmp(can_frame_tx.data, can_frame_rx.data, can_frame_rx.can_dlc) != 0) {
            ESP_LOGE(TAG, "Data different from sent data");
        }
    }
    MCP2515_clearTXInterrupts(MCP2515_Object);
    return true;
}

mcp2515_ctx_t* CAN_Init(mcp2515_ctx_t* ctx, spi_host_device_t spi, int miso, int mosi, int clk, int cs, int int_pin)
{
	ctx->mcp_hardware_instance = MCP2515_init();
	SPI_Init(ctx->mcp_hardware_instance, spi, miso, mosi, clk, cs);
	MCP2515_reset(ctx->mcp_hardware_instance);
    if (!CAN_selftest(ctx->mcp_hardware_instance)) {
        ESP_LOGE(TAG, "CAN self-test failed");
        return NULL;
    }

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE, // Falling edge interrupt trigger
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << int_pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    gpio_isr_handler_add(int_pin, mcp2515_isr, ctx);

	MCP2515_setBitrate(ctx->mcp_hardware_instance, CAN_125KBPS, MCP_8MHZ);
	MCP2515_setListenOnlyMode(ctx->mcp_hardware_instance);
    // enable all interrupts
    MCP2515_setRegister(ctx->mcp_hardware_instance, MCP_CANINTE, 0XFF);
    return ctx;
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

    QueueHandle_t tx_queue_1 = xQueueCreate(4, sizeof(eventqueue_item_t));
    QueueHandle_t tx_queue_2 = xQueueCreate(4, sizeof(eventqueue_item_t));
    SemaphoreHandle_t semaphore_1 = xQueueCreateCountingSemaphore(1, 0);
    SemaphoreHandle_t semaphore_2 = xQueueCreateCountingSemaphore(1, 0);

    mcp2515_ctx_t ctx_1 = {
        .id = 1,
        .mcp_hardware_instance = NULL,
        .incoming_event_queue = tx_queue_1,
        .opposing_event_queue = tx_queue_2,
        .incoming_semaphore = semaphore_1,
        .opposing_semaphore = semaphore_2,
    };
    mcp2515_ctx_t ctx_2 = {
        .id = 2,
        .mcp_hardware_instance = NULL,
        .incoming_event_queue = tx_queue_2,
        .opposing_event_queue = tx_queue_1,
        .incoming_semaphore = semaphore_2,
        .opposing_semaphore = semaphore_1,
    };

    // Install GPIO ISR service
    gpio_install_isr_service(0);

    // if (!CAN_Init(&ctx_1, MCP2515_1_SPI_INSTANCE, PIN_1_MCP2515_MISO, PIN_1_MCP2515_MOSI, PIN_1_MCP2515_CLK, PIN_1_MCP2515_CS, PIN_1_MCP2515_INT)) {
    //     ESP_LOGE(TAG, "failed to initialize CAN 1");
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     abort();
    // }

    if (!CAN_Init(&ctx_2, MCP2515_2_SPI_INSTANCE, PIN_2_MCP2515_MISO, PIN_2_MCP2515_MOSI, PIN_2_MCP2515_CLK, PIN_2_MCP2515_CS, PIN_2_MCP2515_INT)) {
        ESP_LOGE(TAG, "failed to initialize CAN 2");
        vTaskDelay(pdMS_TO_TICKS(1000));
        abort();
    }
    ESP_LOGI(TAG, "CAN initialized");
    // assert(ctx_1.mcp_hardware_instance != NULL);
    assert(ctx_2.mcp_hardware_instance != NULL);

    // 1 = JOYSTICK
    // 2 = MOTOR DRIVER

    MCP2515_setNormalMode(ctx_2.mcp_hardware_instance);

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
        mcp2515_handle_interrupt_task,         // Task function
        "mcp2515_2",       // Task name
        15000,             // Stack size (bytes)
        &ctx_2,              // Parameters
        configMAX_PRIORITIES  - 2,                 // Priority
        &mcp2515_message_task_2,  // Task handle
        0                  // Core 0
    );

    xTaskCreatePinnedToCore(
        comms_task, // Task function
        "comms",       // Task name
        15000,             // Stack size (bytes)
        &ctx_1,              // Parameters
        9,                 // Priority
        &mcp2515_message_task_1,  // Task handle
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
