#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "can.h"
#include "interface.h"
#include "pal.h"


// pinout joystick controller


// 1
// 2
// 3
// 4
// 5 = GND
// 6
// 7
// 8

// periodicities
// serial packet every 50ms
// joystick position every 10ms

// HEARTBEAT 03c30f0f: 87 87 87 87 87 87 87 every 100ms
// ^ does not get acked, so maybe we omit this one for now?

typedef enum {
    TID_SERIAL,
    TID_JOYSTICK_POS,
} timer_id_t;

static const char *TAG = "comms";

static const uint8_t SERIAL_NUMBER[8] = {0xf3, 0xc5, 0x1c, 0x62, 0, 0, 0, 0};
//static const uint8_t AUTH_CHAL_JOYSTICK_TO_MOTOR[8] = {0x85, 0xe7, 0x1e, 0xca, 0xdb, 0x40, 0xcc, 0xef};

// interesting! only the last 4 bytes changed
static const uint8_t AUTH_CHAL_JOYSTICK_TO_MOTOR[8] = {0xa0, 0xa7, 0x5c, 0xea, 0xdb, 0x40, 0xcc, 0xef};


// will be filled in at runtime
static can_frame_t serial_number_frame = {
    .can_id = 0x00e,
    .can_dlc = 8,
    .data = {0},
};

typedef enum {
    INIT,
    AUTH_SENT_OK,
    AUTH, // finished by receiving AUTH 1f72xxxx frame, then wait 100ms
    AUTH_CHECK, // finished by receiving 7b0
} controller_fsm_state_t;



static controller_fsm_state_t fsm_state = INIT;

void send_can_message(const mcp2515_ctx_t* ctx, const can_frame_t* frame) {
    // log_can_frame(20, 0, *frame);

    eventqueue_item_t item = {
        .kind = EVKIND_FRAME,
        .d.frame = *frame
    };

    bool success = xQueueSendToBack(ctx->opposing_event_queue, &item, pdMS_TO_TICKS(100));
    if (success) {
        printf("F%dF\n", uxQueueSpacesAvailable(ctx->opposing_event_queue));
        xSemaphoreGive(ctx->opposing_semaphore);
    } else {
        ESP_LOGE(TAG, "failed to queue frame %08lx len=%d", frame->can_id, frame->can_dlc);
    }
}

void periodic_pmtx_heartbeat(TimerHandle_t xTimer) {
    mcp2515_ctx_t* ctx = (mcp2515_ctx_t*) pvTimerGetTimerID(xTimer);

    can_frame_t msg_pmtx_heartbeat = {
            .can_id = 0x03c30f0f | CAN_EFF_FLAG,
            .can_dlc = 7,
            .data = {0x87, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87}
    };

    send_can_message(ctx, &msg_pmtx_heartbeat);
}

bool handle_receive_queue(const mcp2515_ctx_t* ctx, can_frame_t* frame_out, uint64_t ms_to_wait) {
    uint64_t finish_timestamp_us = esp_timer_get_time() + 1000*ms_to_wait;
    while (esp_timer_get_time() < finish_timestamp_us) {
        uint64_t ticks_to_wait = pdMS_TO_TICKS((esp_timer_get_time() - finish_timestamp_us)/1000);
        if (ticks_to_wait == 0 && ms_to_wait > 0) {
            ticks_to_wait = 1;
        }
        eventqueue_item_t item;
        bool received = xQueueReceive(ctx->incoming_event_queue, &item, ticks_to_wait);
        if (received) {
            if (item.kind == EVKIND_FRAME) {
                *frame_out = item.d.frame;
                ESP_LOGI(TAG, "found frame, %d msg waiting in queue", uxQueueMessagesWaiting(ctx->incoming_event_queue));
                return true;
            } else if (item.kind == EVKIND_TIMER) {
                switch (item.d.timer_id) {
                    default: {
                        ESP_LOGE(TAG, "unknown timer ID");
                    }
                }
            }
        }
    }
    return false;
}

void poweron(mcp2515_ctx_t* ctx) {
    ESP_LOGW(TAG, "PHASE: poweron");
    const can_frame_t poweron_frame = {
        .can_id = 0x00C,
        .can_dlc = 0
    };
    send_can_message(ctx, &poweron_frame); // this will get continually resent until acked
    // send_can_message(ctx, &poweron_frame);
    // while (!has_seen_ack) {
    //     // // TODO this busy-loops, not ideal
    //     // // if queue is empty, queue a frame
    //     // while (!has_seen_ack && uxQueueMessagesWaitingFromISR(ctx->opposing_event_queue) < 1 ) {
    //     //     ESP_LOGI(TAG, "queueing poweron frame");
    //     //     send_can_message(ctx, &poweron_frame);
    //     // }
    //     vTaskDelay(1);
    // }
    // vTaskDelay(1);
}

void periodic_serial_number(TimerHandle_t xTimer) {
    mcp2515_ctx_t* ctx = (mcp2515_ctx_t*) pvTimerGetTimerID(xTimer);
    send_can_message(ctx, &serial_number_frame);
}

void start_pmtx(mcp2515_ctx_t* ctx) {
    TimerHandle_t pmtx_timerhandle = xTimerCreate(
        "pmtx_tmr",
        pdMS_TO_TICKS(100),
        pdTRUE,
        (void*) ctx,
        periodic_pmtx_heartbeat
    );
    assert(pmtx_timerhandle);
    assert(xTimerStart(pmtx_timerhandle, 0));
}

void authenticate(mcp2515_ctx_t* ctx) {
    // Fill in serial number
    memcpy(serial_number_frame.data, SERIAL_NUMBER, sizeof(SERIAL_NUMBER));

    send_can_message(ctx, &serial_number_frame);
    send_can_message(ctx, &serial_number_frame);

    //ESP_LOGW(TAG, "PHASE: auth init frame");
    can_frame_t auth_init_frame = {
        .can_id = 0x7b3,
        .can_dlc = 0,
    };
    send_can_message(ctx, &auth_init_frame);
    ESP_LOGW(TAG, "PHASE: serial number");

    // Create 50 ms timer for serial number
    TimerHandle_t serial_timerhandle = xTimerCreate(
        "ser_tmr",
        pdMS_TO_TICKS(50),
        pdTRUE,
        (void*) ctx,
        periodic_serial_number
    );
    assert(serial_timerhandle);

    can_frame_t rx_frame = {0};
    bool received = handle_receive_queue(ctx, &rx_frame, 10);
    assert(received && rx_frame.can_id == (0x7b3 | CAN_RTR_FLAG));

    can_frame_t auth_frame_0 = {
        .can_id = CAN_EFF_FLAG | 0x1f000000 | (0 << 20) | (AUTH_CHAL_JOYSTICK_TO_MOTOR[0] << 8) | SERIAL_NUMBER[0],
        .can_dlc = 0
    };
    ESP_LOGW(TAG, "PHASE: 1");
    send_can_message(ctx, &auth_frame_0);
    vTaskDelay(pdMS_TO_TICKS(17));
    send_can_message(ctx, &auth_frame_0);

    {
        can_frame_t rx_frame = {0};
        bool received = handle_receive_queue(ctx, &rx_frame, 10);
        assert(received && rx_frame.can_id == (0x7b3 | CAN_RTR_FLAG));
        can_frame_t auth_frame_1 = {
            .can_id = CAN_EFF_FLAG | 0x1f000000 | (1 << 20) | (AUTH_CHAL_JOYSTICK_TO_MOTOR[1] << 8) | SERIAL_NUMBER[1],
            .can_dlc = 0
        };
        ESP_LOGW(TAG, "PHASE: 2");
        send_can_message(ctx, &auth_frame_1);
    }

    {
        can_frame_t rx_frame = {0};
        bool received = handle_receive_queue(ctx, &rx_frame, 10);
        assert(received && rx_frame.can_id == (0x7b3 | CAN_RTR_FLAG));
        vTaskDelay(pdMS_TO_TICKS(18));
        ESP_LOGW(TAG, "PHASE: 3");
        send_can_message(ctx, &serial_number_frame);
        // Start serial timer handle here
        assert(xTimerStart(serial_timerhandle, 0));
    }

    for (uint32_t auth_ctr = 0; auth_ctr < 8; auth_ctr++) {
        can_frame_t auth_frame = {
            .can_id = CAN_EFF_FLAG | 0x1f000000 | (auth_ctr << 20) | (AUTH_CHAL_JOYSTICK_TO_MOTOR[auth_ctr] << 8) | SERIAL_NUMBER[auth_ctr],
            .can_dlc = 0
        };
        send_can_message(ctx, &auth_frame);
    }
    ESP_LOGW(TAG, "PHASE: 4");

    {
        can_frame_t rx_frame = {0};
        bool received = handle_receive_queue(ctx, &rx_frame, 60);
        assert(received && rx_frame.can_id == (0x7b3 | CAN_RTR_FLAG));
        send_can_message(ctx, &auth_init_frame);
        vTaskDelay(pdMS_TO_TICKS(10));
        // send_can_message(ctx, &serial_number_frame);
    }
    ESP_LOGW(TAG, "PHASE: 5");
    {
        bool received = handle_receive_queue(ctx, &rx_frame, 10);
        assert(received && rx_frame.can_id == 0x7b3);
    }
    ESP_LOGW(TAG, "PHASE: 6");
    for (uint32_t auth_ctr = 0; auth_ctr < 8; auth_ctr++) {
        can_frame_t auth_frame = {
            .can_id = CAN_EFF_FLAG | 0x1f000000 | (auth_ctr << 20) | (AUTH_CHAL_JOYSTICK_TO_MOTOR[auth_ctr] << 8) | SERIAL_NUMBER[auth_ctr],
            .can_dlc = 0
        };
        send_can_message(ctx, &auth_frame);
    }
    fsm_state = INIT;

    ESP_LOGW(TAG, "PHASE: waiting for auth 72");
    while (fsm_state == INIT) {
        bool received = handle_receive_queue(ctx, &rx_frame, 1);
        if (received) {
            if (((rx_frame.can_id & CAN_EFF_MASK) & 0xff000000) == 0x1f000000) {
                ESP_LOGI(TAG, "auth frame");
                if (((rx_frame.can_id & CAN_EFF_MASK) & 0xffff0000) == 0x1f720000) {
                    fsm_state = AUTH_SENT_OK;
                }
            } else {
                ESP_LOGE(TAG, "unexpected frame!");
            }
        }
    }
    ESP_LOGW(TAG, "PHASE: serials and 7b1");
    vTaskDelay(pdMS_TO_TICKS(50));
    vTaskDelay(pdMS_TO_TICKS(50));
    vTaskDelay(pdMS_TO_TICKS(30));
    can_frame_t auth_confirm_frame = {
        .can_id = 0x7b1,
        .can_dlc = 0,
    };
    send_can_message(ctx, &auth_confirm_frame);
    vTaskDelay(pdMS_TO_TICKS(40));
    {
        bool received = handle_receive_queue(ctx, &rx_frame, 200);
        assert(received && (rx_frame.can_id == 0x7b0 || rx_frame.can_id == (CAN_RTR_FLAG | 0x7b0) ));
        // TODO what does 0x7b0 RTR mean? in the captures of the real controller, we only ever see without RTR
        if (rx_frame.can_id == (CAN_RTR_FLAG | 0x7b0)) {
            ESP_LOGE(TAG, "saw 7b0 RTR frame");
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    start_pmtx(ctx);
}

// receives until there is an exact match with an expected frame
bool receive_but_ignore_non_specific_frame(mcp2515_ctx_t* ctx, can_frame_t* expected, uint64_t ms_to_wait) {
    uint64_t finish_timestamp_us = esp_timer_get_time() + 1000*ms_to_wait;
    while (esp_timer_get_time() < finish_timestamp_us) {
        uint64_t ticks_to_wait = pdMS_TO_TICKS((esp_timer_get_time() - finish_timestamp_us)/1000);
        if (ticks_to_wait == 0 && ms_to_wait > 0) {
            ticks_to_wait = 1;
        }
        eventqueue_item_t item;
        bool received = xQueueReceive(ctx->incoming_event_queue, &item, ticks_to_wait);
        if (received) {
            if (item.kind == EVKIND_FRAME) {
                if (item.d.frame.can_id != expected->can_id || item.d.frame.can_dlc != expected->can_dlc || 0 != memcmp(item.d.frame.data, expected->data, expected->can_dlc)) {
                    // ESP_LOGW(TAG, "ignoring frame");
                    continue;
                }
                ESP_LOGI(TAG, "found frame, %d msg waiting in queue", uxQueueMessagesWaiting(ctx->incoming_event_queue));
                return true;
            } else if (item.kind == EVKIND_TIMER) {
                switch (item.d.timer_id) {
                    default: {
                        ESP_LOGE(TAG, "unknown timer ID");
                    }
                }
            }
        }
    }
    return false;
}

void parameter_transaction(mcp2515_ctx_t* ctx, uint8_t* request, uint8_t* expected_response) {
    can_frame_t transaction_request = {
        .can_id = 0x780,
        .can_dlc = 8,
    };
    memcpy(transaction_request.data, request, 8);
    send_can_message(ctx, &transaction_request);
    can_frame_t frame_expected = {
        .can_dlc = 8,
        .can_id = 0x792,
    };
    memcpy(frame_expected.data, expected_response, 8);
    // wait up to 200ms for an answer
    bool received = receive_but_ignore_non_specific_frame(ctx, &frame_expected, 200);
    if (!received) {
        ESP_LOGE(TAG, "no answer");
        assert(false);
    }
}

uint8_t hardcoded_parameters[][2][8] = {
    {{0x22, 0x80, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00}, {0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {{0x42, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, {0x20, 0x80, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00}},
    {{0x22, 0x81, 0x00, 0x00, 0x06, 0x00, 0x01, 0x00}, {0x40, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {{0x42, 0x8f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
     {0x20, 0x8f, 0x00, 0x00, 0xee, 0xff, 0x00, 0x00}},
     // old
            // {0x20, 0x8f, 0x00, 0x00, 0xbf, 0xe3, 0x00, 0x00}},
     // oldest value:
        // {0x20, 0x8f, 0x00, 0x00, 0x57, 0x4c, 0x00, 0x00}},
    {{0x22, 0x81, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00}, {0x40, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {{0x42, 0x8f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x20, 0x8f, 0x00, 0x00, 0xde, 0x26, 0x00, 0x00}},
            // {0x20, 0x8f, 0x00, 0x00, 0x9a, 0x7b, 0x00, 0x00}},
    // old value:
        // {0x20, 0x8f, 0x00, 0x00, 0x59, 0x83, 0x00, 0x00}},

    {{0x22, 0x81, 0x00, 0x00, 0x08, 0x00, 0x01, 0x00}, {0x40, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {{0x42, 0x8f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, {0x20, 0x8f, 0x00, 0x00, 0x06, 0x23, 0x00, 0x00}},
    {{0x22, 0x81, 0x00, 0x00, 0x0a, 0x00, 0x01, 0x00}, {0x40, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {{0x42, 0x8f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, {0x20, 0x8f, 0x00, 0x00, 0x72, 0xb9, 0x00, 0x00}},
    {{0x22, 0x81, 0x00, 0x00, 0x0b, 0x00, 0x01, 0x00}, {0x40, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {{0x42, 0x8f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, {0x20, 0x8f, 0x00, 0x00, 0x36, 0xb0, 0x00, 0x00}},
    {{0x22, 0x81, 0x00, 0x00, 0x0c, 0x00, 0x01, 0x00}, {0x40, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {{0x42, 0x8f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, {0x20, 0x8f, 0x00, 0x00, 0x52, 0x6a, 0x00, 0x00}},
    {{0x22, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, {0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {{0x42, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x20, 0x40, 0x00, 0x00, 0x00, 0x00, 0x7e, 0xff}},
    //  {0x20, 0x40, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff}},
    // old value: {0x20, 0x40, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff}},

    {{0x42, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    // old value: {0x20, 0x50, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00}},
    // {0x20, 0x50, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x00}},
    {0x20, 0x50, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00}},
};

void parameter_map(mcp2515_ctx_t* ctx) {
    uint8_t first_t_req[8] = {0x22, 0x80, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00};
    uint8_t first_t_res[8] = {0xc0, 0x80, 0x00, 0x00, 0x26, 0x00, 0x00, 0x00};

    ESP_LOGW(TAG, "first transaction");
    parameter_transaction(ctx, first_t_req, first_t_res);
    vTaskDelay(pdMS_TO_TICKS(90));

    can_frame_t signal_ready = {
        .can_id = 0x1c240001 | CAN_EFF_FLAG,
        .can_dlc = 0,
    };
    send_can_message(ctx, &signal_ready);

    vTaskDelay(pdMS_TO_TICKS(120));
    for (int i = 0; i < (sizeof(hardcoded_parameters)/sizeof(hardcoded_parameters[0])); i++) {
        parameter_transaction(ctx, hardcoded_parameters[i][0], hardcoded_parameters[i][1]);
        ESP_LOGW(TAG, "done param %d", i);
    }

    {
        ESP_LOGW(TAG, "040_0");
        can_frame_t msg_040_0 = {
            .can_id = 0x040,
            .can_dlc = 4,
            .data = {0, 0, 0, 0}
        };
        send_can_message(ctx, &msg_040_0);
        can_frame_t msg_expected_042 = {
            .can_id = 0x042,
            .can_dlc = 4,
            .data = {0x80, 0, 0, 0}
        };
        assert(receive_but_ignore_non_specific_frame(ctx, &msg_expected_042, 120));
    }

    {
        ESP_LOGW(TAG, "040_80");
        can_frame_t msg_040_80 = {
            .can_id = 0x040,
            .can_dlc = 4,
            .data = {0x80, 0, 0, 0}
        };
        send_can_message(ctx, &msg_040_80);
        can_frame_t msg_expected_052 = {
            .can_id = 0x052,
            .can_dlc = 4,
            .data = {0x80, 0x01, 0x00, 0x01}
            // .data = {0x80, 0x00, 0x00, 0x01} -> when in tilting mode
        };
        assert(receive_but_ignore_non_specific_frame(ctx, &msg_expected_052, 120));
    }

    {
        ESP_LOGW(TAG, "060");
        can_frame_t msg_060 = {
            .can_id = 0x060,
            .can_dlc = 4,
            .data = {0, 0x10, 0, 0}
        };
        send_can_message(ctx, &msg_060);
        can_frame_t msg_expected_062 = {
            .can_id = 0x062,
            .can_dlc = 4,
            .data = {0x80, 0, 0, 0x80}
            // .data = {0x80, 0, 0, 0x20} -> when in tilting mode
        };
        assert(receive_but_ignore_non_specific_frame(ctx, &msg_expected_062, 120));
    }
    ESP_LOGE(TAG, "warn: wrong order");
    can_frame_t msg_motor_something = {
            .can_id = 0x0c180002 | CAN_EFF_FLAG,
            .can_dlc = 2,
            .data = {0, 3}
    };
    send_can_message(ctx, &msg_motor_something);
    can_frame_t msg_maxspeed = {
            .can_id = 0x0a040000 | CAN_EFF_FLAG,
            .can_dlc = 1,
            .data = {0} // TODO this can be changed!
    };
    send_can_message(ctx, &msg_maxspeed);
    // vTaskDelay(pdMS_TO_TICKS(1400));
    vTaskDelay(1400);
}

uint64_t starttime = 0;

void periodic_joystick(TimerHandle_t xTimer) {
    mcp2515_ctx_t* ctx = (mcp2515_ctx_t*) pvTimerGetTimerID(xTimer);

    uint64_t current_time = esp_timer_get_time();
    if (starttime == 0) {
        starttime = current_time;
    }

    uint8_t right = 0;
    uint8_t fwd = 0;
    pal_get_joystick(&right, &fwd);

    can_frame_t msg_joystick = {
            .can_id = 0x02000000 | CAN_EFF_FLAG,
            .can_dlc = 2,
            .data = {right, fwd}
    };

    send_can_message(ctx, &msg_joystick);
}

void start_joystick(mcp2515_ctx_t* ctx) {
    TimerHandle_t joystick_timerhandle = xTimerCreate(
        "joy_tmr",
        pdMS_TO_TICKS(10),
        pdTRUE,
        (void*) ctx,
        periodic_joystick
    );
    assert(joystick_timerhandle);
    assert(xTimerStart(joystick_timerhandle, 0));
}

void comms_task(void* pvParam) {
    mcp2515_ctx_t* ctx = (mcp2515_ctx_t*) pvParam;

    poweron(ctx);
    authenticate(ctx);
    parameter_map(ctx);
    start_joystick(ctx);

    can_frame_t toggle_lights = {
        .can_dlc = 0,
        .can_id = CAN_EFF_FLAG | 0x0c000004,
    };
    send_can_message(ctx, &toggle_lights);

    ESP_LOGW(TAG, "PHASE: done, waiting forever, drop all events");
    while (true) {
        eventqueue_item_t item = {0};
        xQueueReceive(ctx->incoming_event_queue, &item, 100000);
    }
}