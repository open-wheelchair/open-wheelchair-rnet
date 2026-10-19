#pragma once

#include "mcp2515.h"
#include "freertos/FreeRTOS.h"

extern volatile bool has_seen_ack;

typedef struct {
    MCP2515 mcp_hardware_instance;
    QueueHandle_t incoming_event_queue;
    QueueHandle_t opposing_event_queue;
    // TODO move out the semaphores and do everything via event queues
    // we will then likely need to buffer the messages waiting in another queue
    SemaphoreHandle_t incoming_semaphore;
    SemaphoreHandle_t opposing_semaphore;
    int id;
} mcp2515_ctx_t;


typedef enum {
    EVKIND_FRAME,
    EVKIND_TIMER,
    EVKIND_SIGNAL
} eventkind_t;

typedef struct {
    eventkind_t kind;
    union {
        can_frame_t frame;
        int timer_id;
    } d;
} eventqueue_item_t;

void comms_task(void* pvParam);

void log_can_frame(uint8_t receiver, uint8_t slot, can_frame_t frame);