#include "pal.h"

#include "freertos/FreeRTOS.h"

#include "esp_timer.h"
#include "esp_log.h"

#define VALID_TIME_WIRELESS_US (1000*1000)
#define VALID_TIME_SERIAL_US (100*1000)

static const char* TAG = "pal";

QueueHandle_t pal_serial_queue;
QueueHandle_t pal_wireless_queue;

void pal_init() {
    (void) TAG; // avoid unused warning
    pal_serial_queue = xQueueCreate(1, sizeof(pal_serial_input_t));
    pal_wireless_queue = xQueueCreate(1, sizeof(pal_wireless_input_t));
}

void pal_push_serial(pal_serial_input_t* serial_input) {
    xQueueOverwrite(pal_serial_queue, serial_input);
}

void pal_push_wireless(pal_wireless_input_t* wireless_input) {
    xQueueOverwrite(pal_wireless_queue, wireless_input);
}

void pal_get_joystick(pal_output_t* output) {
    static pal_serial_input_t serial_input = {0};
    static pal_wireless_input_t wireless_input = {0};

    int64_t current_time_us = esp_timer_get_time();

    // safe value
    output->right = 0;
    output->fwd = 0;
    output->is_seating_mode = false;
    output->seating_axis = PAL_SENTINEL_KEEP_SEATING_AXIS_SAME;

    // First, try to refresh both inputs
    pal_serial_input_t new_serial_input = {0};
    if (xQueueReceive(pal_serial_queue, &new_serial_input, 0)) {
        serial_input = new_serial_input;
    }
    pal_wireless_input_t new_wireless_input = {0};
    if (xQueueReceive(pal_wireless_queue, &new_wireless_input, 0)) {
        wireless_input = new_wireless_input;
    }

    // Now, check if the wireless input is valid
    if (current_time_us - wireless_input.timestamp_us > VALID_TIME_SERIAL_US) {
        // wireless input is invalid
        return;
    }
    // wireless input is valid; if serial is not enabled, we should just take the values from wireless
    if (!wireless_input.use_serial) {
        output->right = wireless_input.right;
        output->fwd = wireless_input.fwd;
        output->is_seating_mode = wireless_input.is_seating_mode;
        // the remote has no input for changing axis, so just allow using the joystick
        output->seating_axis = PAL_SENTINEL_KEEP_SEATING_AXIS_SAME;
        return;
    }
    // Wireless indicates that serial should be used, but it should also be valid
    if (current_time_us - serial_input.timestamp_us > VALID_TIME_SERIAL_US) {
        return;
    }
    // serial is valid
    output->fwd = serial_input.fwd;

    if (serial_input.is_seating_mode) {
        // in seating mode, left/right is used to select axis; using it might interfere with our axis selection code
        output->right = 0;
    } else {
        output->right = serial_input.right;
    }

    output->is_seating_mode = serial_input.is_seating_mode;
    output->seating_axis = serial_input.seating_axis;
}