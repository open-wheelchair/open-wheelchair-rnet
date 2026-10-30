#pragma once

// Permissive Action Link
// time-limited joystick control

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t right;
    uint8_t fwd;
    bool is_seating_mode;
    int64_t timestamp_us;

} pal_serial_input_t;

typedef struct pal
{
    uint8_t right;
    uint8_t fwd;
    bool is_seating_mode;
    int64_t timestamp_us;
    bool use_serial;
} pal_wireless_input_t;

void pal_init();

void pal_push_serial(pal_serial_input_t* serial_input);
void pal_push_wireless(pal_wireless_input_t* wireless_input);

void pal_get_joystick(uint8_t* right_out, uint8_t* fwd_out, bool* is_drive_mode_out);
