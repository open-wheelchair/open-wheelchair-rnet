#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "can.h"
#include "interface.h"
#include "pal.h"

static const char *TAG = "comms";

static const uint8_t SERIAL_NUMBER[8] = {0xf3, 0xc5, 0x1c, 0x62, 0, 0, 0, 0};

// interesting! only the last 4 bytes changed
static const uint8_t AUTH_CHAL_JOYSTICK_TO_MOTOR[8] = {0xa0, 0xa7, 0x5c, 0xea, 0xdb, 0x40, 0xcc, 0xef};


// will be filled in at runtime
static can_frame_t serial_number_frame = {
    .can_id = 0x00e,
    .can_dlc = 8,
    .data = {0},
};

void send_can_message(const can_hardware_ctx_t* ctx, const can_frame_t* frame) {
    eventqueue_item_t item = {
        .kind = EVKIND_FRAME,
        .d.frame = *frame
    };

    bool success = xQueueSendToBack(ctx->opposing_event_queue, &item, pdMS_TO_TICKS(100));
    if (success) {
        xSemaphoreGive(ctx->opposing_semaphore);
    } else {
        ESP_LOGE(TAG, "failed to queue frame %08lx len=%d", frame->can_id, frame->can_dlc);
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

// new architecture idea:
// multiple 'modules', with state machines and non-blocking (no more vtaskdelay)
// each module has following interface:
// poll(time) -> non-blocking, called every 1ms, can be used to transmit messages
// recv(frame, time) -> non-blocking frame handling, updates internal state, should not send messages
// modules also have access to the global object? or how do they pass info between them?
// how do modules tell other modules they should start?

typedef enum {
    INIT_STATE_POWERON,
    INIT_STATE_AUTHENTICATE,
    INIT_STATE_PARAMETER_MAP,
    INIT_STATE_WAIT_MODE,
    INIT_STATE_DONE
} init_fsm_state_t;

typedef enum {
    AUTH_STATE_WAIT_7b3_RTR_INITIAL,
    AUTH_STATE_GO,
    AUTH_STATE_WAIT_CONFIRM
} authenticate_fsm_substate_t;

typedef struct {
    authenticate_fsm_substate_t auth_fsm;

    int64_t ts_started;

    int64_t ts_sent_initial_7b3;

    int64_t ts_recv_7b3_rtr;

    int64_t ts_sent_7b1;

    int64_t ts_last_auth_frame;
    uint8_t current_slot;
    uint8_t my_auth_idx;
} auth_state_t;

typedef enum {
    PARAM_STATE_IDLE,
    PARAM_STATE_BLOCKED_WAITING_ANSWER,
    PARAM_STATE_BLOCKED_WAITING_IDLE,
    PARAM_STATE_WAITING,
    PARAM_STATE_RECEIVED
} param_fsm_t;

typedef struct {
    param_fsm_t param_fsm;

    int64_t ts_started_transaction; // by self or by other
    int64_t ts_recv_transaction_answer;

    int64_t ts_last_foreign_transaction; // last transaction initiated by other node

    can_frame_t answer;
} param_state_t;

typedef enum {
    PARAM_UPLOAD_STATE_FIRST_TRANS,
    PARAM_UPLOAD_STATE_WAIT_EXTRA_AFTER_SIGNAL_READY,
    PARAM_UPLOAD_STATE_REST_OF_PARAMS,
} param_upload_fsm_substate_t;

typedef struct {
    param_upload_fsm_substate_t param_upload_fsm;


    int64_t ts_begin_wait_after_signal_ready;
    size_t param_idx;
} param_upload_state_t;

typedef struct {
    uint32_t bitmask_expected;
    uint32_t bitmask_ready_seen;
    uint32_t bitmask_ack_seen;
} ready_state_t;

typedef enum {
    MODE_STATE_CLOSED,
    MODE_STATE_OPENING,
    MODE_STATE_OPEN,
    MODE_STATE_CLOSING,
} mode_state_fsm_substate_t;

typedef struct {
    uint8_t current_mode;
    uint8_t desired_mode;
    int64_t ts_transition_begin;

    mode_state_fsm_substate_t mode_fsm;
} mode_state_t;

typedef struct {
    can_hardware_ctx_t* hw_ctx;

    uint8_t own_node_address;
    uint8_t pm_node_address;

    bool sent_poweron;

    bool do_joystick_position_publish;
    int64_t ts_joystick_position_publication;

    bool do_pmtx_heartbeat_publish;
    int64_t ts_pmtx_heartbeat_publication;

    bool do_serial_number_publish;
    int64_t ts_serial_number_publication;


    param_state_t param_substate;
    ready_state_t ready_substate;
    mode_state_t mode_substate;

    init_fsm_state_t init_fsm;
    auth_state_t auth_substate;
    param_upload_state_t param_upload_substate;

} joystick_state_t;


void do_parameter_request(joystick_state_t* state, int64_t timestamp, uint8_t* request) {
    can_frame_t transaction_request = {
        .can_id = 0x780 | state->own_node_address,
        .can_dlc = 8,
    };
    memcpy(transaction_request.data, request, 8);
    state->param_substate.ts_started_transaction = timestamp;
    state->param_substate.param_fsm = PARAM_STATE_WAITING;
    send_can_message(state->hw_ctx, &transaction_request);
}

void get_parameter_response(joystick_state_t* state, int64_t timestamp, uint8_t* out_answer) {
    assert(state->param_substate.param_fsm == PARAM_STATE_RECEIVED);
    assert(state->param_substate.answer.can_dlc == 8);
    memcpy(out_answer, state->param_substate.answer.data, 8);

    can_frame_t empty = {0};
    state->param_substate.answer = empty;
    state->param_substate.param_fsm = PARAM_STATE_IDLE;
}

void init_poll(joystick_state_t* state, int64_t timestamp) {
    // PARAM subsystem
    switch (state->param_substate.param_fsm) {
        case PARAM_STATE_BLOCKED_WAITING_ANSWER:
        case PARAM_STATE_WAITING: {
            if (timestamp - state->param_substate.ts_started_transaction > 100*1000) {
                // elapsed without seeing answer
                ESP_LOGE(TAG, "param answer deadline elapsed");
            }
            break;
        }

        case PARAM_STATE_BLOCKED_WAITING_IDLE: {
            if (timestamp - state->param_substate.ts_last_foreign_transaction > 10*1000) {
                // elapsed without observing any further transactions
                state->param_substate.param_fsm = PARAM_STATE_IDLE;
            }
            break;
        }
        case PARAM_STATE_RECEIVED: {
            if (timestamp - state->param_substate.ts_recv_transaction_answer > 10*1000) {
                ESP_LOGE(TAG, "logic error: waited too long to handle received answer");
                abort();
            }
        }
        default: {
            break;
        }
    }
    // READY subsystem
    if (state->ready_substate.bitmask_ready_seen == state->ready_substate.bitmask_expected
        && (state->ready_substate.bitmask_ack_seen & (1 << state->own_node_address)) == 0
    ) {
        state->ready_substate.bitmask_ack_seen |= (1 << state->own_node_address);
        can_frame_t msg_seen_all_ready = {
            .can_id = 0x040 | state->own_node_address,
            .can_dlc = 4,
            .data = {0x80, 0, 0, 0}
        };
        send_can_message(state->hw_ctx, &msg_seen_all_ready);
    }

    // MODE subsystem (only active if READY)
    if (state->ready_substate.bitmask_ack_seen == state->ready_substate.bitmask_expected) {
        switch (state->mode_substate.mode_fsm) {
            case MODE_STATE_CLOSED: {
                // it's closed, let's open it
                can_frame_t msg_open_mode = {
                    .can_id = 0x060 | state->own_node_address,
                    .can_dlc = 4,
                    .data = {0, 0x10 | state->mode_substate.desired_mode, 0, 0}
                };
                send_can_message(state->hw_ctx, &msg_open_mode);
                state->mode_substate.mode_fsm = MODE_STATE_OPENING;
                state->mode_substate.ts_transition_begin = timestamp;
                break;
            }
            case MODE_STATE_OPEN: {
                if (state->mode_substate.current_mode != state->mode_substate.desired_mode) {
                    // begin transitioning
                    // it's open in the wrong mode, so first close it
                    can_frame_t msg_close_mode = {
                        .can_id = 0x060 | state->own_node_address,
                        .can_dlc = 4,
                        .data = {0x40, 0x10 | state->mode_substate.current_mode, 0, 0}
                    };
                    send_can_message(state->hw_ctx, &msg_close_mode);
                    state->mode_substate.mode_fsm = MODE_STATE_OPENING;
                    state->mode_substate.ts_transition_begin = timestamp;
                    }
                break;
            }

            case MODE_STATE_CLOSING:
            case MODE_STATE_OPENING: {
                if (timestamp - state->mode_substate.ts_transition_begin > 20*1000) {
                    ESP_LOGE(TAG, "mode transition took too long");
                    abort();
                }
                break;
            }
        }

    }

    // INIT subsystem
    switch (state->init_fsm) {
        case INIT_STATE_POWERON: {
            if (!state->sent_poweron) {
                state->sent_poweron = true;
                ESP_LOGW(TAG, "PHASE: poweron");
                const can_frame_t poweron_frame = {.can_id = 0x00C, .can_dlc = 0};
                send_can_message(state->hw_ctx, &poweron_frame); // this will get continually resent until acked
            }
            break;
        }
        case INIT_STATE_AUTHENTICATE: {
            if (timestamp - state->auth_substate.ts_started > 1000*1000) {
                ESP_LOGI(TAG, "stuck too long in auth");
                abort();
            }
            switch (state->auth_substate.auth_fsm) {
                case AUTH_STATE_WAIT_7b3_RTR_INITIAL: {
                    if (timestamp - state->auth_substate.ts_sent_initial_7b3 > 10*1000) {
                        ESP_LOGE(TAG, "deadline: initial 7b3 RTR response");
                        abort();
                    }
                    break;
                }
                case AUTH_STATE_GO: {
                    if (state->auth_substate.current_slot == state->own_node_address && timestamp - state->auth_substate.ts_recv_7b3_rtr > 20*1000 ) {
                        can_frame_t auth_frame = {
                            .can_id = CAN_EFF_FLAG | 0x1f000000 | (state->auth_substate.my_auth_idx << 20) | (state->auth_substate.current_slot << 16) | (AUTH_CHAL_JOYSTICK_TO_MOTOR[state->auth_substate.my_auth_idx] << 8) | SERIAL_NUMBER[state->auth_substate.my_auth_idx],
                            .can_dlc = 0
                        };
                        send_can_message(state->hw_ctx, &auth_frame);
                        state->auth_substate.ts_last_auth_frame = timestamp;
                        state->auth_substate.my_auth_idx++;
                        if (state->auth_substate.my_auth_idx == 8) {
                            state->auth_substate.current_slot++;
                        }
                    }
                    if (state->auth_substate.current_slot > state->own_node_address && timestamp - state->auth_substate.ts_last_auth_frame > 90*1000 && state->auth_substate.auth_fsm == AUTH_STATE_GO) {
                        state->auth_substate.ts_sent_7b1 = timestamp;
                        state->auth_substate.auth_fsm = AUTH_STATE_WAIT_CONFIRM;
                        can_frame_t auth_finish_frame = {.can_id = 0x7b1, .can_dlc = 0};
                        send_can_message(state->hw_ctx, &auth_finish_frame);
                    }
                    break;
                }
                case AUTH_STATE_WAIT_CONFIRM: {
                    if (timestamp - state->auth_substate.ts_sent_7b1 > 100*1000) {
                        ESP_LOGE(TAG, "did not receive auth confirm frame in time");
                        abort();
                    }
                    break;
                }
            }
            break;
        }
        case INIT_STATE_PARAMETER_MAP: {
            switch (state->param_upload_substate.param_upload_fsm) {
                case PARAM_UPLOAD_STATE_FIRST_TRANS: {
                    if (state->param_substate.param_fsm == PARAM_STATE_IDLE) {
                        uint8_t first_t_req[8] = {0x22, 0x80, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00};
                        do_parameter_request(state, timestamp, first_t_req);
                    }

                    if (state->param_substate.param_fsm == PARAM_STATE_RECEIVED) {
                        uint8_t answer[8] = {0};
                        get_parameter_response(state, timestamp, answer);

                        uint8_t expected_first_res[8] = {0xc0, 0x80, 0x00, 0x00, 0x26, 0x00, 0x00, 0x00};
                        assert(memcmp(answer, expected_first_res, 8) == 0);

                        can_frame_t signal_ready = {
                            .can_id = 0x1c240001 | state->own_node_address << 8 | CAN_EFF_FLAG,
                            .can_dlc = 0,
                        };
                        send_can_message(state->hw_ctx, &signal_ready);

                        state->param_upload_substate.param_upload_fsm = PARAM_UPLOAD_STATE_WAIT_EXTRA_AFTER_SIGNAL_READY;
                        state->param_upload_substate.ts_begin_wait_after_signal_ready = timestamp;
                    }
                    break;
                }
                case PARAM_UPLOAD_STATE_WAIT_EXTRA_AFTER_SIGNAL_READY: {
                    if (timestamp - state->param_upload_substate.ts_begin_wait_after_signal_ready > 120*1000) {
                        state->param_upload_substate.param_upload_fsm = PARAM_UPLOAD_STATE_REST_OF_PARAMS;
                    }
                    break;
                }
                case PARAM_UPLOAD_STATE_REST_OF_PARAMS: {
                    switch (state->param_substate.param_fsm) {
                        case (PARAM_STATE_IDLE): {
                            if (state->param_upload_substate.param_idx >= (sizeof(hardcoded_parameters)/sizeof(hardcoded_parameters[0]))) {
                                ESP_LOGW(TAG, "PHASE END PARAM UPLOAD");
                                // mark self as ready
                                state->ready_substate.bitmask_ready_seen |= (1 << state->own_node_address);
                                can_frame_t msg_i_am_ready = {
                                    .can_id = 0x040 | state->own_node_address,
                                    .can_dlc = 4,
                                    .data = {0x0, 0, 0, 0}
                                };
                                send_can_message(state->hw_ctx, &msg_i_am_ready);
                                state->init_fsm = INIT_STATE_WAIT_MODE;
                            } else {
                                do_parameter_request(state, timestamp, hardcoded_parameters[state->param_upload_substate.param_idx][0]);
                            }
                            break;
                        }
                        case (PARAM_STATE_RECEIVED): {
                            uint8_t answer[8] = {0};
                            get_parameter_response(state, timestamp, answer);
                            uint8_t* expected = hardcoded_parameters[state->param_upload_substate.param_idx][1];
                            if (memcmp(answer, expected, 8) != 0) {
                                ESP_LOGE(TAG, "answer does not match expected");
                                ESP_LOG_BUFFER_HEXDUMP(TAG, answer, 8, ESP_LOG_ERROR);
                                ESP_LOG_BUFFER_HEXDUMP(TAG, expected, 8, ESP_LOG_ERROR);
                                abort();
                            }
                            state->param_upload_substate.param_idx++;
                        }
                        default: {
                            break;
                        }
                    }
                }
            }
            break;
        }
        case INIT_STATE_WAIT_MODE: {
            if (state->mode_substate.mode_fsm == MODE_STATE_OPEN && state->mode_substate.desired_mode == state->mode_substate.current_mode) {
                ESP_LOGW(TAG, "PHASE MODE CHANGE DONE");
                can_frame_t msg_motor_something = {
                        .can_id = 0x0c180002 | state->own_node_address << 8 |  CAN_EFF_FLAG,
                        .can_dlc = 2,
                        .data = {0, 3}
                };
                send_can_message(state->hw_ctx, &msg_motor_something);
                can_frame_t msg_maxspeed = {
                        .can_id = 0x0a040000 | state->own_node_address << 8 | CAN_EFF_FLAG,
                        .can_dlc = 1,
                        .data = {0} // TODO this can be changed!
                };
                send_can_message(state->hw_ctx, &msg_maxspeed);

                state->init_fsm = INIT_STATE_DONE;
            }
        }

        default: {
            break;
        }
    }

    if (state->do_pmtx_heartbeat_publish) {
        if (timestamp - state->ts_pmtx_heartbeat_publication > 100*1000) {
            can_frame_t msg_pmtx_heartbeat = {
                .can_id = 0x03c30f0f | CAN_EFF_FLAG,
                .can_dlc = 7,
                .data = {0x87, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87}
            };
            send_can_message(state->hw_ctx, &msg_pmtx_heartbeat);
            state->ts_pmtx_heartbeat_publication = timestamp;
        }
    }
    if (state->do_serial_number_publish) {
        if (timestamp - state->ts_serial_number_publication > 50*1000) {
            send_can_message(state->hw_ctx, &serial_number_frame);
            state->ts_serial_number_publication = timestamp;
        }
    }

    // joystick timer
    if (state->init_fsm == INIT_STATE_DONE && state->mode_substate.mode_fsm == MODE_STATE_OPEN) {
        if (timestamp - state->ts_joystick_position_publication > 20*1000) {
            uint8_t right = 0;
            uint8_t fwd = 0;
            pal_get_joystick(&right, &fwd);

            can_frame_t msg_joystick = {
                    .can_id = 0x02000000 | state->own_node_address << 8 | CAN_EFF_FLAG,
                    .can_dlc = 2,
                    .data = {right, fwd}
            };

            send_can_message(state->hw_ctx, &msg_joystick);
            state->ts_joystick_position_publication = timestamp;
        }
    }

}

void init_receive(joystick_state_t* state, int64_t timestamp, can_frame_t* frame) {
    // PARAM subsystem
    if (!(frame->can_id & CAN_EFF_FLAG) && ((frame->can_id & CAN_SFF_MASK & ~0xf) == 0x780)) {
        // parameter request
        assert(
            (state->param_substate.param_fsm == PARAM_STATE_IDLE)
            || (state->param_substate.param_fsm == PARAM_STATE_BLOCKED_WAITING_IDLE)
        );
        state->param_substate.param_fsm = PARAM_STATE_BLOCKED_WAITING_ANSWER;
        state->param_substate.ts_started_transaction = timestamp;
    } else if (!(frame->can_id & CAN_EFF_FLAG) && ((frame->can_id & CAN_SFF_MASK & ~0xf) == 0x790)) {
        // parameter response
        assert(
            (state->param_substate.param_fsm == PARAM_STATE_WAITING)
            || (state->param_substate.param_fsm == PARAM_STATE_BLOCKED_WAITING_ANSWER)
        );
        if (state->param_substate.param_fsm == PARAM_STATE_WAITING) {
            state->param_substate.answer = *frame;
            state->param_substate.ts_recv_transaction_answer = timestamp;
            state->param_substate.param_fsm = PARAM_STATE_RECEIVED;
        } else if (state->param_substate.param_fsm == PARAM_STATE_BLOCKED_WAITING_ANSWER) {
            state->param_substate.ts_last_foreign_transaction = timestamp;
            state->param_substate.param_fsm = PARAM_STATE_BLOCKED_WAITING_IDLE;
        }
    }

    // READY subsystem
    if (!(frame->can_id & CAN_EFF_FLAG) && ((frame->can_id & CAN_SFF_MASK & ~0xf) == 0x040)) {
        assert(frame->can_dlc == 4);
        uint8_t from_node = (frame->can_id & CAN_SFF_MASK & 0xf);
        if (frame->data[0] == 0x00) { // 'i am done'
            // only one 'i am done' per node
            assert((state->ready_substate.bitmask_ready_seen & (1 << from_node)) == 0);
            state->ready_substate.bitmask_ready_seen |= (1 << from_node);
        } else if (frame->data[0] == 0x80) { // 'i saw that all nodes are done'
            // first, all nodes have to have been acked
            assert(state->ready_substate.bitmask_ready_seen == state->ready_substate.bitmask_expected);
            // only one ack per node
            assert((state->ready_substate.bitmask_ack_seen & (1 << from_node)) == 0);
            state->ready_substate.bitmask_ack_seen |= (1 << from_node);
        } else {
            ESP_LOGE(TAG, "unknown 040 message");
            abort();
        }
    }

    // MODE subsystem
    if (!(frame->can_id & CAN_EFF_FLAG) && ((frame->can_id & CAN_SFF_MASK & ~0xf) == 0x060)) {
        assert(frame->can_dlc == 4);
        uint8_t from_node = (frame->can_id & CAN_SFF_MASK & 0xf);

        // TODO what are all the other responses used for? for now, discard
        (void) from_node;
        if (frame->data[0] == 0x80) {
            // this is confirmation
            switch (state->mode_substate.mode_fsm) {
                case MODE_STATE_OPENING: {
                    state->mode_substate.mode_fsm = MODE_STATE_OPEN;
                    state->mode_substate.current_mode = state->mode_substate.desired_mode;
                    break;
                }
                case MODE_STATE_CLOSING: {
                    state->mode_substate.mode_fsm = MODE_STATE_CLOSED;
                    break;
                }
                default: {
                    ESP_LOGE(TAG, "recv mode change confirm, but in state %d", state->mode_substate.mode_fsm);
                    abort();
                }
            }
        }
    }


    // INIT subsystem
    switch (state->init_fsm) {
        case INIT_STATE_AUTHENTICATE: {
            if ((frame->can_id & 0x1f000000) == 0x1f000000) {
                state->auth_substate.ts_last_auth_frame = timestamp;
                uint8_t rx_node = (frame->can_id >> 16) & 0xf;
                uint8_t rx_auth_idx = (frame->can_id >> 20) & 0xf;
                if (rx_auth_idx == 7) {
                    state->auth_substate.current_slot = rx_node + 1;
                }
            }
            if (frame->can_id == (0x7b3 | CAN_RTR_FLAG)) {
                if (state->auth_substate.auth_fsm == AUTH_STATE_WAIT_7b3_RTR_INITIAL) {
                    state->auth_substate.auth_fsm = AUTH_STATE_GO;
                }

                state->auth_substate.current_slot = 0;
                state->auth_substate.my_auth_idx = 0;
                state->auth_substate.ts_recv_7b3_rtr = timestamp;
            }
            if (frame->can_id == 0x7b0 || (frame->can_id == (0x7b0 | CAN_RTR_FLAG))) {
                // we're done!
                if (state->auth_substate.current_slot <= state->own_node_address || state->auth_substate.current_slot <= state->pm_node_address) {
                    ESP_LOGE(TAG, "got 7b0, but not succesful!");
                    abort();
                }
                if (frame->can_id == (0x7b0 | CAN_RTR_FLAG)) {
                    ESP_LOGE(TAG, "got 7b0 RTR, expect weirdness");
                }
                ESP_LOGW(TAG, "END PHASE AUTHENTICATE");
                state->init_fsm = INIT_STATE_PARAMETER_MAP;
                state->do_pmtx_heartbeat_publish = true;
            }
            if (frame->can_id == 0x7b2) {
                ESP_LOGE(TAG, "Got 7b2");
                abort();
            }
            break;
        }
        default: {
            break;
        }
    }
}

void init_initial_ack(joystick_state_t* state, int64_t timestamp) {
    ESP_LOGW(TAG, "END PHASE POWERON");

    ESP_LOGW(TAG, "START PHASE AUTHENTICATE");
    state->init_fsm = INIT_STATE_AUTHENTICATE;
    state->auth_substate.auth_fsm = AUTH_STATE_WAIT_7b3_RTR_INITIAL;

    memcpy(serial_number_frame.data, SERIAL_NUMBER, sizeof(SERIAL_NUMBER));

    send_can_message(state->hw_ctx, &serial_number_frame);
    send_can_message(state->hw_ctx, &serial_number_frame);
    state->do_serial_number_publish = true;
    state->ts_serial_number_publication = timestamp;

    can_frame_t auth_init_frame = {.can_id = 0x7b3, .can_dlc = 0};
    send_can_message(state->hw_ctx, &auth_init_frame);
    state->auth_substate.ts_started = timestamp;
    state->auth_substate.ts_sent_initial_7b3 = timestamp;
}


void comms_task(void* pvParam) {
    can_hardware_ctx_t* ctx = (can_hardware_ctx_t*) pvParam;

    joystick_state_t state = {.hw_ctx = ctx, .init_fsm = INIT_STATE_POWERON,
        .own_node_address = 0, // first in the auth sequence
        .pm_node_address = 2, // TODO find this dynamically

        .ready_substate = {
            .bitmask_expected = 0b111, // three nodes
        },
        .mode_substate = {
            .desired_mode = 0
        }
    };

    while (true) {
        eventqueue_item_t item = {0};
        if (xQueueReceive(ctx->incoming_event_queue, &item, 1)) {
            if (item.kind == EVKIND_FRAME) {
                init_receive(&state, esp_timer_get_time(), &(item.d.frame));
            } else if (item.kind == EVKIND_SIGNAL_FIRST_ACK) {
                init_initial_ack(&state, esp_timer_get_time());
            }
        }
        init_poll(&state, esp_timer_get_time());
    }


    // TODO implement
    // can_frame_t toggle_lights = {
    //     .can_dlc = 0,
    //     .can_id = CAN_EFF_FLAG | 0x0c000004,
    // };
    // send_can_message(ctx, &toggle_lights);
}