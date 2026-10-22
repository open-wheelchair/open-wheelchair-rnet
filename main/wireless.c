#include <string.h>

#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"

#include "wireless.h"
#include "pal.h"

const char* TAG = "wireless";

#define ESPNOW_CHANNEL (4)

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, BROADCAST, ESP_NOW_ETH_ALEN) == 0)


static uint8_t BROADCAST[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* WiFi should start before using ESPNOW */
void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    uint8_t * mac_addr = recv_info->src_addr;
    uint8_t * des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    // if (IS_BROADCAST_ADDR(des_addr)) {
    //     /* If added a peer with encryption before, the receive packets may be
    //      * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
    //      * Users can check the destination address to distinguish it.
    //      */
    //     ESP_LOGI(TAG, "Receive broadcast ESPNOW data");
    // } else {
    //     ESP_LOGI(TAG, "Receive unicast ESPNOW data");
    // }

    if (len == 9 && memcmp(data, "ROBOB", 5) == 0) {
        // Reasonably sure this is a control packet
        // TODO use encryption
        bool manual_mode = !!data[5];
        uint8_t joystick_fwd_encoded = data[6];
        uint8_t joystick_right_encoded = data[7];
        uint8_t auto_speed = data[8];
        // TODO use auto_speed
        (void) auto_speed;

        pal_wireless_input_t wireless_input = {
            .fwd = joystick_fwd_encoded,
            .right = joystick_right_encoded,
            .use_serial = !manual_mode,
            .timestamp_us = esp_timer_get_time()
        };
        pal_push_wireless(&wireless_input);
    }
}

esp_err_t espnow_init(void)
{
    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    // ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    assert(peer);
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = ESPNOW_CHANNEL;
    peer->ifidx = WIFI_IF_STA;
    peer->encrypt = false;
    memcpy(peer->peer_addr, BROADCAST, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);
    return ESP_OK;
}