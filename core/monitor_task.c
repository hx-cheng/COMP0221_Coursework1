#include "config.h"
#include "neighbour_table.h"
#include "physics_integration.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "MONITOR";

extern QueueHandle_t state_queue;

extern Neighbour_table NeighbourState[MAX_NEIGHBOURS];

extern float neighbour_last_rssi[MAX_NEIGHBOURS];
extern float neighbour_last_snr[MAX_NEIGHBOURS];
extern float neighbour_latency_ms[MAX_NEIGHBOURS];
extern float neighbour_jitter_ms[MAX_NEIGHBOURS];

extern uint32_t neighbour_last_seq[MAX_NEIGHBOURS];   // last seq
extern uint32_t neighbour_lost_count[MAX_NEIGHBOURS]; // lost
extern uint32_t neighbour_recv_count[MAX_NEIGHBOURS]; // receive

static float loss_rate(int i)
{
    uint32_t recv = neighbour_recv_count[i];
    uint32_t lost = neighbour_lost_count[i];

    if (recv + lost == 0) return 0.0f;
    return (float)lost / (recv + lost);
}

void monitor_task(void *pv)
{
    PhysicsState self;

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        if (xQueuePeek(state_queue, &self, 0) == pdTRUE) {
            
            extern volatile uint32_t self_tx_seq;

            ESP_LOGI(TAG,
                "SELF STATE Pos(%u,%u,%u) Vel(%d,%d,%d) Yaw=%u SEQ=%lu",
                self.x_mm,
                self.y_mm,
                self.z_mm,
                self.vx_mm_s,
                self.vy_mm_s,
                self.vz_mm_s,
                self.yaw_cd,
                self_tx_seq
            );
        }

        for (int i = 0; i < MAX_NEIGHBOURS; i++) {

            if (neighbour_recv_count[i] == 0) continue;

            ESP_LOGI(TAG,
                "NEIGHBOUR[%d] %02X:%02X:%02X:%02X:%02X:%02X "
                "Pos(%u,%u,%u) Vel(%d,%d,%d) Yaw=%u "
                "RSSI=%.1f SNR=%.1f LAT=%.1fms JIT=%.1fms "
                "SEQ=%lu LOSS=%.1f%%",

                i,
                NeighbourState[i].node_id[0], NeighbourState[i].node_id[1],
                NeighbourState[i].node_id[2], NeighbourState[i].node_id[3],
                NeighbourState[i].node_id[4], NeighbourState[i].node_id[5],

                NeighbourState[i].x_mm,
                NeighbourState[i].y_mm,
                NeighbourState[i].z_mm,
                NeighbourState[i].vx_mm_s,
                NeighbourState[i].vy_mm_s,
                NeighbourState[i].vz_mm_s,
                NeighbourState[i].yaw_cd,

                neighbour_last_rssi[i],
                neighbour_last_snr[i],
                neighbour_latency_ms[i],
                neighbour_jitter_ms[i],                          

                neighbour_last_seq[i],
                loss_rate(i) * 100.0f
            );
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
    }
}