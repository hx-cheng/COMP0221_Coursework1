#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "config.h"

#define RADIO_DT (1000 / LORA_TASK_FREQ)   // 2Hz

extern volatile uint32_t self_tx_seq;

extern QueueHandle_t state_queue;
extern QueueHandle_t packet_queue;
extern Neighbour_table NeighbourState[MAX_NEIGHBOURS];
extern uint32_t neighbour_last_seen[MAX_NEIGHBOURS];

extern float neighbour_last_rssi[MAX_NEIGHBOURS];
extern float neighbour_last_snr[MAX_NEIGHBOURS];

extern float neighbour_latency_ms[MAX_NEIGHBOURS];
extern float neighbour_jitter_ms[MAX_NEIGHBOURS];

extern uint32_t neighbour_last_seq[MAX_NEIGHBOURS];   // last seq
extern uint32_t neighbour_lost_count[MAX_NEIGHBOURS]; // lost
extern uint32_t neighbour_recv_count[MAX_NEIGHBOURS]; // receive

void init_mac();
bool init_radio();
int lora_send_packet(uint8_t *data, size_t len);
int lora_receive_packet(uint8_t *buf, size_t buf_len);
//void neighbor_table_update(const Neighbour_table *pkt);
void radio_task(void *pv);



#ifdef __cplusplus
}
#endif