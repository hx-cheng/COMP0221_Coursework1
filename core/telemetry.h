#pragma once

#include "physics_integration.h"
#include "freertos/queue.h"

#define TELEMETRY_DT (1000 / TELEMETRY_TASK_FREQ)   // 2Hz

extern QueueHandle_t packet_queue;

void init_sntp(void);
void wait_for_time_sync(void);
void wifi_connect_eduroam(void);
void telemetry_task(void *pv);