#pragma once

#include "freertos/queue.h"
#include "config.h"

#define PHYSICS_DT (1000 / PHYSICS_TASK_FREQ)   // 50Hz

extern QueueHandle_t command_queue;
extern QueueHandle_t state_queue;

void random_physics_state(void);
void physics_integration_task(void *pv);