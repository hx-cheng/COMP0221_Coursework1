#pragma once

#include "neighbour_table.h"
#include "config.h"
#include "freertos/queue.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FLOCKING_DT (1000 / FLOCKING_TASK_FREQ)   // 10Hz

extern QueueHandle_t state_queue;     
extern QueueHandle_t command_queue; 

ControlCommand compute_flocking_command(const Neighbour_table *neighbours, 
                                        int count,
                                        const PhysicsState *self);

void flocking_control_task(void *pv);
                                        