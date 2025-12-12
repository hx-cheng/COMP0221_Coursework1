#include "flocking_control.h"
#include "neighbour_table.h"
#include "config.h"

#include <math.h>
#include "esp_log.h"
#include "freertos/queue.h"

ControlCommand compute_flocking_command(const Neighbour_table *neighbours, 
                                        int count,
                                        const PhysicsState *self)
{   
    ControlCommand cmd = {0};
    
    // No neighbours
    if (count <= 0) {
        return cmd;
    }

    // STEP 1: Compute average pos, vel, sep
    int64_t sum_x = 0, sum_y = 0, sum_z = 0;
    int64_t sum_vx = 0, sum_vy = 0, sum_vz = 0;
    int64_t sep_x = 0, sep_y = 0, sep_z = 0;

    for (int i = 0; i < count; i++)
    {
        sum_x += neighbours[i].x_mm;
        sum_y += neighbours[i].y_mm;
        sum_z += neighbours[i].z_mm;

        sum_vx += neighbours[i].vx_mm_s;
        sum_vy += neighbours[i].vy_mm_s;
        sum_vz += neighbours[i].vz_mm_s;

        int32_t dx = self->x_mm - neighbours[i].x_mm;
        int32_t dy = self->y_mm - neighbours[i].y_mm;
        int32_t dz = self->z_mm - neighbours[i].z_mm;

        int64_t dist2 = (int64_t)dx*dx + (int64_t)dy*dy + (int64_t)dz*dz;
        if (dist2 < 1) dist2 = 1;

        // --- Separation ---
        if (dist2 < (int64_t)MIN_SEPARATION_MM * MIN_SEPARATION_MM) {
            float inv_dist = 1.0f / sqrtf((float)dist2);
            sep_x += dx * inv_dist;
            sep_y += dy * inv_dist;
            sep_z += dz * inv_dist;
        }
    }

    float avg_x = sum_x / count;
    float avg_y = sum_y / count;
    float avg_z = sum_z / count;

    float avg_vx = sum_vx / count;
    float avg_vy = sum_vy / count;
    float avg_vz = sum_vz / count;

    // STEP 2: Compute influence vectors
    // --- Cohesion ---
    float cohesion_x = (avg_x - self->x_mm);
    float cohesion_y = (avg_y - self->y_mm);
    float cohesion_z = (avg_z - self->z_mm);

    // --- Alignment ---
    float align_x = (avg_vx - self->vx_mm_s);
    float align_y = (avg_vy - self->vy_mm_s);
    float align_z = (avg_vz - self->vz_mm_s);

    // STEP 3: Combine to velocity update
    float vx = COHESION_WEIGHT * cohesion_x + ALIGNMENT_WEIGHT * align_x + SEPARATION_WEIGHT * sep_x;
    float vy = COHESION_WEIGHT * cohesion_y + ALIGNMENT_WEIGHT * align_y + SEPARATION_WEIGHT * sep_y;
    float vz = COHESION_WEIGHT * cohesion_z + ALIGNMENT_WEIGHT * align_z + SEPARATION_WEIGHT * sep_z;

    // STEP 4: Limit speed
    float speed = sqrtf(vx*vx + vy*vy + vz*vz);
    if (speed > MAX_SPEED_MMS) {
        float s = MAX_SPEED_MMS / speed;
        vx *= s;
        vy *= s;
        vz *= s;
    }

    cmd.vx = (int32_t)vx;
    cmd.vy = (int32_t)vy;
    cmd.vz = (int32_t)vz;

    // STEP 5: Compute heading
    float yaw = atan2f(cmd.vy, cmd.vx) * 180.0f / M_PI;

    // Wrap to 0–35999 centi-degrees
        while (yaw >= MAX_YAW) yaw -= 360.f;
        while (yaw < 0)        yaw += 360.f;

    cmd.yaw_rate = (uint16_t) (yaw * 100.0f);
    
    return cmd;
}

static int get_neighbors(Neighbour_table *neighbours)
{
    int count = 0;

    for (int i = 0; i < MAX_NEIGHBOURS; i++) {
        // node_id != 00:00:00:00:00:00
        if (memcmp(NeighbourState[i].node_id, "\0\0\0\0\0\0", 6) != 0) {
            neighbours[count++] = NeighbourState[i];
        }
    }
    return count;
}

void flocking_control_task(void *pv)
{
    ESP_LOGI("FLOCK", "FLocking Control Task started, period = %d ms", FLOCKING_DT);
    const TickType_t period_ticks = pdMS_TO_TICKS(FLOCKING_DT);
    TickType_t last_wake = xTaskGetTickCount();

    PhysicsState self_state;
    ControlCommand cmd;

    Neighbour_table neigh_copy[MAX_NEIGHBOURS];

    while (1)
    {   
        if (xQueuePeek(state_queue, &self_state, 0) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        int count = get_neighbors(neigh_copy);

        cmd = compute_flocking_command(
                neigh_copy,
                count,
                &self_state  
            );
        xQueueOverwrite(command_queue, &cmd);

        TickType_t now_tick = xTaskGetTickCount();
        TickType_t expected_wake = last_wake + period_ticks;

        if (now_tick > expected_wake) {
            ESP_LOGW("FLOCK",
                     "Overrun by %ld ms",
                     (now_tick - expected_wake) * portTICK_PERIOD_MS);
        }
        
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FLOCKING_DT));
    }
}

