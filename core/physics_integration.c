#include "esp_random.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "config.h"
#include "physics_integration.h"


// Initialisation
static PhysicsState self_state = {0};

void random_physics_state(void)
{
    self_state.x_mm = esp_random() % WORLD_SIZE_MM;   
    self_state.y_mm = esp_random() % WORLD_SIZE_MM;   
    self_state.z_mm = esp_random() % WORLD_SIZE_MM;

    self_state.vx_mm_s = 0;
    self_state.vy_mm_s = 0;
    self_state.vz_mm_s = 0;

    self_state.yaw_cd = 0;
}

// Reflect boundary logic
static inline void reflect_boundary(int64_t *pos, int32_t *vel)
{
    while (1) {
        // left boundary (pos < 0)
        if (*pos < 0) {
            *pos = -*pos;     // reflect: x = -x
            *vel = -*vel;     // reverse velocity
        }
        // right boundary (pos > MAX)
        else if (*pos > WORLD_SIZE_MM) {
            *pos = 2 * WORLD_SIZE_MM  - *pos;  // reflect symmetrical
            *vel = -*vel;                      // reverse velocity
        }
        else {
            break;
        }
    }
}

void physics_integration_task(void *pv)
{
    ESP_LOGI("PHYSICS", "Physics Integration Task started, period = %d ms", PHYSICS_DT);
    const TickType_t period_ticks = pdMS_TO_TICKS(PHYSICS_DT);
    TickType_t last_wake = xTaskGetTickCount();
    
    while (1) 
    {
        ControlCommand cmd = {0};
        if (command_queue != NULL){
            if (xQueueReceive(command_queue, &cmd, 0) == pdTRUE){
                // Apply command
                self_state.vx_mm_s = cmd.vx;
                self_state.vy_mm_s = cmd.vy;
                self_state.vz_mm_s = cmd.vz;
                self_state.yaw_cd = cmd.yaw_rate;
            };
        } 

        // Position update
        int32_t dx_u = self_state.vx_mm_s * PHYSICS_DT / 1000;
        int32_t dy_u = self_state.vy_mm_s * PHYSICS_DT / 1000;
        int32_t dz_u = self_state.vz_mm_s * PHYSICS_DT / 1000;

        int64_t x_mm = self_state.x_mm + dx_u;
        int64_t y_mm = self_state.y_mm + dy_u;
        int64_t z_mm = self_state.z_mm + dz_u;

        // Boundary check
        reflect_boundary(&x_mm, &self_state.vx_mm_s);
        reflect_boundary(&y_mm, &self_state.vy_mm_s);
        reflect_boundary(&z_mm, &self_state.vz_mm_s);

        self_state.x_mm = (uint32_t)x_mm;
        self_state.y_mm = (uint32_t)y_mm;
        self_state.z_mm = (uint32_t)z_mm;

        // Overwrite state_queue
        xQueueOverwrite(state_queue, &self_state);

        TickType_t now_tick = xTaskGetTickCount();
        TickType_t expected_wake = last_wake + period_ticks;

        if (now_tick > expected_wake) {
            ESP_LOGW("PHYSICS",
                     "Overrun by %ld ms",
                     (now_tick - expected_wake) * portTICK_PERIOD_MS);
        }

        // Frequency 50Hz
       vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PHYSICS_DT));
    }
}
