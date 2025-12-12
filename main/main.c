#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_mac.h" 
#include "nvs.h"
#include "nvs_flash.h"

#include "config.h"
#include "neighbour_table.h"
#include "physics_integration.h"
#include "flocking_control.h"
#include "telemetry.h"
#include "compute_cmac.h"
#include "monitor_task.h"

static const char *TAG = "MAIN";

QueueHandle_t state_queue = NULL;
QueueHandle_t command_queue = NULL;
QueueHandle_t packet_queue = NULL;

static void create_tasks(void)
{
    // Physics integration task (50 Hz)
    xTaskCreate(
        physics_integration_task,
        "physics_task",
        4096,
        NULL,
        5,          
        NULL
    );

    // Flocking control task (10 Hz)
    xTaskCreate(
        flocking_control_task,
        "flocking_task",
        4096,
        NULL,
        4,
        NULL
    );

    // Radio I/O task (2 Hz)
    xTaskCreate(
        radio_task,
        "radio_task",
        8192,
        NULL,
        3,
        NULL
    );

    // Telemetry task (2 Hz)
    xTaskCreate(
        telemetry_task,
        "telemetry_task",
        8192,
        NULL,
        3,
        NULL
    );

    xTaskCreate(
        monitor_task,
        "monitor",
        4096,
        NULL,
        1,
        NULL
    );
}

void init_neighbour_tables()
{
    memset(NeighbourState, 0, sizeof(NeighbourState));
    memset(neighbour_last_seen, 0, sizeof(neighbour_last_seen));
    memset(neighbour_last_rssi, 0, sizeof(neighbour_last_rssi));
    memset(neighbour_last_snr, 0, sizeof(neighbour_last_snr));

    memset(neighbour_latency_ms, 0, sizeof(neighbour_latency_ms));
    memset(neighbour_jitter_ms, 0, sizeof(neighbour_jitter_ms));
    
    memset(neighbour_last_seq, 0, sizeof(neighbour_last_seq));
    memset(neighbour_lost_count, 0, sizeof(neighbour_lost_count));
    memset(neighbour_recv_count, 0, sizeof(neighbour_recv_count));
}

void app_main(void)
{
    ESP_LOGI(TAG, "Swarm demo starting...");

    init_mac();

    // Create queues
    state_queue = xQueueCreate(1, sizeof(PhysicsState));
    if (!state_queue) {
        ESP_LOGE(TAG, "Failed to create state_queue");
        return;
    }

    command_queue = xQueueCreate(1, sizeof(ControlCommand));
    if (!command_queue) {
        ESP_LOGE(TAG, "Failed to create command_queue");
        return;
    }

    packet_queue = xQueueCreate(1, sizeof(Neighbour_table));
    if (!packet_queue) {
        ESP_LOGE(TAG, "Failed to create packet_queue");
        return;
    }

    // Initialise neighbour tables stuff
    init_neighbour_tables();
    
    // Connect WIFI and synchronise time
    // This needs to be here... 
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    wifi_connect_eduroam();
    init_sntp();
    wait_for_time_sync(); 

    // Initialise lora for radio task
    if (!init_radio()) {
        ESP_LOGE(TAG, "Radio init failed, halting.");
        vTaskDelay(portMAX_DELAY);
    }

    // Create FreeRTOS tasks
    random_physics_state();
    create_tasks();
}