#include "neighbour_table.h"
#include "telemetry.h"
#include "config.h"

#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_eap_client.h"
#include "lwip/inet.h"
#include "esp_sntp.h"

#include "mqtt_client.h"     // NEW

static const char *TAG = "TELEMETRY";

// --- USER SETTINGS (UCL-style) ---
#define EDUROAM_SSID     "eduroam"
#define EDUROAM_IDENTITY "@ucl.ac.uk"       //personal email address
#define EDUROAM_USERNAME "@ucl.ac.uk"       //personal email address
#define EDUROAM_PASSWORD "password"         //password
// ---------------------------------

// Note for testing:
//   Default: engf0001.cs.ucl.ac.uk:1883  (UCL broker)
//   This is only visible inside UCL
// 
//   Alternative for off-campus testing:
//     broker.hivemq.com:1883 and other free brokers

//#define MQTT_BROKER "mqtt://engf0001.cs.ucl.ac.uk:1883"
#define MQTT_BROKER "mqtt://broker.hivemq.com:1883"

static EventGroupHandle_t s_ev;
#define WIFI_CONNECTED_BIT BIT0

// ----------------------------------------------------------
// Wi-Fi event handler
// ----------------------------------------------------------
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Disconnected - retrying");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_ev, WIFI_CONNECTED_BIT);
    }
}

// ----------------------------------------------------------
// Connect to eduroam
// ----------------------------------------------------------
void wifi_connect_eduroam(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t cfg = {0};
    strncpy((char *)cfg.sta.ssid, EDUROAM_SSID, sizeof(cfg.sta.ssid));
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_ENTERPRISE;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));

    // PEAP / MSCHAPv2 credentials
    ESP_ERROR_CHECK(esp_eap_client_set_identity((const uint8_t *)EDUROAM_IDENTITY, strlen(EDUROAM_IDENTITY)));
    ESP_ERROR_CHECK(esp_eap_client_set_username((const uint8_t *)EDUROAM_USERNAME, strlen(EDUROAM_USERNAME)));
    ESP_ERROR_CHECK(esp_eap_client_set_password((const uint8_t *)EDUROAM_PASSWORD, strlen(EDUROAM_PASSWORD)));

    ESP_ERROR_CHECK(esp_wifi_sta_enterprise_enable());
    ESP_ERROR_CHECK(esp_wifi_start());

    s_ev = xEventGroupCreate();
    EventBits_t bits = xEventGroupWaitBits(s_ev, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "Timed out waiting for IP");
    }
}

// ----------------------------------------------------------
// MQTT event handler
// ----------------------------------------------------------
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;

    switch (event->event_id) {

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        // Subscribe to a test topic
        esp_mqtt_client_subscribe(client, "COMP0221/test", 1);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT message on topic: %.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG, "Payload: %.*s", event->data_len, event->data);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;

    default:
        break;
    }
    return ESP_OK;
}

// Wrapper required by esp_mqtt_client_register_event
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    mqtt_event_handler_cb(event_data);
}

// ----------------------------------------------------------
// Start MQTT client
// ----------------------------------------------------------
static esp_mqtt_client_handle_t start_mqtt(void)
{
    const esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    return client;
}

void init_sntp(void)
{
    ESP_LOGI("TIME", "Initializing SNTP");

    // Set SNTP mode
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);

    // Set NTP server
    esp_sntp_setservername(0, "pool.ntp.org");

    // Start SNTP
    esp_sntp_init();
}

void wait_for_time_sync(void)
{
    time_t now = 0;
    struct tm timeinfo = { 0 };

    int retry = 0;
    const int retry_count = 12;

    while (timeinfo.tm_year < (2024 - 1900) && ++retry < retry_count) {
        ESP_LOGI("TIME", "Waiting for NTP sync... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(1000));

        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (retry == retry_count)
        ESP_LOGW("TIME", "NTP sync failed (using default time)");
    else
        ESP_LOGI("TIME", "Time synced: %s", asctime(&timeinfo));
}

// ----------------------------------------------------------
// Build JSON message
// ----------------------------------------------------------
void build_json(const Neighbour_table *p, char *buf, size_t buf_size)
{
    char node_id_str[20];
    snprintf(node_id_str, sizeof(node_id_str),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             p->node_id[0], p->node_id[1], p->node_id[2],
             p->node_id[3], p->node_id[4], p->node_id[5]);

    char mac_tag_str[9];
    snprintf(mac_tag_str, sizeof(mac_tag_str),
             "%02X%02X%02X%02X",
             p->mac_tag[0], p->mac_tag[1], p->mac_tag[2], p->mac_tag[3]);

    snprintf(buf, buf_size,
        "{"
        "\"version\":%u,"
        "\"team_id\":%u,"
        "\"node_id\":\"%s\","
        "\"seq_number\":%u,"
        "\"ts_s\":%lu,"
        "\"ts_ms\":%u,"
        "\"x_mm\":%lu,"
        "\"y_mm\":%lu,"
        "\"z_mm\":%lu,"
        "\"vx_mm_s\":%ld,"
        "\"vy_mm_s\":%ld,"
        "\"vz_mm_s\":%ld,"
        "\"yaw_cd\":%u,"
        "\"mac_tag\":\"%s\""
        "}",
        p->version,
        p->team_id,
        node_id_str,
        p->seq_number,
        p->ts_s,
        p->ts_ms,
        p->x_mm,
        p->y_mm,
        p->z_mm,
        p->vx_mm_s,
        p->vy_mm_s,
        p->vz_mm_s,
        p->yaw_cd,
        mac_tag_str
    );
}

static uint16_t read_u16(const uint8_t *p) {
    return (p[0] << 8) | p[1];
}

static uint32_t read_u32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |
            (uint32_t)p[3];
}
void build_json_from_raw(const uint8_t *raw, size_t len,
                        char *json, size_t json_size)
{
    if (len < WIRE_TOTAL_LEN) {
        snprintf(json, json_size, "{\"error\":\"len_short\"}");
    }

    int idx = 0;

    uint8_t version  = raw[idx++];
    uint8_t team_id  = raw[idx++];

    char node_id_str[18];
    snprintf(node_id_str, sizeof(node_id_str),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             raw[idx], raw[idx+1], raw[idx+2],
             raw[idx+3], raw[idx+4], raw[idx+5]);
    idx += 6;

    uint16_t seq_number = read_u16(raw + idx); idx += 2;
    uint32_t ts_s       = read_u32(raw + idx); idx += 4;
    uint16_t ts_ms      = read_u16(raw + idx); idx += 2;

    uint32_t x_mm = read_u32(raw + idx); idx += 4;
    uint32_t y_mm = read_u32(raw + idx); idx += 4;
    uint32_t z_mm = read_u32(raw + idx); idx += 4;

    int32_t vx = (int32_t)read_u32(raw + idx); idx += 4;
    int32_t vy = (int32_t)read_u32(raw + idx); idx += 4;
    int32_t vz = (int32_t)read_u32(raw + idx); idx += 4;

    uint16_t yaw_cd = read_u16(raw + idx); idx += 2;

    char mac_tag_str[9];
    snprintf(mac_tag_str, sizeof(mac_tag_str),
             "%02X%02X%02X%02X",
             raw[idx], raw[idx+1], raw[idx+2], raw[idx+3]);

    // ---- Write JSON ----
    snprintf(json, json_size,
        "{"
        "\"version\":%u,"
        "\"team_id\":%u,"
        "\"node_id\":\"%s\","
        "\"seq_number\":%u,"
        "\"ts_s\":%lu,"
        "\"ts_ms\":%u,"
        "\"x_mm\":%lu,"
        "\"y_mm\":%lu,"
        "\"z_mm\":%lu,"
        "\"vx_mm_s\":%ld,"
        "\"vy_mm_s\":%ld,"
        "\"vz_mm_s\":%ld,"
        "\"yaw_cd\":%u,"
        "\"mac_tag\":\"%s\""
        "}",
        version,
        team_id,
        node_id_str,
        seq_number,
        ts_s,
        ts_ms,
        x_mm,
        y_mm,
        z_mm,
        vx,
        vy,
        vz,
        yaw_cd,
        mac_tag_str
    );
}

// ----------------------------------------------------------
// Telemetry_task: connect to eduroam, then connect to MQTT
// ----------------------------------------------------------
void telemetry_task(void *pv)
{
    ESP_LOGI(TAG, "Telemetry Task started, period = %d ms", TELEMETRY_DT);

    const TickType_t period_ticks = pdMS_TO_TICKS(PHYSICS_DT);
    TickType_t last_wake = xTaskGetTickCount();

    // Start MQTT once online
    esp_mqtt_client_handle_t client = start_mqtt();

    char json_buf[256];
    Neighbour_table pkt;
    //const char *topic = "COMP0221/flock/0/state";
    const char *topic = "flocksim";

    // Self state publisher
    while (1) {
        if (xQueueReceive(packet_queue, &pkt, 0) == pdTRUE)
        {
            //build_json_from_raw(msg.raw, msg.len, json_buf, sizeof(json_buf));
            build_json(&pkt, json_buf, sizeof(json_buf));
            esp_mqtt_client_publish(client, topic, json_buf, 0, 1, 0);
        }
        else
        {
            //ESP_LOGW(TAG, "MQTT not ready yet");
        }
        
        TickType_t now_tick = xTaskGetTickCount();
        TickType_t expected_wake = last_wake + period_ticks;

        if (now_tick > expected_wake) {
            ESP_LOGW(TAG,
                     "Overrun by %ld ms",
                     (now_tick - expected_wake) * portTICK_PERIOD_MS);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_DT));
    }
}

