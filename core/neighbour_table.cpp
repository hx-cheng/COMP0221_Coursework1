extern "C" {
#include "config.h"
#include "compute_cmac.h"
#include "physics_integration.h"   

#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_mac.h"

#include <math.h>
#include <sys/time.h>
}

#include <cstring>
#include <string>

#include "RadioLib.h"
#include "EspHal.h"

Neighbour_table NeighbourState[MAX_NEIGHBOURS];
uint32_t neighbour_last_seen[MAX_NEIGHBOURS];   // ms timestamp

float neighbour_last_rssi[MAX_NEIGHBOURS];
float neighbour_last_snr[MAX_NEIGHBOURS];

float neighbour_latency_ms[MAX_NEIGHBOURS];
float neighbour_jitter_ms[MAX_NEIGHBOURS];

uint32_t neighbour_last_seq[MAX_NEIGHBOURS];   // last seq
uint32_t neighbour_lost_count[MAX_NEIGHBOURS]; // lost
uint32_t neighbour_recv_count[MAX_NEIGHBOURS]; // receive

volatile uint32_t self_tx_seq = 0;

uint8_t mac_addr[6];
void init_mac()
{
    esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);
}

Neighbour_table physics_state_integration(const PhysicsState *self_state, uint16_t seq_number)
{
    Neighbour_table lora_pkt;
    memset(&lora_pkt, 0, sizeof(lora_pkt));

    lora_pkt.version = 1;
    lora_pkt.team_id = TEAM_ID;
    memcpy(lora_pkt.node_id, mac_addr, 6);    
    lora_pkt.seq_number = seq_number;

    struct timeval tv;
    gettimeofday(&tv, NULL);

    lora_pkt.ts_s  = (uint32_t)tv.tv_sec;
    lora_pkt.ts_ms = (uint16_t)(tv.tv_usec / 1000);

    // ---- copy from PhysicsState ----
    lora_pkt.x_mm = self_state->x_mm;
    lora_pkt.y_mm = self_state->y_mm;
    lora_pkt.z_mm = self_state->z_mm;

    lora_pkt.vx_mm_s = self_state->vx_mm_s;
    lora_pkt.vy_mm_s = self_state->vy_mm_s;
    lora_pkt.vz_mm_s = self_state->vz_mm_s;

    lora_pkt.yaw_cd = self_state->yaw_cd;

    return lora_pkt;
}

// -------- Pin mapping for TTGO LoRa32 v2.1 --------
// SPI bus
static constexpr int PIN_SPI_SCK   = 5;
static constexpr int PIN_SPI_MISO  = 19;
static constexpr int PIN_SPI_MOSI  = 27;
// LoRa (SX1276)
static constexpr int PIN_LORA_CS   = 18;   // NSS
static constexpr int PIN_LORA_RST  = 23;   // RESET
static constexpr int PIN_LORA_DIO0 = 26;   // IRQ
static constexpr int PIN_LORA_DIO1 = 33;   // DIO1

// -------- Radio parameters (UK/EU defaults) --------
static constexpr float    LORA_FREQ_MHZ  = 868.1;  // MHz
static constexpr float    LORA_BW_KHZ    = 250.0;  // kHz
static constexpr uint8_t  LORA_SF        = 9;      // spreading factor 7..12
static constexpr uint8_t  LORA_CR        = 7;      // coding rate denominator 5..8 -> pass 5..8 or use setCodingRate(5..8) in some contexts
// static constexpr uint8_t  LORA_SYNCWORD  = 0x12;   // LoRaWAN public sync word, ok for simple P2P too
static constexpr uint8_t  LORA_SYNCWORD  = 0x46;   // LoRaWAN public sync word, ok for simple P2P too
static constexpr uint16_t LORA_PREAMBLE  = 10;     // symbols
static constexpr int8_t   LORA_POWER_DBM = 14;     // respect local limits
static constexpr bool     LORA_CRC_ON    = true;

// Create HAL for ESP-IDF SPI and GPIO
static EspHal* hal = new EspHal(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

// Create RadioLib SX1276 instance using the HAL
SX1276 lora = new Module(hal, PIN_LORA_CS, PIN_LORA_DIO0, PIN_LORA_RST, PIN_LORA_DIO1);

static const char *TAG = "NEIGHBOUR";

extern "C" bool init_radio()
{
    ESP_LOGI("RADIO", "Initialising RadioLib (SX1276)...");
    // begin(freq MHz, bw kHz, sf, cr, syncWord, preambleLen, power dBm, crcOn)
    int state = lora.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_SYNCWORD, LORA_PREAMBLE, LORA_POWER_DBM, LORA_CRC_ON);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE("RADIO", "lora.begin failed, code %d", state);
        return false;
    }

    // Respect power limits
    lora.setOutputPower(LORA_POWER_DBM);
  
    ESP_LOGI("RADIO", "Radio ready at %.1f MHz, BW %.0f kHz, SF %u, CR %u",
            LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR);
    return true;
}

extern "C" int lora_send_packet(uint8_t *data, size_t len)
{
    int state = lora.transmit(data, len);

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGW("RADIO", "LoRa TX failed, code %d", state);
        return -1;
    }
    return state;
}

int lora_send_neighbour_packet(const Neighbour_table *pkt)
{
    uint8_t wire[128];

    Neighbour_table pkt_copy = *pkt;
    lora_cmac_sign_small(&pkt_copy);
    xQueueSend(packet_queue, &pkt_copy, 0);
    
    size_t n = neighbour_to_wire(pkt, wire);

    lora_cmac_sign(wire, n);
    n += 4;

    return lora_send_packet(wire, n);
}

extern "C" int lora_receive_packet(uint8_t *buf, size_t buf_len)
{
    int state = lora.receive(buf, buf_len);
    if (state == RADIOLIB_ERR_NONE) {
        return lora.getPacketLength();
    }
    return -1;
}

void neighbor_table_update(const Neighbour_table *pkt)
{
    static float last_latency[MAX_NEIGHBOURS] = {0};
    
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t now = (uint64_t)(tv.tv_sec * 1000ULL + tv.tv_usec / 1000ULL);
    
    int free_index = -1;

    // ---- Compute latency ----
    uint64_t tx_time = pkt->ts_s * 1000ULL + pkt->ts_ms;
    float latency = now - tx_time;

    // Search for match or free slot
    for (int i = 0; i < MAX_NEIGHBOURS; i++) {

        // Found existing --> overwrite
        if (memcmp(NeighbourState[i].node_id, pkt->node_id, 6) == 0) {
            
            uint16_t prev = neighbour_last_seq[i];
            uint16_t curr = pkt->seq_number;

            if (curr <= prev) {
                ESP_LOGW("RADIO",
                    "Out-of-order/replay packet ignored.");
                return;
            }

            // ---- Compute jitter ----
            neighbour_latency_ms[i] = latency;
            neighbour_jitter_ms[i] = fabsf(latency - last_latency[i]);
            last_latency[i] = latency;

            NeighbourState[i] = *pkt;
            neighbour_last_seen[i] = now;
            neighbour_last_rssi[i] = lora.getRSSI();
            neighbour_last_snr[i]  = lora.getSNR();

            // // ----- Packet loss calculation -----

            if (neighbour_recv_count[i] == 0) {
                // first packet — no loss
                neighbour_last_seq[i] = curr;
            } else {
                if (curr > prev) {
                    neighbour_lost_count[i] += (curr - prev - 1);
                } else if (curr < prev) {
                    // sequence number wrap around (uint16_t)
                    neighbour_lost_count[i] += (65535 - prev + curr);
                }
                    neighbour_last_seq[i] = curr;
            }

            neighbour_recv_count[i]++;
            return;
        }

        // Remember first empty slot
        if ((memcmp(NeighbourState[i].node_id, "\0\0\0\0\0\0", 6) == 0) && free_index == -1) {
            free_index = i;
        }
    }

    // Insert at first empty slot
    if (free_index >= 0) {
        memcpy(&NeighbourState[free_index], pkt, sizeof(Neighbour_table));
        neighbour_last_seen[free_index] = now;
        
        // compute latency/jitter for first packet
        neighbour_latency_ms[free_index] = latency;
        neighbour_jitter_ms[free_index]  = 0;

        neighbour_last_rssi[free_index] = lora.getRSSI();
        neighbour_last_snr[free_index]  = lora.getSNR();

        neighbour_last_seq[free_index] = pkt->seq_number;
        neighbour_recv_count[free_index] = 1;
        neighbour_lost_count[free_index] = 0;
        ESP_LOGI(TAG, "Added new neighbour at index %d", free_index);
        return;
    }

    // Table full → replace oldest entry
    int oldest = 0;
    uint32_t oldest_time = neighbour_last_seen[0];

    for (int i = 1; i < MAX_NEIGHBOURS; i++) {
        if (neighbour_last_seen[i] < oldest_time) {
            oldest = i;
            oldest_time = neighbour_last_seen[i];
        }
    }

    memcpy(&NeighbourState[oldest], pkt, sizeof(Neighbour_table));
    neighbour_last_seen[oldest] = now;
    
    neighbour_latency_ms[oldest] = latency;
    neighbour_jitter_ms[oldest]  = 0;
    last_latency[oldest]         = latency;

    neighbour_last_rssi[oldest] = lora.getRSSI();
    neighbour_last_snr[oldest]  = lora.getSNR();

    ESP_LOGW(TAG, "Table full → replaced neighbour at index %d", oldest);
}

void neighbor_table_prune()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint32_t now = (uint32_t)(tv.tv_sec * 1000 + tv.tv_usec / 1000);

    for (int i = 0; i < MAX_NEIGHBOURS; i++) {
        if (now - neighbour_last_seen[i] > TIMEOUT_MS) {

            memset(&NeighbourState[i], 0, sizeof(Neighbour_table));
            neighbour_last_rssi[i] = 0;
            neighbour_last_snr[i] = 0;
            neighbour_latency_ms[i]  = 0;
            neighbour_jitter_ms[i]   = 0;
        }
    }
}

extern "C" void radio_task(void *pv)
{
    ESP_LOGI("RADIO", "Radio I/O Task started, period = %d ms", RADIO_DT);
    const TickType_t period_ticks = pdMS_TO_TICKS(PHYSICS_DT);
    TickType_t last_wake = xTaskGetTickCount();

    PhysicsState self_state; 
    Neighbour_table tx_pkt;
    Neighbour_table rx_pkt;

    uint8_t rx_buffer[128];
    uint16_t seq = 0;
    
    while (1)
    {
        // Get last self PhysicsState
        xQueuePeek(state_queue, &self_state, portMAX_DELAY);

        // Generate lora packets
        tx_pkt = physics_state_integration(&self_state, seq++);
        self_tx_seq = tx_pkt.seq_number;
        
        // Broadcast local state
        lora_send_neighbour_packet(&tx_pkt);
        vTaskDelay(1);

        // Receive neighbour states
        int len = lora_receive_packet(rx_buffer, sizeof(rx_buffer));
        vTaskDelay(1);

        if (len > 0)
        {   
            if (wire_to_neighbour(rx_buffer, len, &rx_pkt) != 0) {
                ESP_LOGW(TAG, "Decode failed.");
                continue;
            }

            if (lora_cmac_verify(rx_buffer, len) != 0) {
                ESP_LOGW(TAG, "CMAC verify failed.");
                continue;
            }

            if (memcmp(rx_pkt.node_id, mac_addr, 6) == 0) {
                ESP_LOGD("RADIO", "Ignored self packet");
                continue;
            }

            if (memcmp(rx_pkt.node_id, mac_addr, 6) == 0) {
                ESP_LOGD("RADIO", "Ignored self packet");
                continue;
            }

            // Verify received lora packet
            if (rx_pkt.team_id == TEAM_ID)
            {
                // Update neighbour tables
                neighbor_table_update(&rx_pkt);               
            }
        }

        // Prune expired neighbours
        neighbor_table_prune();

        TickType_t now_tick = xTaskGetTickCount();
        TickType_t expected_wake = last_wake + period_ticks;

        if (now_tick > expected_wake) {
            ESP_LOGW("RADIO",
                     "Overrun by %ld ms",
                     (now_tick - expected_wake) * portTICK_PERIOD_MS);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(RADIO_DT));
    }
}