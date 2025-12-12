#pragma once

#include <stdint.h>

#define WIRE_PAYLOAD_LEN 42
#define WIRE_TOTAL_LEN (WIRE_PAYLOAD_LEN + 4)

// --- Task frequency ---
#define PHYSICS_TASK_FREQ   50
#define FLOCKING_TASK_FREQ  10
#define LORA_TASK_FREQ      2
#define TELEMETRY_TASK_FREQ 2

// --- Physics Integration Parameters ---
#define WORLD_SIZE_MM   100000  // Maximum location value 100m
#define MAX_YAW         36000   // Maximum heading degrees 360

typedef struct {
    uint32_t x_mm;
    uint32_t y_mm;
    uint32_t z_mm;

    int32_t vx_mm_s;
    int32_t vy_mm_s;
    int32_t vz_mm_s;

    uint16_t yaw_cd;  // 0–35999
} PhysicsState;

// --- FLocking Control Parameters ---
#define ALIGNMENT_WEIGHT    0.08f
#define COHESION_WEIGHT     0.10f
#define SEPARATION_WEIGHT   1.00f

#define MIN_SEPARATION_MM     5000       // 5m
#define PERCEPTION_RADIUS_MM  3000       // 3m

#define MAX_SPEED_MMS      500        // mm/s

typedef struct {
    int32_t vx;        // desired velocity x (mm/s)
    int32_t vy;        // desired velocity y (mm/s)
    int32_t vz;        // desired velocity z (mm/s)
    uint16_t yaw_rate; // desired yaw rate (cd/s)
} ControlCommand;

// --- Radio I/O Parameters ---
#define MAX_NEIGHBOURS  5
#define TEAM_ID         18
#define TIMEOUT_MS      20000

typedef struct __attribute__((packed)) {

    uint8_t  version;      // protocol version
    uint8_t  team_id;      // team id (0 = everyone)
    uint8_t  node_id[6];   // 48-bit MAC address of the node

    uint16_t seq_number;   // sequence number

    uint32_t ts_s;         // Unix timestamp (seconds, NTP synced)
    uint16_t ts_ms;        // millisecond remainder (0..999)

    uint32_t  x_mm;        // x position in millimetres
    uint32_t  y_mm;        // y position in millimetres
    uint32_t  z_mm;        // z position in millimetres

    int32_t  vx_mm_s;      // x velocity in mm/s
    int32_t  vy_mm_s;      // y velocity in mm/s
    int32_t  vz_mm_s;      // z velocity in mm/s

    uint16_t yaw_cd;       // heading in centi-degrees (0..35999)

    uint8_t  mac_tag[4];   // truncated 4-byte MAC tag (security)

} Neighbour_table;

// --- Telemetry Parameters ---
#define SYS_TIME_SYNCED_BIT   (1 << 0)