#include "neighbour_table.h"
#include "compute_cmac.h"
#include "config.h"

#include "mbedtls/cipher.h"
#include "mbedtls/cmac.h"

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

static const uint8_t TEAM_KEY[16] = {};

// ---- Write big-endian integers ----
static inline void put_u16(uint8_t *buf, uint16_t v) {
    buf[0] = (v >> 8) & 0xFF;
    buf[1] = v & 0xFF;
}

static inline void put_u32(uint8_t *buf, uint32_t v) {
    buf[0] = (v >> 24) & 0xFF;
    buf[1] = (v >> 16) & 0xFF;
    buf[2] = (v >> 8)  & 0xFF;
    buf[3] = v & 0xFF;
}

static inline uint32_t get_u32(const uint8_t *buf) {
    return ((uint32_t)buf[0] << 24) |
           ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] <<  8) |
            (uint32_t)buf[3];
}

static inline uint16_t get_u16(const uint8_t *buf) {
    return ((uint16_t)buf[0] << 8) | buf[1];
}

size_t neighbour_to_wire(const Neighbour_table *n, uint8_t *out)
{
    uint8_t *p = out;

    *p++ = n->version;
    *p++ = n->team_id;
    memcpy(p, n->node_id, 6);  p += 6;

    put_u16(p, n->seq_number); p += 2;

    put_u32(p, n->ts_s);  p += 4;
    put_u16(p, n->ts_ms); p += 2;

    put_u32(p, n->x_mm);  p += 4;
    put_u32(p, n->y_mm);  p += 4;
    put_u32(p, n->z_mm);  p += 4;

    put_u32(p, n->vx_mm_s); p += 4;
    put_u32(p, n->vy_mm_s); p += 4;
    put_u32(p, n->vz_mm_s); p += 4;

    put_u16(p, n->yaw_cd); p += 2;

    // last 4 bytes are mac_tag (caller fills)
    return (size_t)(p - out);
}

// ---------------------------------------------------------------------
// Decode wire buffer → Neighbour_table struct
// ---------------------------------------------------------------------
int wire_to_neighbour(const uint8_t *buf, size_t len, Neighbour_table *n)
{
    if (len != WIRE_TOTAL_LEN) return -1;

    const uint8_t *p = buf;
    n->version = *p++;
    n->team_id = *p++;

    memcpy(n->node_id, p, 6);  p += 6;

    n->seq_number = get_u16(p); p += 2;

    n->ts_s  = get_u32(p); p += 4;
    n->ts_ms = get_u16(p); p += 2;

    n->x_mm = get_u32(p); p += 4;
    n->y_mm = get_u32(p); p += 4;
    n->z_mm = get_u32(p); p += 4;

    n->vx_mm_s = (int32_t)get_u32(p); p += 4;
    n->vy_mm_s = (int32_t)get_u32(p); p += 4;
    n->vz_mm_s = (int32_t)get_u32(p); p += 4;

    n->yaw_cd = get_u16(p); p += 2;

    memcpy(n->mac_tag, p, 4);

    return 0;
}

// Constant-time memcmp: returns 0 if equal, non-zero otherwise
static int const_memcmp(const uint8_t *a, const uint8_t *b, size_t n)
{
    uint8_t diff = 0;
    for (size_t i = 0; i < n; i++) diff |= (a[i] ^ b[i]);
    return diff;
}

// Compute AES-128 CMAC(tag16) over neighbour table
static int cmac16(const uint8_t *m, size_t n, uint8_t tag16[16]) {
    const mbedtls_cipher_info_t *info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
    if (!info) return -1;
    
    mbedtls_cipher_context_t ctx;
    mbedtls_cipher_init(&ctx);
    
    int r = mbedtls_cipher_setup(&ctx, info);
    if (r == 0) r = mbedtls_cipher_cmac_starts(&ctx, TEAM_KEY, 128);
    if (r == 0) r = mbedtls_cipher_cmac_update(&ctx, m, n);
    if (r == 0) r = mbedtls_cipher_cmac_finish(&ctx, tag16);
    
    mbedtls_cipher_free(&ctx);
    return r;
}

// Truncate last 4 bytes of a 16-byte CMAC
static inline void cmac_trunc32(const uint8_t tag16[16], uint8_t out4[4]) {
    out4[0] = tag16[12];
    out4[1] = tag16[13];
    out4[2] = tag16[14];
    out4[3] = tag16[15];
}

// Compute CMAC and sign to lora packets
void lora_cmac_sign_small(Neighbour_table *neighbours)
{
    uint8_t full[16];

    size_t len_without_mac = sizeof(Neighbour_table) - sizeof(neighbours->mac_tag);

    if (cmac16((const uint8_t *)neighbours, len_without_mac, full) != 0) {
        memset(neighbours->mac_tag, 0, sizeof(neighbours->mac_tag));
        printf("CMAC error: signing failed\n");
        return;
    }

    cmac_trunc32(full, neighbours->mac_tag);
}

void lora_cmac_sign(uint8_t *wire, size_t len_without_tag)
{
    uint8_t tag[16];
    if (cmac16(wire, len_without_tag, tag) != 0) {
        memset(wire + len_without_tag, 0, 4);
        printf("CMAC error: signing failed\n");
        return;
    }
    cmac_trunc32(tag, wire + len_without_tag);
}

// Verify 4-byte truncated CMAC for neighbour table in constant time
int lora_cmac_verify(const uint8_t *wire, size_t total_len)
{
    if (total_len < 4) return -1;

    size_t msg_len = total_len - 4;
    const uint8_t *recv_tag = wire + msg_len;

    uint8_t full[16], calc4[4];
    if (cmac16(wire, msg_len, full) != 0) return -1;

    cmac_trunc32(full, calc4);

    return const_memcmp(calc4, recv_tag, 4);
}