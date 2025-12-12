#pragma once

#include "neighbour_table.h"
#include <stdint.h>

size_t neighbour_to_wire(const Neighbour_table *n, uint8_t *out);
int wire_to_neighbour(const uint8_t *buf, size_t len, Neighbour_table *n);

void lora_cmac_sign_small(Neighbour_table *neighbours);
void lora_cmac_sign(uint8_t *wire, size_t len_without_tag);
int lora_cmac_verify(const uint8_t *wire, size_t total_len);