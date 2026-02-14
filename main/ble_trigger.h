#pragma once

#include <stddef.h>
#include <stdbool.h>

void ble_trigger_init(void);
bool ble_trigger_get_timestamp(char *out, size_t out_size);
