#pragma once

#include "esp_err.h"

esp_err_t mic_capture_to_file(const char *path, int seconds);
