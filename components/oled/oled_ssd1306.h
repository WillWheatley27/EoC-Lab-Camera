#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include "esp_err.h"

esp_err_t oled_ssd1306_init(void);
esp_err_t oled_ssd1306_display_text(const char *text);

#endif  // OLED_SSD1306_H
