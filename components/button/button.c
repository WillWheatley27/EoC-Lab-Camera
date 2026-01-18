#include "button.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define BUTTON_GPIO GPIO_NUM_1
#define DEBOUNCE_MS 30
#define LONG_PRESS_MS 500

static const char *TAG = "button";

static volatile bool s_paused = false;
static volatile bool s_recording = false;

static void s_button_task(void *arg)
{
    (void)arg;
    bool last_level = true;
    TickType_t press_tick = 0;

    while (true) {
        bool level = gpio_get_level(BUTTON_GPIO);
        if (level != last_level) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            level = gpio_get_level(BUTTON_GPIO);
            if (level != last_level) {
                last_level = level;
                if (!level) {
                    press_tick = xTaskGetTickCount();
                } else {
                    TickType_t held = xTaskGetTickCount() - press_tick;
                    if (held >= pdMS_TO_TICKS(LONG_PRESS_MS)) {
                        s_recording = !s_recording;
                        ESP_LOGI(TAG, "%s", s_recording ? "Recording started" : "Recording stopped");
                        if (!s_recording) {
                            s_paused = false;
                        }
                    } else {
                        if (s_recording) {
                            s_paused = !s_paused;
                            ESP_LOGI(TAG, "%s", s_paused ? "Paused" : "Resumed");
                        }
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    xTaskCreate(s_button_task, "button_task", 2048, NULL, 10, NULL);
}

bool button_is_paused(void)
{
    return s_paused;
}

bool button_is_recording(void)
{
    return s_recording;
}
