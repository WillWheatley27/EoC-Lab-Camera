#include "mic_capture.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "esp_log.h"
#include "button.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"

#define I2S_SAMPLE_RATE_HZ 16000
#define I2S_BCLK_IO        9
#define I2S_WS_IO          10
#define I2S_DIN_IO         11

static const char *TAG = "mic";

static void s_write_le16(FILE *f, uint16_t value)
{
    uint8_t b[2] = {value & 0xff, (value >> 8) & 0xff};
    fwrite(b, 1, sizeof(b), f);
}

static void s_write_le32(FILE *f, uint32_t value)
{
    uint8_t b[4] = {
        value & 0xff,
        (value >> 8) & 0xff,
        (value >> 16) & 0xff,
        (value >> 24) & 0xff
    };
    fwrite(b, 1, sizeof(b), f);
}

static bool s_has_wav_extension(const char *path)
{
    const char *dot = strrchr(path, '.');
    return (dot != NULL) && (strcmp(dot, ".wav") == 0);
}

static void s_write_wav_header(FILE *f, uint32_t sample_rate_hz, uint16_t bits_per_sample,
                               uint16_t channels, uint32_t data_bytes)
{
    const uint32_t byte_rate = sample_rate_hz * channels * (bits_per_sample / 8);
    const uint16_t block_align = channels * (bits_per_sample / 8);
    const uint32_t riff_size = 36 + data_bytes;

    fwrite("RIFF", 1, 4, f);
    s_write_le32(f, riff_size);
    fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f);
    s_write_le32(f, 16);
    s_write_le16(f, 1);
    s_write_le16(f, channels);
    s_write_le32(f, sample_rate_hz);
    s_write_le32(f, byte_rate);
    s_write_le16(f, block_align);
    s_write_le16(f, bits_per_sample);
    fwrite("data", 1, 4, f);
    s_write_le32(f, data_bytes);
}

esp_err_t mic_capture_to_file(const char *path, int seconds)
{
    esp_err_t ret;
    i2s_chan_handle_t rx_handle = NULL;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    ret = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel (%s)", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_DIN_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    ret = i2s_channel_init_std_mode(rx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S standard mode (%s)", esp_err_to_name(ret));
        i2s_del_channel(rx_handle);
        return ret;
    }

    ret = i2s_channel_enable(rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S channel (%s)", esp_err_to_name(ret));
        i2s_del_channel(rx_handle);
        return ret;
    }

    ESP_LOGI(TAG, "Waiting for long press to start recording");
    while (!button_is_recording()) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI(TAG, "Recording started");

    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open %s for audio capture (errno=%d)", path, errno);
        i2s_channel_disable(rx_handle);
        i2s_del_channel(rx_handle);
        return ESP_FAIL;
    }

    const bool write_wav = s_has_wav_extension(path);
    const bool stop_on_button = (seconds <= 0);
    if (!stop_on_button && seconds < 1) {
        seconds = 1;
    }

    const size_t bytes_per_sample = 4;
    const size_t samples_per_chunk = 512;
    const int flush_interval_ms = 1000;
    uint32_t next_flush_ms = flush_interval_ms;
    const size_t chunk_bytes = samples_per_chunk * bytes_per_sample;
    uint8_t *buffer = (uint8_t *)malloc(chunk_bytes);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate audio buffer");
        fclose(f);
        i2s_channel_disable(rx_handle);
        i2s_del_channel(rx_handle);
        return ESP_ERR_NO_MEM;
    }

    size_t total_samples = stop_on_button ? SIZE_MAX : (size_t)I2S_SAMPLE_RATE_HZ * (size_t)seconds;
    size_t captured_samples = 0;
    if (write_wav) {
        s_write_wav_header(f, I2S_SAMPLE_RATE_HZ, 32, 1, 0);
    }
    while (captured_samples < total_samples) {
        if (stop_on_button && !button_is_recording()) {
            ESP_LOGI(TAG, "Stop requested");
            break;
        }
        size_t bytes_read = 0;
        size_t samples_to_read = samples_per_chunk;
        if (captured_samples + samples_to_read > total_samples) {
            samples_to_read = total_samples - captured_samples;
        }
        size_t bytes_to_read = samples_to_read * bytes_per_sample;

        ret = i2s_channel_read(rx_handle, buffer, bytes_to_read, &bytes_read, pdMS_TO_TICKS(1000));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S read failed (%s)", esp_err_to_name(ret));
            break;
        }
        if (bytes_read > 0) {
            if (button_is_paused()) {
                memset(buffer, 0, bytes_read);
            }
            fwrite(buffer, 1, bytes_read, f);
            captured_samples += bytes_read / bytes_per_sample;
        }

        if (write_wav && (captured_samples * 1000 / I2S_SAMPLE_RATE_HZ) >= next_flush_ms) {
            const uint32_t data_bytes = (uint32_t)(captured_samples * bytes_per_sample);
            fflush(f);
            fsync(fileno(f));
            fseek(f, 0, SEEK_SET);
            s_write_wav_header(f, I2S_SAMPLE_RATE_HZ, 32, 1, data_bytes);
            fseek(f, 0, SEEK_END);
            next_flush_ms += flush_interval_ms;
        }
    }

    if (write_wav) {
        const uint32_t data_bytes = (uint32_t)(captured_samples * bytes_per_sample);
        fseek(f, 0, SEEK_SET);
        s_write_wav_header(f, I2S_SAMPLE_RATE_HZ, 32, 1, data_bytes);
    }

    free(buffer);
    fclose(f);
    i2s_channel_disable(rx_handle);
    i2s_del_channel(rx_handle);

    int captured_seconds = (int)(captured_samples / I2S_SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "Captured %d seconds of audio to %s", captured_seconds, path);
    return ret;
}
