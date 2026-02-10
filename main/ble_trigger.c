#include "ble_trigger.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "button.h"

#if CONFIG_BT_NIMBLE_ENABLED
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"
#endif

// BLE trigger UUIDs (128-bit). Use distinct UUIDs for short vs long press.
#define BLE_TRIGGER_UUID_SHORT "00000000-0000-0000-0000-000000000001"
#define BLE_TRIGGER_UUID_LONG  "00000000-0000-0000-0000-000000000002"
#define BLE_TRIGGER_DEBOUNCE_MS 500

static const char *TAG = "ble_trigger";

#if CONFIG_BT_NIMBLE_ENABLED
static uint8_t s_ble_own_addr_type;
static ble_uuid128_t s_ble_short_uuid;
static ble_uuid128_t s_ble_long_uuid;
static int64_t s_ble_last_trigger_us = 0;

static int s_ble_hex_val(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    }
    return -1;
}

static bool s_ble_parse_uuid128_le(const char *uuid_str, uint8_t out_le[16])
{
    uint8_t be[16];
    int idx = 0;
    for (const char *p = uuid_str; *p != '\0'; ++p) {
        if (*p == '-') {
            continue;
        }
        int hi = s_ble_hex_val(*p);
        if (hi < 0) {
            return false;
        }
        ++p;
        if (*p == '\0') {
            return false;
        }
        int lo = s_ble_hex_val(*p);
        if (lo < 0) {
            return false;
        }
        if (idx >= 16) {
            return false;
        }
        be[idx++] = (uint8_t)((hi << 4) | lo);
    }
    if (idx != 16) {
        return false;
    }
    for (int i = 0; i < 16; ++i) {
        out_le[i] = be[15 - i];
    }
    return true;
}

static bool s_ble_uuid_match(const ble_uuid128_t *adv_uuid, const ble_uuid128_t *target)
{
    return ble_uuid_cmp((const ble_uuid_t *)adv_uuid, (const ble_uuid_t *)target) == 0;
}

static void s_ble_handle_trigger(bool is_long)
{
    int64_t now_us = esp_timer_get_time();
    if ((now_us - s_ble_last_trigger_us) < (BLE_TRIGGER_DEBOUNCE_MS * 1000LL)) {
        return;
    }
    s_ble_last_trigger_us = now_us;
    if (is_long) {
        button_trigger_long_press();
    } else {
        button_trigger_short_press();
    }
}

static int s_ble_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (event->type == BLE_GAP_EVENT_DISC) {
        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) != 0) {
            return 0;
        }
        bool found_short = false;
        bool found_long = false;
        for (int i = 0; i < fields.num_uuids128; ++i) {
            if (s_ble_uuid_match(&fields.uuids128[i], &s_ble_short_uuid)) {
                found_short = true;
            }
            if (s_ble_uuid_match(&fields.uuids128[i], &s_ble_long_uuid)) {
                found_long = true;
            }
        }
        if (found_long) {
            s_ble_handle_trigger(true);
        } else if (found_short) {
            s_ble_handle_trigger(false);
        }
    }
    return 0;
}

static void s_ble_start_scan(void)
{
    struct ble_gap_disc_params params = {
        .itvl = 0x0010,
        .window = 0x0010,
        .filter_duplicates = 1,
        .passive = 1,
    };
    int rc = ble_gap_disc(s_ble_own_addr_type, BLE_HS_FOREVER, &params, s_ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "BLE scan start failed (%d)", rc);
    }
}

static void s_ble_on_sync(void)
{
    ble_hs_id_infer_auto(0, &s_ble_own_addr_type);
    s_ble_start_scan();
}

static void s_ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}
#endif

void ble_trigger_init(void)
{
#if CONFIG_BT_NIMBLE_ENABLED
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed (%s)", esp_err_to_name(err));
        return;
    }

    uint8_t short_le[16];
    uint8_t long_le[16];
    if (!s_ble_parse_uuid128_le(BLE_TRIGGER_UUID_SHORT, short_le) ||
        !s_ble_parse_uuid128_le(BLE_TRIGGER_UUID_LONG, long_le)) {
        ESP_LOGE(TAG, "Invalid BLE trigger UUID format");
        return;
    }
    s_ble_short_uuid.u.type = BLE_UUID_TYPE_128;
    memcpy(s_ble_short_uuid.value, short_le, sizeof(short_le));
    s_ble_long_uuid.u.type = BLE_UUID_TYPE_128;
    memcpy(s_ble_long_uuid.value, long_le, sizeof(long_le));

    err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE init failed (%s)", esp_err_to_name(err));
        return;
    }
    ble_hs_cfg.sync_cb = s_ble_on_sync;
    nimble_port_freertos_init(s_ble_host_task);
    ESP_LOGI(TAG, "BLE trigger scan started");
#else
    ESP_LOGW(TAG, "BLE trigger disabled (CONFIG_BT_NIMBLE_ENABLED=0)");
#endif
}
