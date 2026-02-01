#include "camera_ov2640.h"

void camera_ov2640_get_default_pins(camera_ov2640_pins_t *pins)
{
    if (!pins) {
        return;
    }

    *pins = (camera_ov2640_pins_t){
        .pin_d0 = 8,
        .pin_d1 = 9,
        .pin_d2 = 10,
        .pin_d3 = 11,
        .pin_d4 = 12,
        .pin_d5 = 13,
        .pin_d6 = 14,
        .pin_d7 = 17,
        .pin_pclk = 18,
        .pin_vsync = 21,
        .pin_href = 46,
        .pin_xclk = 3,
        .pin_sda = 41,
        .pin_scl = 42,
        .pin_pwdn = 45,
        .pin_reset = 47,
    };
}
