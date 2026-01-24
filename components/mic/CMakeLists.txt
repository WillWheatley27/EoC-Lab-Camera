idf_component_register(SRCS "mic_capture.c"
                      INCLUDE_DIRS "."
                      REQUIRES esp_driver_i2s button oled)
