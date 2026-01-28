#pragma once

#include <stdbool.h>

void button_init(void);
bool button_is_paused(void);
bool button_is_recording(void);
void button_set_idle_display(const char *line1, const char *line2);
