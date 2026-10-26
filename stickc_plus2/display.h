#pragma once

#include "esp_err.h"

typedef enum {
    DISPLAY_STATUS_NOT_CONNECTED = 0,
    DISPLAY_STATUS_CONNECTED,
    DISPLAY_STATUS_RECORDING,
} display_status_t;

esp_err_t display_init(void);
void display_set_status(display_status_t status);
void display_sleep(void);

/* Paint 0xF800 / 0x07E0 / 0x001F / 0xFFFF as bands to re-measure the panel's
 * actual channel order (serial 'p'). */
void display_test_pattern(void);

/* Blank the panel and park its pads so nothing floats through deep sleep. */
void display_prepare_deep_sleep(void);

/* Undo display_prepare_deep_sleep() when a planned sleep was abandoned.
 * The caller must follow with display_set_status() to redraw and relight. */
esp_err_t display_resume(void);
