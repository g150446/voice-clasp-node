#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t axp192_init(void);
void axp192_set_ldo2(bool on);
void axp192_set_mic_ldo(bool on);
void axp192_prepare_sleep(void);
void axp192_wake_rails(void);
int axp192_battery_millivolt(void);
bool axp192_is_charging(void);
bool axp192_vbus_present(void);
