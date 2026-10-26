#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "host/ble_gatt.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register SMP GATT service definitions into the app's service table builder. */
const struct ble_gatt_svc_def *smp_ota_svc_defs(void);

/** Call after NVS init; prepares OTA state and schedules auto-confirm. */
void smp_ota_init(void);

/** Stop recording/mic before heavy upload (optional hook from main). */
typedef void (*smp_ota_busy_cb_t)(bool busy);
void smp_ota_set_busy_callback(smp_ota_busy_cb_t cb);

#ifdef __cplusplus
}
#endif
