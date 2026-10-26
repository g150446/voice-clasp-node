#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2s_pdm.h"
#include "driver/rtc_io.h"
#include "soc/i2s_struct.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "adpcm.h"
#include "display.h"
#include "smp_ota.h"

static const char *TAG = "hn_plus2";

/* M5StickC Plus2 pins (M5Unified / docs). */
#define POWER_HOLD_GPIO   4
#define PDM_CLK_GPIO      0
#define PDM_DIN_GPIO      34
#define BUTTON_A_GPIO     37
#define LED_GPIO          19

#define BUTTON_POLL_MS          10
#define BUTTON_DEBOUNCE_MS      30
#define BUTTON_DOUBLE_CLICK_MS  500
#define BUTTON_LONG_PRESS_MS    1000

#define I2S_PORT_NUM      I2S_NUM_0
#define I2S_SAMPLE_RATE   16000
#define DMA_BUFFER_COUNT  8
#define DMA_FRAME_COUNT   256
/* Sized from the sweep: loud continuous speech on the correct slot measured
 * raw rms 1157 with mean -902, i.e. ~725 after DC removal, so x4 lands normal
 * speech near the XIAO reference (output rms 560-692) without clipping.
 * Runtime-adjustable via serial 'g' so calibration does not need a reflash. */
#define MIC_GAIN_DEFAULT  4
#define MIC_START_DISCARD_MS  400
#define MIC_DC_SHIFT      5   /* offset IIR: offset += (x - offset) >> MIC_DC_SHIFT */
#define MIC_DC_SETTLE_THRESH  800
#define MIC_DC_SETTLE_SAMPLES 512
/* i2s_channel_read() takes MILLISECONDS and applies pdMS_TO_TICKS itself.
 * Wrapping it again gives 0 ticks at CONFIG_FREERTOS_HZ=100, i.e. a
 * non-blocking read that fails instantly with ESP_ERR_TIMEOUT. */
#define MIC_READ_TIMEOUT_MS   50
#define MIC_SWEEP_DISCARD_MS  200
#define MIC_SWEEP_MEASURE_MS  300
/* distinct-value counting is O(table) per sample; only sample the head. */
#define MIC_STATS_DISTINCT_CAP     256
#define MIC_STATS_DISTINCT_WINDOW  4096
#define MIC_STATS_DUMP_SAMPLES     32
/* Classification thresholds (16 kHz => Nyquist 8 kHz). */
#define MIC_ZCR_NOISE_HZ    2500.0   /* above this the stream is broadband noise */
#define MIC_ZCR_SIGNAL_MIN  100.0    /* speech sits in roughly 300-1500 Hz */

#define BLE_MTU_SIZE      512
#define BLE_ADV_NAME      "HarnessNode-Plus2"

#define RECORDING_PACKET_SIZE  200
#define AUDIO_PACKET_SYNC_BYTE 0xAA
#define EVENT_PACKET_SYNC_BYTE 0x55
#define EVENT_RECORDING_STARTED 0x01
#define EVENT_RECORDING_STOPPED 0x02
#define EVENT_DOUBLE_CLICK 0x12
#define EVENT_SINGLE_CLICK 0x14
#define COMMAND_SET_OPERATION_MODE 0x05
/* Voice Clasp asks for notify-only taps so the Android host authorizes
 * recording with RX 0x01/0x00. Default off keeps the Handy single-click toggle. */
#define COMMAND_SET_TAP_NOTIFY_ONLY 0x08
#define EVENT_OPERATION_MODE 0x40
#define OPERATION_MODE_NORMAL 0x00
#define OPERATION_MODE_DRIVING 0x01
#define OPERATION_MODE_PENDING_NONE 0xff

#define UART_PORT_NUM     UART_NUM_0
#define UART_BAUD_RATE    115200

/* HarnessNode UUIDs (LSB-first). */
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00);
static const ble_uuid128_t gatt_svr_tx_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00);
static const ble_uuid128_t gatt_svr_rx_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00);

static uint16_t audio_conn_handle = BLE_HS_CONN_HANDLE_NONE;
/* Streaming instrumentation: how much of the audio actually reaches the peer. */
static uint32_t tx_sent;
static uint32_t tx_failed;
static uint32_t tx_nomem;
static int64_t tx_started_us;
static esp_timer_handle_t conn_param_timer;
static uint16_t audio_tx_char_handle;
static uint16_t audio_rx_char_handle;

static adpcm_state_t adpcm_enc_state;
static uint8_t seq_num = 0;

static TaskHandle_t audio_task_handle;
static TaskHandle_t button_task_handle;

static volatile bool is_recording = false;
static volatile bool recording_requested = false;
static volatile bool stop_requested = false;
static volatile bool stop_event_pending = false;
static volatile uint8_t operation_mode = OPERATION_MODE_NORMAL;
static volatile uint8_t operation_mode_pending = OPERATION_MODE_PENDING_NONE;
/* Set by Voice Clasp: single-click only notifies, never toggles recording.
 * RAM only and cleared on disconnect so a later Handy session keeps its toggle. */
static volatile bool tap_notify_only = false;

/* Which code owns the PDM clock.
 *
 * MIC_CLK_IDF: esp_driver_i2s computes it. i2s_pdm.c forces bclk_div >= 8, so a
 *   16 kHz DSR_16S stream needs mclk = 2.048 MHz * 8 = 16.384 MHz fractionally
 *   divided from the 160 MHz PLL (ratio 9.7656).
 * MIC_CLK_M5: M5Unified's own path. Mic_Class.cpp sets sample_rate_hz to a dummy
 *   48000 and then programs the registers directly with bits=64, div_m=2, i.e.
 *   mclk = 4.096 MHz (ratio 39.0625), plus the div_y==0 workaround for the
 *   fractional-divider erratum. Same nominal 2.048 MHz PDM clock, different jitter. */
typedef enum {
    MIC_CLK_IDF = 0,
    MIC_CLK_M5,
} mic_clk_path_t;

typedef struct {
    mic_clk_path_t clk_path;
    i2s_pdm_dsr_t dn_sample_mode;   /* also drives rx_sinc_dsr_16_en on the M5 path */
    i2s_pdm_slot_mask_t slot_mask;
    i2s_slot_mode_t slot_mode;
} mic_pdm_cfg_t;

/* Measured on hardware with the serial 'm' sweep, not copied from M5Unified.
 *
 * M5Unified's mic_config_t defaults to input_only_right, but that pairs with
 * M5's own clock (div_m = 2). Under the IDF clock path (bclk_div >= 8) the WS
 * phase lands one slot over and the mic appears on LEFT instead: sweeping all
 * four combinations gave SIGNAL on IDF+LEFT and on M5-raw+RIGHT, while
 * IDF+RIGHT gave a constant -30935 with DSR_8S and white noise with DSR_16S.
 * 2.048 MHz (DSR_16S) over 1.024 MHz because SPM1423's normal mode starts at
 * 1.0 MHz and DSR_8S sits right on that edge. */
static const mic_pdm_cfg_t MIC_PDM_DEFAULT = {
    .clk_path = MIC_CLK_IDF,
    .dn_sample_mode = I2S_PDM_DSR_16S,
    .slot_mask = I2S_PDM_SLOT_LEFT,
    .slot_mode = I2S_SLOT_MODE_MONO,
};

static i2s_chan_handle_t i2s_rx_handle = NULL;
static bool mic_enabled = false;
static int32_t mic_dc_offset = 0;
static uint32_t mic_level_log_left = 0;
static volatile bool mic_sweep_requested = false;
static mic_pdm_cfg_t mic_pdm_cfg = MIC_PDM_DEFAULT;
static volatile int32_t mic_gain = MIC_GAIN_DEFAULT;
/* In stereo capture only one slot carries the mic; which one the sweep reveals. */
static uint8_t mic_stereo_pick = 0;

static void ble_app_advertise(void);
static void ble_app_on_sync(void);
static void ble_app_on_reset(int reason);
static int ble_app_gap_event(struct ble_gap_event *event, void *arg);
static void request_recording_stop(const char *source);
static void refresh_status_display(void);
static void log_conn_params(const char *when);
static void request_conn_params(bool fast);
static void deferred_idle_conn_params(void *arg);
static bool conn_interval_is_fast(void);
static bool wait_for_fast_conn_params(uint32_t timeout_ms);
static void on_ota_busy(bool busy);
static esp_err_t set_microphone_enabled(bool enabled);
static esp_err_t init_audio(const mic_pdm_cfg_t *cfg);
static esp_err_t apply_pdm_config(const mic_pdm_cfg_t *cfg);
static void apply_m5_mic_clock(const mic_pdm_cfg_t *cfg);
static void run_mic_config_sweep(int16_t *i2s_buffer, size_t i2s_buffer_bytes);
static void audio_stream_task(void *pvParameters);
static void button_task(void *pvParameters);
static void uart_task(void *pvParameters);
static void send_status_event(uint8_t event_code, const char *event_name);
static void send_operation_mode_status(void);
static void apply_pending_operation_mode(void);

static void power_hold_on(void)
{
    /*
     * Preload the output latch first: gpio_config() enables the driver before
     * anyone writes GPIO_OUT, so configuring first would drive HOLD low for a
     * few microseconds and cut the board's own power.
     */
    gpio_set_level(POWER_HOLD_GPIO, 1);

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << POWER_HOLD_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(POWER_HOLD_GPIO, 1);
}

static void led_set(bool on)
{
    gpio_set_level(LED_GPIO, on ? 1 : 0);
}

static void led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    led_set(true);
    vTaskDelay(pdMS_TO_TICKS(200));
    led_set(false);
}

static void request_recording_start(const char *source)
{
    if (is_recording || recording_requested) {
        return;
    }
    ESP_LOGI(TAG, "%s: Start recording requested", source);
    stop_requested = false;
    recording_requested = true;
    /* Need 7.5–15 ms before PCM; request immediately (not via 500 ms timer). */
    if (conn_param_timer != NULL) {
        (void)esp_timer_stop(conn_param_timer);
    }
    request_conn_params(true);
    refresh_status_display();
}

static void request_recording_stop(const char *source)
{
    if (!is_recording && !recording_requested) {
        return;
    }
    ESP_LOGI(TAG, "%s: Stop recording requested", source);
    bool was_recording = is_recording;
    recording_requested = false;
    is_recording = false;
    stop_requested = true;
    stop_event_pending = was_recording;
    led_set(false);
    /* Drop back to idle radio ASAP; safe even if stream task still winding down. */
    request_conn_params(false);
    refresh_status_display();
}

static void on_ota_busy(bool busy)
{
    if (busy) {
        request_recording_stop("OTA");
        ESP_LOGI(TAG, "OTA busy: recording blocked");
    }
}

static void emulate_single_click(const char *source)
{
    ESP_LOGI(TAG, "%s: Single-click", source);

    if (audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "%s: Single-click ignored: no BLE connection", source);
        return;
    }

    if (tap_notify_only) {
        /* Voice Clasp host owns tap semantics: it answers the tap with
         * RX 0x01/0x00 (or G2 paging) instead of a local toggle. */
        ESP_LOGI(TAG, "%s: notify-only (host authorizes recording)", source);
        send_status_event(EVENT_SINGLE_CLICK, "single_click");
        return;
    }

    if (is_recording) {
        request_recording_stop(source);
    } else if (recording_requested) {
        recording_requested = false;
        ESP_LOGI(TAG, "%s: Pending recording start cancelled", source);
        request_conn_params(false);
        apply_pending_operation_mode();
        refresh_status_display();
    } else {
        request_recording_start(source);
    }

    send_status_event(EVENT_SINGLE_CLICK, "single_click");
}

static void emulate_double_click(const char *source)
{
    ESP_LOGI(TAG, "%s: Double-click (notify only)", source);
    send_status_event(EVENT_DOUBLE_CLICK, "double_click");
}

/*
 * Fast params (7.5–15 ms): required while streaming 16 kHz mono PCM
 * (~160 notif/s). Without them the mbuf pool drains and audio is dropped
 * (see docs/stickc_plus2_guide.md). Idle params (50–100 ms): used whenever
 * connected but not recording to cut radio wakeups. Latency stays 0 so
 * START/STOP still arrive within one interval.
 *
 * Do not call ble_gap_update_params from inside a GAP callback; defer via
 * esp_timer (connect path) or call from the audio/button/uart tasks.
 */
#define CONN_ITVL_FAST_MIN       6    /* 7.5 ms  */
#define CONN_ITVL_FAST_MAX       12   /* 15 ms   */
#define CONN_ITVL_IDLE_MIN       40   /* 50 ms   */
#define CONN_ITVL_IDLE_MAX       80   /* 100 ms  */
#define CONN_SUPERVISION_TO      400  /* 4 s, 10 ms units */
#define CONN_FAST_GATE_TIMEOUT_MS 500

static void request_conn_params(bool fast)
{
    if (audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    struct ble_gap_upd_params p = {
        .itvl_min = fast ? CONN_ITVL_FAST_MIN : CONN_ITVL_IDLE_MIN,
        .itvl_max = fast ? CONN_ITVL_FAST_MAX : CONN_ITVL_IDLE_MAX,
        .latency = 0,
        .supervision_timeout = CONN_SUPERVISION_TO,
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    int rc = ble_gap_update_params(audio_conn_handle, &p);
    if (rc != 0) {
        ESP_LOGW(TAG, "conn param update (%s) failed rc=%d",
                 fast ? "fast" : "idle", rc);
    } else {
        /* itvl is 1.25 ms units → print as N.xx ms without float printf. */
        const unsigned min_us = (unsigned)p.itvl_min * 1250u;
        const unsigned max_us = (unsigned)p.itvl_max * 1250u;
        ESP_LOGI(TAG, "conn param update requested (%s: %u.%02u-%u.%02u ms)",
                 fast ? "fast" : "idle",
                 min_us / 1000u, (min_us % 1000u) / 10u,
                 max_us / 1000u, (max_us % 1000u) / 10u);
    }
}

/* Connect +500 ms: ask for idle unless a recording start already raced in. */
static void deferred_idle_conn_params(void *arg)
{
    (void)arg;
    if (audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    if (is_recording || recording_requested) {
        ESP_LOGI(TAG, "skip deferred idle conn params (recording)");
        return;
    }
    request_conn_params(false);
}

static bool conn_interval_is_fast(void)
{
    struct ble_gap_conn_desc desc;
    if (audio_conn_handle == BLE_HS_CONN_HANDLE_NONE ||
        ble_gap_conn_find(audio_conn_handle, &desc) != 0) {
        return false;
    }
    return desc.conn_itvl > 0 && desc.conn_itvl <= CONN_ITVL_FAST_MAX;
}

/* Wait until the link is fast enough for PCM, or timeout (host may refuse). */
static bool wait_for_fast_conn_params(uint32_t timeout_ms)
{
    if (conn_interval_is_fast()) {
        log_conn_params("fast already");
        return true;
    }
    request_conn_params(true);
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        if (stop_requested || !recording_requested) {
            return false;
        }
        if (audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            return false;
        }
        if (conn_interval_is_fast()) {
            log_conn_params("fast gate ok");
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    log_conn_params("fast gate timeout");
    ESP_LOGW(TAG, "fast conn params not confirmed within %u ms; streaming anyway",
             (unsigned)timeout_ms);
    return false;
}

static void log_conn_params(const char *when)
{
    struct ble_gap_conn_desc desc;
    if (audio_conn_handle == BLE_HS_CONN_HANDLE_NONE ||
        ble_gap_conn_find(audio_conn_handle, &desc) != 0) {
        return;
    }
    /* Who actually holds the link: we advertise only while disconnected, so a
     * stray central (a paired Mac, an old phone) locks Handy out entirely. */
    const uint8_t *a = desc.peer_id_addr.val;
    ESP_LOGI(TAG, "Peer (%s): %02x:%02x:%02x:%02x:%02x:%02x type=%u",
             when, a[5], a[4], a[3], a[2], a[1], a[0],
             (unsigned)desc.peer_id_addr.type);
    /* conn_itvl is in 1.25 ms units. 16 kHz mono needs ~160 notifications/s. */
    const unsigned itvl_us = (unsigned)desc.conn_itvl * 1250u;
    ESP_LOGI(TAG, "Conn params (%s): interval=%u.%02u ms latency=%u timeout=%u ms",
             when, itvl_us / 1000u, (itvl_us % 1000u) / 10u,
             (unsigned)desc.conn_latency,
             (unsigned)desc.supervision_timeout * 10u);
}

static void refresh_status_display(void)
{
    if (is_recording || recording_requested) {
        display_set_status(DISPLAY_STATUS_RECORDING);
    } else if (audio_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        display_set_status(DISPLAY_STATUS_CONNECTED);
    } else {
        display_set_status(DISPLAY_STATUS_NOT_CONNECTED);
    }
}

static void enter_deep_sleep(const char *source)
{
    ESP_LOGI(TAG, "%s: entering deep sleep", source);

    request_recording_stop(source);
    led_set(false);

    /*
     * Tell the peer we are going away, but do NOT tear NimBLE down:
     * nimble_port_stop() makes nimble_port_run() return, and an ESP-IDF task
     * function that returns hits abort() in vPortTaskWrapper. Deep sleep
     * powers the controller down anyway.
     */
    if (audio_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(audio_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    (void)ble_gap_adv_stop();

    /*
     * Long-press is still held (active-low). ext0 wake-on-low would fire
     * immediately, so wait for release + debounce before arming sleep.
     */
    ESP_LOGI(TAG, "wait BtnA release before sleep");
    while (gpio_get_level(BUTTON_A_GPIO) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS + 50));
    while (gpio_get_level(BUTTON_A_GPIO) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /*
     * Hand POWER_HOLD from the digital driver to the RTC driver without ever
     * releasing it: RTC output data and enable are set while the pad is still
     * on the digital mux, so rtc_gpio_init() switches over an already-high
     * pad. Doing it in the documented order (init -> direction -> level)
     * drives the pad low in between and powers the board off.
     */
    rtc_gpio_set_level((gpio_num_t)POWER_HOLD_GPIO, 1);
    rtc_gpio_set_direction((gpio_num_t)POWER_HOLD_GPIO, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_init((gpio_num_t)POWER_HOLD_GPIO);
    rtc_gpio_hold_en((gpio_num_t)POWER_HOLD_GPIO);

    /* Park the LCD pads so they do not float through deep sleep. */
    display_prepare_deep_sleep();

    /* BtnA (G37) is input-only: no internal pulls exist, the board pulls it up. */
    esp_err_t wr = esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_A_GPIO, 0);
    if (wr != ESP_OK) {
        /* Sleeping now would be unwakeable; stay up instead. */
        ESP_LOGE(TAG, "ext0 wake enable failed (%s); staying awake",
                 esp_err_to_name(wr));
        /* Same handover order as the wake path: drive first, release last. */
        power_hold_on();
        rtc_gpio_hold_dis((gpio_num_t)POWER_HOLD_GPIO);
        (void)display_resume();
        refresh_status_display();
        ble_app_advertise();
        return;
    }

    ESP_LOGI(TAG, "deep sleep now (wake on BtnA)");
    esp_deep_sleep_start();
}

static void emulate_long_press(const char *source)
{
    ESP_LOGI(TAG, "%s: Long-press -> sleep", source);
    enter_deep_sleep(source);
}

static void wait_button_a_release(void)
{
    while (gpio_get_level(BUTTON_A_GPIO) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    /* Settle after release so the wake press is not treated as a click. */
    vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS + 20));
}

static void handle_serial_command(uint8_t command, const char *source)
{
    if (command == 'r' || command == 'R') {
        request_recording_start(source);
    } else if (command == 's' || command == 'S') {
        request_recording_stop(source);
    } else if (command == 'c' || command == 'C' || command == '1') {
        emulate_single_click(source);
    } else if (command == 'd' || command == 'D' || command == '2') {
        emulate_double_click(source);
    } else if (command == 'p' || command == 'P') {
        ESP_LOGI(TAG, "%s: colour bars 0xF800/0x07E0/0x001F/0xFFFF top-to-bottom",
                 source);
        display_test_pattern();
    } else if (command == 'l' || command == 'L') {
        emulate_long_press(source);
    } else if (command == 'm' || command == 'M') {
        if (is_recording || recording_requested) {
            ESP_LOGW(TAG, "%s: mic sweep ignored while recording", source);
        } else if (mic_sweep_requested) {
            ESP_LOGW(TAG, "%s: mic sweep already pending", source);
        } else {
            ESP_LOGI(TAG, "%s: mic PDM config sweep requested", source);
            mic_sweep_requested = true;
        }
    } else if (command == 'g' || command == 'G') {
        int32_t next = mic_gain * 2;
        if (next > 16) {
            next = 1;
        }
        mic_gain = next;
        ESP_LOGI(TAG, "%s: mic gain = %d", source, (int)mic_gain);
    } else if (command == 'h' || command == 'H') {
        ESP_LOGI(TAG,
                 "%s commands: 'r'=start, 's'=stop, "
                 "'c'/'1'=single-click, 'd'/'2'=double-click, "
                 "'l'=long-press sleep, 'p'=colour bars, "
                 "'m'=mic PDM sweep, 'g'=cycle mic gain (1..16), 'h'=help",
                 source);
    }
}

static void send_status_event(uint8_t event_code, const char *event_name)
{
    if (audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    const uint8_t packet[] = {0x00, EVENT_PACKET_SYNC_BYTE, event_code};
    struct os_mbuf *om = ble_hs_mbuf_from_flat(packet, sizeof(packet));
    if (om == NULL) {
        ESP_LOGW(TAG, "Failed to allocate status event buffer for %s", event_name);
        return;
    }

    int rc = ble_gattc_notify_custom(audio_conn_handle, audio_tx_char_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to send status event %s: rc=%d", event_name, rc);
        return;
    }

    ESP_LOGI(TAG, "Sent status event: %s", event_name);
}

static void send_operation_mode_status(void)
{
    if (audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    const uint8_t packet[] = {
        0x00,
        EVENT_PACKET_SYNC_BYTE,
        EVENT_OPERATION_MODE,
        operation_mode,
        operation_mode_pending,
    };
    struct os_mbuf *om = ble_hs_mbuf_from_flat(packet, sizeof(packet));
    if (om == NULL) {
        ESP_LOGW(TAG, "Failed to allocate operation mode status buffer");
        return;
    }

    int rc = ble_gattc_notify_custom(audio_conn_handle, audio_tx_char_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to send operation mode status: rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "Sent operation mode status: effective=%u pending=%u",
             operation_mode, operation_mode_pending);
}

static void apply_pending_operation_mode(void)
{
    uint8_t requested = operation_mode_pending;

    if (requested == OPERATION_MODE_PENDING_NONE) {
        return;
    }
    if (requested == operation_mode) {
        operation_mode_pending = OPERATION_MODE_PENDING_NONE;
        send_operation_mode_status();
        return;
    }
    if (requested != OPERATION_MODE_NORMAL &&
        requested != OPERATION_MODE_DRIVING) {
        operation_mode_pending = OPERATION_MODE_PENDING_NONE;
        send_operation_mode_status();
        return;
    }
    if (is_recording || recording_requested) {
        return;
    }

    operation_mode = requested;
    operation_mode_pending = OPERATION_MODE_PENDING_NONE;
    ESP_LOGI(TAG, "Operation mode: %s",
             operation_mode == OPERATION_MODE_DRIVING ? "DRIVING" : "NORMAL");
    send_operation_mode_status();
}

static int
audio_gatt_svr_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ble_uuid_cmp(ctxt->chr->uuid, &gatt_svr_rx_uuid.u) == 0) {
        if (OS_MBUF_PKTLEN(ctxt->om) >= 1) {
            uint8_t data[2] = {0};
            uint16_t data_len = OS_MBUF_PKTLEN(ctxt->om) < sizeof(data)
                                    ? OS_MBUF_PKTLEN(ctxt->om)
                                    : sizeof(data);
            if (os_mbuf_copydata(ctxt->om, 0, data_len, data) != 0) {
                return BLE_ATT_ERR_UNLIKELY;
            }

            uint8_t cmd = data[0];
            if (cmd == 0x01) {
                request_recording_start("BLE");
            } else if (cmd == 0x00) {
                request_recording_stop("BLE");
            } else if (cmd == COMMAND_SET_OPERATION_MODE && data_len >= 2) {
                uint8_t requested = data[1];
                if (requested == OPERATION_MODE_NORMAL ||
                    requested == OPERATION_MODE_DRIVING) {
                    operation_mode_pending = requested;
                    ESP_LOGI(TAG, "Operation mode request: %s%s",
                             requested == OPERATION_MODE_DRIVING ? "DRIVING" : "NORMAL",
                             (is_recording || recording_requested) ? " (pending)" : "");
                    send_operation_mode_status();
                    apply_pending_operation_mode();
                } else {
                    ESP_LOGW(TAG, "Invalid operation mode: %u", requested);
                    send_operation_mode_status();
                }
            } else if (cmd == COMMAND_SET_TAP_NOTIFY_ONLY && data_len >= 2) {
                tap_notify_only = (data[1] != 0);
                ESP_LOGI(TAG, "Tap notify-only: %s",
                         tap_notify_only ? "ON (host authorizes)" : "OFF (local toggle)");
            }
        }
        return 0;
    }

    if (ble_uuid_cmp(ctxt->chr->uuid, &gatt_svr_tx_uuid.u) == 0) {
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &gatt_svr_tx_uuid.u,
                .access_cb = audio_gatt_svr_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &audio_tx_char_handle,
            },
            {
                .uuid = &gatt_svr_rx_uuid.u,
                .access_cb = audio_gatt_svr_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &audio_rx_char_handle,
            },
            {
                0,
            },
        },
    },
    {
        0,
    },
};

static void
ble_app_advertise(void)
{
    ble_svc_gap_device_name_set(BLE_ADV_NAME);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_uuid128_t adv_uuids128[] = { gatt_svr_svc_uuid };
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = adv_uuids128;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting adv fields: %d", rc);
        return;
    }

    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.name = (uint8_t *)BLE_ADV_NAME;
    rsp_fields.name_len = strlen(BLE_ADV_NAME);
    rsp_fields.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting scan rsp fields: %d", rc);
        return;
    }

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_app_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting advertising: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising started: %s", BLE_ADV_NAME);
}

static void
ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "BLE synced");
    ble_app_advertise();
}

static void
ble_app_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE reset, reason=%d", reason);
}

static int
ble_app_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE connection %s",
                 event->connect.status == 0 ? "established" : "failed");
        if (event->connect.status != 0) {
            ble_app_advertise();
            return 0;
        }
        audio_conn_handle = event->connect.conn_handle;
        ble_att_set_preferred_mtu(BLE_MTU_SIZE);
        ble_gattc_exchange_mtu(audio_conn_handle, NULL, NULL);
        log_conn_params("connect");
        if (conn_param_timer != NULL) {
            (void)esp_timer_stop(conn_param_timer);
            (void)esp_timer_start_once(conn_param_timer, 500 * 1000);
        }
        refresh_status_display();
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected (reason=0x%04x)", event->disconnect.reason);
        if (conn_param_timer != NULL) {
            (void)esp_timer_stop(conn_param_timer);
        }
        audio_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        tap_notify_only = false;
        is_recording = false;
        recording_requested = false;
        stop_requested = false;
        stop_event_pending = false;
        led_set(false);
        (void)set_microphone_enabled(false);
        apply_pending_operation_mode();
        refresh_status_display();
        ble_app_advertise();
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU exchange: mtu=%d", event->mtu.value);
        break;

    case BLE_GAP_EVENT_CONN_UPDATE:
        log_conn_params("conn update");
        /* ble_gap_update_params is unsafe here; start gate / stop paths re-request. */
        if ((is_recording || recording_requested) && !conn_interval_is_fast()) {
            ESP_LOGW(TAG, "conn update left interval slow while recording");
        }
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == audio_tx_char_handle) {
            ESP_LOGI(TAG, "Client %s notifications",
                     event->subscribe.cur_notify ? "enabled" : "disabled");
            if (event->subscribe.cur_notify) {
                send_operation_mode_status();
            }
        }
        break;
    }

    return 0;
}

static const char *pdm_dsr_name(i2s_pdm_dsr_t dsr)
{
    return dsr == I2S_PDM_DSR_16S ? "DSR_16S/2.048MHz" : "DSR_8S/1.024MHz";
}

static const char *pdm_slot_name(i2s_pdm_slot_mask_t mask)
{
    switch (mask) {
    case I2S_PDM_SLOT_LEFT:  return "LEFT";
    case I2S_PDM_SLOT_RIGHT: return "RIGHT";
    default:                 return "BOTH";
    }
}

static const char *pdm_mode_name(i2s_slot_mode_t mode)
{
    return mode == I2S_SLOT_MODE_STEREO ? "STEREO" : "MONO";
}

static const char *pdm_clk_path_name(mic_clk_path_t path)
{
    return path == MIC_CLK_M5 ? "M5-raw" : "IDF";
}

static void pdm_cfg_describe(char *out, size_t len, const mic_pdm_cfg_t *cfg)
{
    snprintf(out, len, "%s %s %s/%s", pdm_clk_path_name(cfg->clk_path),
             pdm_dsr_name(cfg->dn_sample_mode), pdm_mode_name(cfg->slot_mode),
             pdm_slot_name(cfg->slot_mask));
}

/* Port of M5Unified's clock divider search (Speaker_Class.cpp calcClockDiv),
 * already used verbatim in atom_echo_s3r/main.c:563. */
static void calc_clock_div(uint32_t *div_a, uint32_t *div_b, uint32_t *div_n,
                           uint32_t base_clock, uint32_t target_freq)
{
    if (base_clock <= (target_freq << 1)) {
        *div_n = 2;
        *div_a = 1;
        *div_b = 0;
        return;
    }

    uint32_t save_n = 255;
    uint32_t save_a = 63;
    uint32_t save_b = 62;
    if (target_freq != 0) {
        float fdiv = (float)base_clock / target_freq;
        uint32_t n = (uint32_t)fdiv;
        if (n < 256) {
            fdiv -= n;

            float check_base = base_clock;
            while ((int32_t)target_freq >= 0) {
                target_freq <<= 1;
                check_base *= 2;
            }
            float check_target = target_freq;

            uint32_t save_diff = UINT32_MAX;
            if (n < 255) {
                save_a = 1;
                save_b = 0;
                save_n = n + 1;
                save_diff = abs((int)(check_target - check_base / (float)save_n));
            }

            for (uint32_t a = 1; a < 64; ++a) {
                uint32_t b = lroundf(a * fdiv);
                if (a <= b) {
                    continue;
                }
                uint32_t diff = abs((int)(check_target - ((check_base * a) / (n * a + b))));
                if (save_diff <= diff) {
                    continue;
                }
                save_diff = diff;
                save_a = a;
                save_b = b;
                save_n = n;
                if (diff == 0) {
                    break;
                }
            }
        }
    }

    *div_n = save_n;
    *div_a = save_a;
    *div_b = save_b;
}

/* M5Unified Mic_Class.cpp:444/528-548, ESP32-classic (I2S HW v1) branch.
 * atom_echo_s3r/main.c:621 is the same idea for HW v2 and uses a different
 * register layout, so this is a port rather than a reuse.
 * Must run after i2s_channel_enable(): M5 writes the registers and then starts. */
static void apply_m5_mic_clock(const mic_pdm_cfg_t *cfg)
{
    /* PDM: bits = 64, div_m = 2 (Mic_Class.cpp:444). PLL_D2_CLK = 160 MHz / 2. */
    static const uint32_t PLL_D2_CLK = 80u * 1000u * 1000u;
    static const uint32_t bits_per_sample = 64;
    static const uint32_t div_m = 2;

    uint32_t div_a = 0, div_b = 0, div_n = 0;
    calc_clock_div(&div_a, &div_b, &div_n, PLL_D2_CLK / (bits_per_sample * div_m),
                   I2S_SAMPLE_RATE);

    i2s_dev_t *dev = &I2S0;
    dev->pdm_conf.rx_sinc_dsr_16_en = (cfg->dn_sample_mode == I2S_PDM_DSR_16S) ? 1 : 0;
    dev->pdm_conf.pdm2pcm_conv_en = 1;
    dev->pdm_conf.rx_pdm_en = 1;

    dev->sample_rate_conf.rx_bck_div_num = div_m;
    dev->clkm_conf.clkm_div_a = div_a;
    dev->clkm_conf.clkm_div_b = div_b;
    dev->clkm_conf.clkm_div_num = div_n;
    dev->clkm_conf.clka_en = 0;   /* APLL off => PLL_160M */

    /* If RX is not reset here, BCK polarity may be inverted (M5 comment). */
    dev->conf.rx_reset = 1;
    dev->conf.rx_fifo_reset = 1;
    dev->conf.rx_reset = 0;
    dev->conf.rx_fifo_reset = 0;

    ESP_LOGI(TAG, "M5 raw PDM clock: div_n=%u div_a=%u div_b=%u div_m=%u dsr16=%d",
             (unsigned)div_n, (unsigned)div_a, (unsigned)div_b, (unsigned)div_m,
             (int)(cfg->dn_sample_mode == I2S_PDM_DSR_16S));
}

static esp_err_t set_microphone_enabled(bool enabled)
{
    if (i2s_rx_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (enabled == mic_enabled) {
        return ESP_OK;
    }

    esp_err_t ret;
    if (enabled) {
        ret = i2s_channel_enable(i2s_rx_handle);
    } else {
        ret = i2s_channel_disable(i2s_rx_handle);
    }
    if (ret == ESP_OK) {
        mic_enabled = enabled;
        if (enabled && mic_pdm_cfg.clk_path == MIC_CLK_M5) {
            /* Overwrite the clock the IDF driver just programmed. */
            apply_m5_mic_clock(&mic_pdm_cfg);
        }
    }
    return ret;
}

static void fill_pdm_rx_config(i2s_pdm_rx_config_t *pdm_cfg, const mic_pdm_cfg_t *cfg)
{
    *pdm_cfg = (i2s_pdm_rx_config_t){
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                   cfg->slot_mode),
        .gpio_cfg = {
            .clk = PDM_CLK_GPIO,
            .din = PDM_DIN_GPIO,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };
    pdm_cfg->clk_cfg.dn_sample_mode = cfg->dn_sample_mode;
    pdm_cfg->slot_cfg.slot_mode = cfg->slot_mode;
    pdm_cfg->slot_cfg.slot_mask = cfg->slot_mask;
}

static esp_err_t apply_pdm_config(const mic_pdm_cfg_t *cfg)
{
    if (i2s_rx_handle == NULL || cfg == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    bool was_enabled = mic_enabled;
    if (was_enabled) {
        esp_err_t dis = set_microphone_enabled(false);
        if (dis != ESP_OK) {
            return dis;
        }
    }

    i2s_pdm_rx_config_t pdm_cfg;
    fill_pdm_rx_config(&pdm_cfg, cfg);

    esp_err_t ret = i2s_channel_reconfig_pdm_rx_clock(i2s_rx_handle, &pdm_cfg.clk_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "reconfig PDM clock failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2s_channel_reconfig_pdm_rx_slot(i2s_rx_handle, &pdm_cfg.slot_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "reconfig PDM slot failed: %s", esp_err_to_name(ret));
        return ret;
    }

    mic_pdm_cfg = *cfg;
    if (was_enabled) {
        ret = set_microphone_enabled(true);
    }
    return ret;
}

static esp_err_t init_audio(const mic_pdm_cfg_t *cfg)
{
    const mic_pdm_cfg_t *use = cfg != NULL ? cfg : &MIC_PDM_DEFAULT;
    char desc[64];
    pdm_cfg_describe(desc, sizeof(desc), use);
    ESP_LOGI(TAG, "Initializing PDM microphone (G0/G34) %s...", desc);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT_NUM, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = DMA_BUFFER_COUNT;
    chan_cfg.dma_frame_num = DMA_FRAME_COUNT;

    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &i2s_rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_pdm_rx_config_t pdm_cfg;
    fill_pdm_rx_config(&pdm_cfg, use);

    ret = i2s_channel_init_pdm_rx_mode(i2s_rx_handle, &pdm_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_pdm_rx_mode failed: %s", esp_err_to_name(ret));
        return ret;
    }

    mic_pdm_cfg = *use;
    /* Keep disabled until recording starts. */
    mic_enabled = false;
    pdm_cfg_describe(desc, sizeof(desc), &mic_pdm_cfg);
    ESP_LOGI(TAG, "PDM microphone ready (%s, gain=%d)", desc, (int)mic_gain);
    return ESP_OK;
}

typedef struct {
    int32_t min_v;
    int32_t max_v;
    int64_t sum;
    int64_t sum_sq;
    uint32_t samples;
    uint32_t zero_cross;
    int16_t last;
    bool have_last;
    uint16_t exact[MIC_STATS_DISTINCT_CAP];
    uint16_t exact_count;
    int16_t head[MIC_STATS_DUMP_SAMPLES];
    uint8_t head_count;
} mic_raw_stats_t;

static void mic_raw_stats_init(mic_raw_stats_t *st)
{
    memset(st, 0, sizeof(*st));
    st->min_v = INT16_MAX;
    st->max_v = INT16_MIN;
}

static void mic_raw_stats_add(mic_raw_stats_t *st, int16_t sample)
{
    int32_t x = sample;
    if (x < st->min_v) {
        st->min_v = x;
    }
    if (x > st->max_v) {
        st->max_v = x;
    }
    st->sum += x;
    st->sum_sq += (int64_t)x * (int64_t)x;

    if (st->have_last && ((st->last < 0) != (sample < 0))) {
        st->zero_cross++;
    }
    st->last = sample;
    st->have_last = true;

    if (st->head_count < MIC_STATS_DUMP_SAMPLES) {
        st->head[st->head_count++] = sample;
    }

    /* Linear scan over the table is fine only while both the table and the
     * window are small; a full-rate scan of 256 entries at 16 kHz is not. */
    if (st->samples < MIC_STATS_DISTINCT_WINDOW &&
        st->exact_count < MIC_STATS_DISTINCT_CAP) {
        bool found = false;
        for (uint16_t i = 0; i < st->exact_count; i++) {
            if ((int16_t)st->exact[i] == sample) {
                found = true;
                break;
            }
        }
        if (!found) {
            st->exact[st->exact_count++] = (uint16_t)sample;
        }
    }
    st->samples++;
}

/* A constant stream and a broadband-noise stream both fail transcription but
 * mean completely different things, so classify on zero-crossing rate:
 * speech sits around 300-1500 Hz, white noise near half of Nyquist. */
static const char *mic_raw_stats_verdict(const mic_raw_stats_t *st, double zcr_hz)
{
    if (st->zero_cross == 0 || st->exact_count < 8) {
        return "CONST";
    }
    if (zcr_hz > MIC_ZCR_NOISE_HZ) {
        return "NOISE";
    }
    if (zcr_hz >= MIC_ZCR_SIGNAL_MIN && st->exact_count >= 100) {
        return "SIGNAL";
    }
    return "WEAK";
}

static void mic_raw_stats_log(const char *label, const mic_raw_stats_t *st,
                              uint32_t rate_hz)
{
    if (st->samples == 0) {
        ESP_LOGW(TAG, "%s: no samples", label);
        return;
    }
    double mean = (double)st->sum / (double)st->samples;
    int32_t rms = (int32_t)sqrt((double)st->sum_sq / (double)st->samples);
    double dur_s = (double)st->samples / (double)rate_hz;
    double zcr_hz = dur_s > 0.0 ? ((double)st->zero_cross / 2.0) / dur_s : 0.0;
    const char *distinct_suffix =
        st->exact_count >= MIC_STATS_DISTINCT_CAP ? "+" : "";

    ESP_LOGI(TAG,
             "%s n=%u min=%d max=%d mean=%.1f rms=%d zcr=%.0fHz distinct=%u%s %s",
             label, (unsigned)st->samples, (int)st->min_v, (int)st->max_v, mean,
             (int)rms, zcr_hz, (unsigned)st->exact_count, distinct_suffix,
             mic_raw_stats_verdict(st, zcr_hz));

    /* The aggregate numbers hide the difference between a waveform and a
     * sample-to-sample full-scale jump; the head dump does not. */
    char line[MIC_STATS_DUMP_SAMPLES * 8 + 8];
    size_t off = 0;
    for (uint8_t i = 0; i < st->head_count && off + 8 < sizeof(line); i++) {
        off += snprintf(line + off, sizeof(line) - off, "%d ", (int)st->head[i]);
    }
    ESP_LOGI(TAG, "%s head: %s", label, line);
}

static void run_mic_config_sweep(int16_t *i2s_buffer, size_t i2s_buffer_bytes)
{
    static const mic_pdm_cfg_t sweep_cfgs[] = {
        { MIC_CLK_IDF, I2S_PDM_DSR_8S,  I2S_PDM_SLOT_RIGHT, I2S_SLOT_MODE_MONO },
        { MIC_CLK_IDF, I2S_PDM_DSR_16S, I2S_PDM_SLOT_RIGHT, I2S_SLOT_MODE_MONO },
        { MIC_CLK_IDF, I2S_PDM_DSR_8S,  I2S_PDM_SLOT_LEFT,  I2S_SLOT_MODE_MONO },
        { MIC_CLK_IDF, I2S_PDM_DSR_16S, I2S_PDM_SLOT_LEFT,  I2S_SLOT_MODE_MONO },
        { MIC_CLK_IDF, I2S_PDM_DSR_8S,  I2S_PDM_SLOT_BOTH,  I2S_SLOT_MODE_STEREO },
        { MIC_CLK_IDF, I2S_PDM_DSR_16S, I2S_PDM_SLOT_BOTH,  I2S_SLOT_MODE_STEREO },
        { MIC_CLK_M5,  I2S_PDM_DSR_16S, I2S_PDM_SLOT_RIGHT, I2S_SLOT_MODE_MONO },
        { MIC_CLK_M5,  I2S_PDM_DSR_16S, I2S_PDM_SLOT_BOTH,  I2S_SLOT_MODE_STEREO },
        { MIC_CLK_M5,  I2S_PDM_DSR_8S,  I2S_PDM_SLOT_RIGHT, I2S_SLOT_MODE_MONO },
    };
    const size_t cfg_count = sizeof(sweep_cfgs) / sizeof(sweep_cfgs[0]);
    mic_pdm_cfg_t restore = mic_pdm_cfg;
    size_t bytes_read = 0;
    char desc[64];
    char label[80];

    ESP_LOGI(TAG, "=== Mic PDM sweep start: %u configs, keep talking for ~%u s ===",
             (unsigned)cfg_count,
             (unsigned)((cfg_count * (MIC_SWEEP_DISCARD_MS + MIC_SWEEP_MEASURE_MS)) / 1000));

    for (size_t i = 0; i < cfg_count; i++) {
        const mic_pdm_cfg_t *cfg = &sweep_cfgs[i];
        bool stereo = cfg->slot_mode == I2S_SLOT_MODE_STEREO;
        pdm_cfg_describe(desc, sizeof(desc), cfg);
        ESP_LOGI(TAG, "Sweep #%u: %s", (unsigned)(i + 1), desc);

        esp_err_t ret = apply_pdm_config(cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Sweep #%u apply failed: %s", (unsigned)(i + 1),
                     esp_err_to_name(ret));
            continue;
        }
        ret = set_microphone_enabled(true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Sweep #%u enable failed: %s", (unsigned)(i + 1),
                     esp_err_to_name(ret));
            continue;
        }

        int64_t t0 = esp_timer_get_time();
        while ((esp_timer_get_time() - t0) < (int64_t)MIC_SWEEP_DISCARD_MS * 1000) {
            if (i2s_channel_read(i2s_rx_handle, i2s_buffer, i2s_buffer_bytes,
                                 &bytes_read, MIC_READ_TIMEOUT_MS) != ESP_OK) {
                break;
            }
        }

        /* In stereo the two slots are interleaved and only one carries the mic,
         * so they have to be measured apart. */
        mic_raw_stats_t st[2];
        mic_raw_stats_init(&st[0]);
        mic_raw_stats_init(&st[1]);
        t0 = esp_timer_get_time();
        while ((esp_timer_get_time() - t0) < (int64_t)MIC_SWEEP_MEASURE_MS * 1000) {
            esp_err_t rd = i2s_channel_read(i2s_rx_handle, i2s_buffer,
                                            i2s_buffer_bytes, &bytes_read,
                                            MIC_READ_TIMEOUT_MS);
            if (rd != ESP_OK) {
                ESP_LOGW(TAG, "Sweep #%u read failed: %s", (unsigned)(i + 1),
                         esp_err_to_name(rd));
                break;
            }
            size_t n = bytes_read / sizeof(int16_t);
            if (stereo) {
                for (size_t k = 0; k + 1 < n; k += 2) {
                    mic_raw_stats_add(&st[0], i2s_buffer[k]);
                    mic_raw_stats_add(&st[1], i2s_buffer[k + 1]);
                }
            } else {
                for (size_t k = 0; k < n; k++) {
                    mic_raw_stats_add(&st[0], i2s_buffer[k]);
                }
            }
        }

        if (stereo) {
            snprintf(label, sizeof(label), "Sweep#%u[L] %s", (unsigned)(i + 1), desc);
            mic_raw_stats_log(label, &st[0], I2S_SAMPLE_RATE);
            snprintf(label, sizeof(label), "Sweep#%u[R] %s", (unsigned)(i + 1), desc);
            mic_raw_stats_log(label, &st[1], I2S_SAMPLE_RATE);
        } else {
            snprintf(label, sizeof(label), "Sweep#%u %s", (unsigned)(i + 1), desc);
            mic_raw_stats_log(label, &st[0], I2S_SAMPLE_RATE);
        }
        (void)set_microphone_enabled(false);
    }

    if (apply_pdm_config(&restore) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to restore PDM config; reapplying default");
        (void)apply_pdm_config(&MIC_PDM_DEFAULT);
    }
    pdm_cfg_describe(desc, sizeof(desc), &mic_pdm_cfg);
    ESP_LOGI(TAG, "=== Mic PDM sweep done (restored %s) ===", desc);
}

static esp_err_t init_button(void)
{
    gpio_config_t button_cfg = {
        .pin_bit_mask = 1ULL << BUTTON_A_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE, /* board has external pull-up; G37 input-only */
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&button_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "button gpio_config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Button A on GPIO %d (active low)", BUTTON_A_GPIO);
    return ESP_OK;
}

static void audio_stream_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "Audio streaming task started");

    esp_err_t audio_ret = init_audio(&MIC_PDM_DEFAULT);
    if (audio_ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio init failed (%s) — audio task exiting",
                 esp_err_to_name(audio_ret));
        vTaskDelete(NULL);
        return;
    }

    size_t i2s_buffer_bytes = DMA_FRAME_COUNT * sizeof(int16_t);
    int16_t *i2s_buffer = (int16_t *)malloc(i2s_buffer_bytes);
    uint8_t *tx_packet = (uint8_t *)malloc(RECORDING_PACKET_SIZE + 2);

    if (i2s_buffer == NULL || tx_packet == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffers");
        vTaskDelete(NULL);
        return;
    }

    size_t bytes_read = 0;
    mic_raw_stats_t raw_log_stats;
    mic_raw_stats_init(&raw_log_stats);
    uint32_t clip_count = 0;
    uint32_t proc_samples = 0;
    int32_t proc_peak = 0;
    int64_t proc_sum_sq = 0;

    while (1) {
        if (mic_sweep_requested && !is_recording && !recording_requested) {
            mic_sweep_requested = false;
            run_mic_config_sweep(i2s_buffer, i2s_buffer_bytes);
        }

        if (recording_requested && !is_recording) {
            ESP_LOGI(TAG, "Starting recording...");
            /* Overlap fast-link negotiation with mic bring-up. */
            request_conn_params(true);
            if (set_microphone_enabled(true) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to enable microphone");
                recording_requested = false;
                request_conn_params(false);
                refresh_status_display();
                continue;
            }
            /* Settle DC: feed IIR until residual is small, with timeout. */
            mic_dc_offset = 0;
            {
                int64_t t0 = esp_timer_get_time();
                uint32_t settled = 0;
                bool ok = false;
                while ((esp_timer_get_time() - t0) <
                       (int64_t)MIC_START_DISCARD_MS * 1000) {
                    if (stop_requested || !recording_requested) {
                        break;
                    }
                    esp_err_t rd = i2s_channel_read(i2s_rx_handle, i2s_buffer,
                                                    i2s_buffer_bytes, &bytes_read,
                                                    MIC_READ_TIMEOUT_MS);
                    if (rd != ESP_OK) {
                        ESP_LOGW(TAG, "DC settle read failed: %s", esp_err_to_name(rd));
                        break;
                    }
                    size_t n = bytes_read / sizeof(int16_t);
                    for (size_t i = 0; i < n; i++) {
                        int32_t x = i2s_buffer[i];
                        mic_dc_offset += (x - mic_dc_offset) >> MIC_DC_SHIFT;
                        int32_t resid = x - mic_dc_offset;
                        if (resid < 0) {
                            resid = -resid;
                        }
                        if (resid <= MIC_DC_SETTLE_THRESH) {
                            settled++;
                            if (settled >= MIC_DC_SETTLE_SAMPLES) {
                                ok = true;
                                break;
                            }
                        } else {
                            settled = 0;
                        }
                    }
                    if (ok) {
                        break;
                    }
                }
                ESP_LOGI(TAG, "Mic DC settle offset=%d %s (%u ms)",
                         (int)mic_dc_offset, ok ? "ok" : "timeout",
                         (unsigned)((esp_timer_get_time() - t0) / 1000));
            }
            if (stop_requested || !recording_requested) {
                ESP_LOGI(TAG, "Recording start aborted during settle");
                (void)set_microphone_enabled(false);
                recording_requested = false;
                request_conn_params(false);
                led_set(false);
                refresh_status_display();
                continue;
            }
            /* Prefer fast interval before first PCM notify; do not block forever. */
            (void)wait_for_fast_conn_params(CONN_FAST_GATE_TIMEOUT_MS);
            if (stop_requested || !recording_requested ||
                audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
                ESP_LOGI(TAG, "Recording start aborted during conn gate");
                (void)set_microphone_enabled(false);
                recording_requested = false;
                request_conn_params(false);
                led_set(false);
                refresh_status_display();
                continue;
            }
            mic_level_log_left = I2S_SAMPLE_RATE; /* raw+proc stats for ~1 s */
            mic_raw_stats_init(&raw_log_stats);
            clip_count = 0;
            proc_samples = 0;
            proc_peak = 0;
            proc_sum_sq = 0;

            is_recording = true;
            recording_requested = false;
            seq_num = 0;
            tx_sent = 0;
            tx_failed = 0;
            tx_nomem = 0;
            tx_started_us = esp_timer_get_time();
            led_set(true);
            refresh_status_display();
            send_status_event(EVENT_RECORDING_STARTED, "recording_started");
        }

        if (stop_requested) {
            if (stop_event_pending) {
                ESP_LOGI(TAG, "Stopping recording...");
                const uint32_t ms = tx_started_us
                    ? (uint32_t)((esp_timer_get_time() - tx_started_us) / 1000) : 0;
                const uint32_t total = tx_sent + tx_failed + tx_nomem;
                ESP_LOGI(TAG,
                         "TX summary: sent=%u failed=%u nomem=%u over %u ms "
                         "(%u pkt/s, %u B/s; 16 kHz mono needs 160 pkt/s, 32000 B/s)",
                         (unsigned)tx_sent, (unsigned)tx_failed, (unsigned)tx_nomem,
                         (unsigned)ms,
                         (unsigned)(ms ? total * 1000u / ms : 0),
                         (unsigned)(ms ? (uint32_t)((uint64_t)tx_sent * RECORDING_PACKET_SIZE
                                                    * 1000u / ms) : 0));
                log_conn_params("at stop");
                send_status_event(EVENT_RECORDING_STOPPED, "recording_stopped");
            }
            (void)set_microphone_enabled(false);
            stop_requested = false;
            stop_event_pending = false;
            led_set(false);
            request_conn_params(false);
            apply_pending_operation_mode();
            refresh_status_display();
        }

        if (!is_recording || audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        esp_err_t ret = i2s_channel_read(i2s_rx_handle, i2s_buffer, i2s_buffer_bytes,
                                         &bytes_read, MIC_READ_TIMEOUT_MS * 2);
        if (ret != ESP_OK || bytes_read == 0) {
            continue;
        }
        if (stop_requested || !is_recording) {
            continue;
        }

        size_t samples_read = bytes_read / sizeof(int16_t);

        /* A stereo capture interleaves both PDM slots but only one carries the
         * mic, so compact the picked slot down before the DSP and BLE stages. */
        if (mic_pdm_cfg.slot_mode == I2S_SLOT_MODE_STEREO) {
            size_t out = 0;
            for (size_t i = mic_stereo_pick; i < samples_read; i += 2) {
                i2s_buffer[out++] = i2s_buffer[i];
            }
            samples_read = out;
        }

        /* Raw stats before DC/gain; then process for BLE. */
        for (size_t i = 0; i < samples_read; i++) {
            int16_t raw = i2s_buffer[i];
            if (mic_level_log_left > 0) {
                mic_raw_stats_add(&raw_log_stats, raw);
            }
            int32_t x = (int32_t)raw;
            mic_dc_offset += (x - mic_dc_offset) >> MIC_DC_SHIFT;
            int32_t centered = x - mic_dc_offset;
            int32_t v = centered * mic_gain;
            if (v > INT16_MAX) {
                v = INT16_MAX;
                if (mic_level_log_left > 0) {
                    clip_count++;
                }
            } else if (v < INT16_MIN) {
                v = INT16_MIN;
                if (mic_level_log_left > 0) {
                    clip_count++;
                }
            }
            i2s_buffer[i] = (int16_t)v;
            if (mic_level_log_left > 0) {
                int32_t a = v < 0 ? -v : v;
                if (a > proc_peak) {
                    proc_peak = a;
                }
                proc_sum_sq += (int64_t)v * (int64_t)v;
                proc_samples++;
            }
        }
        if (mic_level_log_left > 0) {
            uint32_t n = samples_read < mic_level_log_left
                             ? (uint32_t)samples_read
                             : mic_level_log_left;
            mic_level_log_left -= n;
            if (mic_level_log_left == 0 && proc_samples > 0) {
                mic_raw_stats_log("Mic raw", &raw_log_stats, I2S_SAMPLE_RATE);
                int32_t rms = (int32_t)sqrt((double)proc_sum_sq / (double)proc_samples);
                unsigned clip_pct = (unsigned)((clip_count * 100u) / proc_samples);
                ESP_LOGI(TAG, "Mic proc peak=%d rms=%d gain=%d clip=%u%%",
                         (int)proc_peak, (int)rms, (int)mic_gain, clip_pct);
            }
        }

        size_t sample_index = 0;
        while (sample_index < samples_read) {
            size_t samples_to_send =
                (samples_read - sample_index < RECORDING_PACKET_SIZE / 2)
                    ? (samples_read - sample_index)
                    : (RECORDING_PACKET_SIZE / 2);

            tx_packet[0] = seq_num++;
            tx_packet[1] = AUDIO_PACKET_SYNC_BYTE;
            memcpy(&tx_packet[2], &i2s_buffer[sample_index], samples_to_send * 2);
            size_t packet_size = 2 + (samples_to_send * 2);

            struct os_mbuf *om = ble_hs_mbuf_from_flat(tx_packet, packet_size);
            if (om == NULL) {
                /* Pool drained by the link, not a permanent failure - give the
                 * controller a tick to drain before giving up on the frame. */
                vTaskDelay(1);
                om = ble_hs_mbuf_from_flat(tx_packet, packet_size);
            }
            if (om == NULL) {
                tx_nomem++;
            } else {
                int nrc = ble_gattc_notify_custom(audio_conn_handle, audio_tx_char_handle, om);
                if (nrc == 0) {
                    tx_sent++;
                } else {
                    /* One tick is the shortest real wait at CONFIG_FREERTOS_HZ=100;
                     * pdMS_TO_TICKS(2) rounds to 0 and does not back off at all. */
                    vTaskDelay(1);
                    om = ble_hs_mbuf_from_flat(tx_packet, packet_size);
                    if (om == NULL) {
                        tx_nomem++;
                    } else {
                        nrc = ble_gattc_notify_custom(audio_conn_handle, audio_tx_char_handle, om);
                        if (nrc == 0) {
                            tx_sent++;
                        } else {
                            tx_failed++;
                            if (tx_failed <= 5 || (tx_failed % 100) == 0) {
                                ESP_LOGW(TAG, "notify failed rc=%d (drop #%u)",
                                         nrc, (unsigned)tx_failed);
                            }
                        }
                    }
                }
            }

            sample_index += samples_to_send;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

static void button_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "Button task started");

    int stable_level = gpio_get_level(BUTTON_A_GPIO);
    int last_level = stable_level;
    TickType_t last_change_tick = xTaskGetTickCount();
    TickType_t first_click_tick = 0;
    TickType_t press_start_tick = last_change_tick;
    uint8_t click_count = 0;
    bool long_press_fired = false;
    /* Ignore a press that is already down here (e.g. the wake press): a stale
     * press_start_tick would fire long-press immediately and sleep again. */
    bool press_active = false;
    bool awaiting_single = false;

    while (1) {
        int level = gpio_get_level(BUTTON_A_GPIO);
        TickType_t now = xTaskGetTickCount();

        if (level != last_level) {
            last_level = level;
            last_change_tick = now;
        }

        if (level != stable_level &&
            (now - last_change_tick) >= pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
            stable_level = level;
            if (stable_level == 0) {
                press_active = true;
                press_start_tick = now;
                long_press_fired = false;
                awaiting_single = false;
                if (click_count == 0) {
                    click_count = 1;
                    first_click_tick = now;
                } else {
                    TickType_t click_delta = now - first_click_tick;
                    if (click_delta <= pdMS_TO_TICKS(BUTTON_DOUBLE_CLICK_MS)) {
                        emulate_double_click("Button A double-click");
                        click_count = 0;
                        first_click_tick = 0;
                    } else {
                        click_count = 1;
                        first_click_tick = now;
                    }
                }
            } else {
                press_active = false;
                if (long_press_fired) {
                    click_count = 0;
                    first_click_tick = 0;
                    long_press_fired = false;
                    awaiting_single = false;
                } else if (click_count == 1) {
                    /* Defer single until double-click window ends after release. */
                    awaiting_single = true;
                }
            }
        }

        if (press_active && !long_press_fired && stable_level == 0 &&
            (now - press_start_tick) >= pdMS_TO_TICKS(BUTTON_LONG_PRESS_MS)) {
            long_press_fired = true;
            click_count = 0;
            first_click_tick = 0;
            awaiting_single = false;
            emulate_long_press("Button A");
        }

        if (awaiting_single && click_count == 1 && !long_press_fired &&
            (now - first_click_tick) > pdMS_TO_TICKS(BUTTON_DOUBLE_CLICK_MS)) {
            click_count = 0;
            first_click_tick = 0;
            awaiting_single = false;
            emulate_single_click("Button A");
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

static void uart_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "Serial command task started");

    uint8_t uart_buf[256];
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, uart_buf, sizeof(uart_buf),
                                  pdMS_TO_TICKS(100));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                handle_serial_command(uart_buf[i], "UART");
            }
        }
    }
}

static void ble_host_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    /* nimble_port_run() returns on stop; returning from a task aborts. */
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    /*
     * Must hold power immediately on StickC Plus2. After a deep-sleep wake the
     * pad is still latched high by the RTC hold, so drive the digital output
     * high first and release the hold last - the other order lets gpio_config()
     * drive HOLD low and the board powers itself off.
     */
    power_hold_on();
    rtc_gpio_hold_dis((gpio_num_t)POWER_HOLD_GPIO);
    power_hold_on();
    ESP_LOGI(TAG, "HarnessNode Plus2 (M5StickC Plus2) - Starting");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    smp_ota_init();
    smp_ota_set_busy_callback(on_ota_busy);

    adpcm_init_state(&adpcm_enc_state);
    led_init();

    if (display_init() == ESP_OK) {
        refresh_status_display();
    } else {
        ESP_LOGW(TAG, "Display init failed; continuing without LCD");
    }

    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(init_button());
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        ESP_LOGI(TAG, "Woke from deep sleep (BtnA); waiting for release");
        wait_button_a_release();
    }

    /*
     * NimBLE logs "GATT procedure initiated: notify" at INFO for every audio
     * notification - ~600 lines (55 kB) per short recording. esp_log writes to
     * the console UART synchronously, so at 115200 baud that stalls the BLE
     * host task for seconds and starves the audio stream. Warnings and errors
     * still come through.
     */
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    const esp_timer_create_args_t cp_args = {
        .callback = deferred_idle_conn_params,
        .name = "conn_param",
    };
    ESP_ERROR_CHECK(esp_timer_create(&cp_args, &conn_param_timer));

    ESP_LOGI(TAG, "Initializing NimBLE...");
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;

    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_count_cfg(smp_ota_svc_defs());
    ble_gatts_add_svcs(gatt_svr_svcs);
    ble_gatts_add_svcs(smp_ota_svc_defs());
    nimble_port_freertos_init(ble_host_task);

    xTaskCreatePinnedToCore(audio_stream_task, "audio_stream", 8192, NULL, 5,
                            &audio_task_handle, 1);
    xTaskCreate(button_task, "button_task", 3072, NULL, 4, &button_task_handle);
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG,
             "Init complete. Adv name %s; BtnA single=record, long=sleep; SMP OTA enabled",
             BLE_ADV_NAME);
    ESP_LOGI(TAG,
             "Serial: 'r'=start 's'=stop 'c'=single 'd'=double 'l'=sleep "
             "'m'=mic-sweep 'g'=gain 'p'=colour-bars 'h'=help");
}
