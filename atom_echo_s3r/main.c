#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/i2c.h"
#include "hal/i2s_ll.h"
#include "soc/i2s_struct.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/util/util.h"

#include "adpcm.h"

static const char *TAG = "voice_bridge";

// Atom Echo S3R Mic_Class configuration from M5Unified.
#define I2S_PORT_NUM      I2S_NUM_1
#define I2S_BCLK_GPIO    17
#define I2S_WS_GPIO       3
#define I2S_MCLK_GPIO    11
#define I2S_DIN_GPIO      4

// ES8311 codec I2C
#define I2C_SDA_GPIO     45
#define I2C_SCL_GPIO      0
#define ES8311_I2C_ADDR  0x18

// Grove Mini OLED (external I2C on Atom Echo S3R)
#define OLED_I2C_PORT         I2C_NUM_1
#define OLED_I2C_SCL_GPIO     1
#define OLED_I2C_SDA_GPIO     2
#define OLED_I2C_ADDR         0x3C
#define OLED_WIDTH            72
#define OLED_HEIGHT           40
#define OLED_PAGE_COUNT       (OLED_HEIGHT / 8)
#define OLED_X_OFFSET         28

// Speaker amp enable (keep LOW while using the microphone)
#define PA_EN_GPIO       18
#define BUTTON_A_GPIO    41
#define BUTTON_POLL_MS   10
#define BUTTON_DEBOUNCE_MS 30
#define BUTTON_DOUBLE_CLICK_MS 500

// I2S configuration
#define I2S_SAMPLE_RATE   16000
#define MIC_OVERSAMPLING  1
#define MIC_NOISE_FILTER  8
#define MIC_MAGNIFICATION 16

// DMA buffer configuration
#define DMA_BUFFER_COUNT  8
#define DMA_FRAME_COUNT   256

// BLE configuration
#define BLE_MTU_SIZE      512
#define BLE_ADV_NAME      "HarnessNode-Echo"

// Recording configuration
#define RECORDING_PACKET_SIZE  200  // bytes of PCM data per packet (fits in BLE MTU)
#define AUDIO_PACKET_SYNC_BYTE 0xAA
#define EVENT_PACKET_SYNC_BYTE 0x55
#define EVENT_RECORDING_STARTED 0x01
#define EVENT_RECORDING_STOPPED 0x02
#define EVENT_DOUBLE_CLICK 0x12
#define EVENT_SINGLE_CLICK 0x14
#define COMMAND_SET_OPERATION_MODE 0x05
#define EVENT_OPERATION_MODE 0x40
#define OPERATION_MODE_NORMAL 0x00
#define OPERATION_MODE_DRIVING 0x01
#define OPERATION_MODE_PENDING_NONE 0xff

/* Handy / HarnessNode UUIDs (same as nordic-main; LSB-first for NimBLE). */
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00);
static const ble_uuid128_t gatt_svr_tx_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00);
static const ble_uuid128_t gatt_svr_rx_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00);

// UART configuration
#define UART_PORT_NUM     UART_NUM_0
#define UART_BAUD_RATE    115200
#define UART_TX_PIN       UART_PIN_NO_CHANGE
#define UART_RX_PIN       UART_PIN_NO_CHANGE

static uint16_t audio_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t audio_tx_char_handle;
static uint16_t audio_rx_char_handle;
static SemaphoreHandle_t oled_mutex;

// ADPCM state
static adpcm_state_t adpcm_enc_state;

// Sequence number for packet ordering
static uint8_t seq_num = 0;

// Task handles
static TaskHandle_t audio_task_handle;
static TaskHandle_t button_task_handle;

// Recording state
static volatile bool is_recording = false;
static volatile bool recording_requested = false;
static volatile bool stop_requested = false;
static volatile bool stop_event_pending = false;
static volatile uint8_t operation_mode = OPERATION_MODE_NORMAL;
static volatile uint8_t operation_mode_pending = OPERATION_MODE_PENDING_NONE;

// I2S handle
static i2s_chan_handle_t i2s_rx_handle = NULL;

typedef enum {
    SYSTEM_STATUS_BOOT,
    SYSTEM_STATUS_READY,
    SYSTEM_STATUS_CONNECTED,
    SYSTEM_STATUS_RECORDING,
    SYSTEM_STATUS_ERROR,
} system_status_t;

static system_status_t current_status = SYSTEM_STATUS_BOOT;
static bool oled_initialized = false;
static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGE_COUNT];

// Forward declarations
static void ble_app_advertise(void);
static void ble_app_on_sync(void);
static void ble_app_on_reset(int reason);
static int ble_app_gap_event(struct ble_gap_event *event, void *arg);
static void audio_stream_task(void *pvParameters);
static void button_task(void *pvParameters);
static void uart_task(void *pvParameters);
static esp_err_t init_button(void);
static esp_err_t init_oled(void);
static void set_system_status(system_status_t status);
static void set_system_error(const char *reason);
static void send_status_event(uint8_t event_code, const char *event_name);
static void send_operation_mode_status(void);
static void apply_pending_operation_mode(void);
static esp_err_t write_i2c_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t value);
static esp_err_t write_i2c_bulk_data(uint8_t dev_addr, const uint8_t *data);
static esp_err_t set_microphone_enabled(bool enabled);
static void calc_clock_div(uint32_t *div_a, uint32_t *div_b, uint32_t *div_n, uint32_t base_clock, uint32_t target_freq);
static void apply_mic_clock_config(i2s_port_t port, uint32_t sample_rate_hz);

static void request_recording_start(const char *source)
{
    if (is_recording || recording_requested) {
        return;
    }
    ESP_LOGI(TAG, "%s: Start recording requested", source);
    stop_requested = false;
    recording_requested = true;
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
}

static void emulate_single_click(const char *source)
{
    ESP_LOGI(TAG, "%s: Single-click", source);

    /* Single click toggles recording in every operation mode (FW-owned). */
    if (audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "%s: Single-click ignored: no BLE connection", source);
        return;
    }

    if (is_recording) {
        request_recording_stop(source);
    } else if (recording_requested) {
        recording_requested = false;
        ESP_LOGI(TAG, "%s: Pending recording start cancelled", source);
        apply_pending_operation_mode();
    } else {
        request_recording_start(source);
    }

    send_status_event(EVENT_SINGLE_CLICK, "single_click");
}

static void emulate_double_click(const char *source)
{
    ESP_LOGI(TAG, "%s: Double-click (notify only, no recording toggle)", source);
    send_status_event(EVENT_DOUBLE_CLICK, "double_click");
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
    } else if (command == 'h' || command == 'H') {
        ESP_LOGI(TAG,
                 "%s commands: 'r'=start, 's'=stop, "
                 "'c'/'1'=single-click, 'd'/'2'=double-click, 'h'=help",
                 source);
    }
}

static const char *status_to_text(system_status_t status)
{
    switch (status) {
    case SYSTEM_STATUS_BOOT:
        return "BOOT";
    case SYSTEM_STATUS_READY:
        return "READY";
    case SYSTEM_STATUS_CONNECTED:
        return "LINK";
    case SYSTEM_STATUS_RECORDING:
        return "REC";
    case SYSTEM_STATUS_ERROR:
    default:
        return "ERROR";
    }
}

static const uint8_t *font5x7_get_glyph(char c)
{
    static const uint8_t glyph_space[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t glyph_a[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
    static const uint8_t glyph_b[5] = {0x7F, 0x49, 0x49, 0x49, 0x36};
    static const uint8_t glyph_c[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};
    static const uint8_t glyph_d[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
    static const uint8_t glyph_e[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};
    static const uint8_t glyph_i[5] = {0x00, 0x41, 0x7F, 0x41, 0x00};
    static const uint8_t glyph_k[5] = {0x7F, 0x08, 0x14, 0x22, 0x41};
    static const uint8_t glyph_l[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
    static const uint8_t glyph_n[5] = {0x7F, 0x02, 0x04, 0x08, 0x7F};
    static const uint8_t glyph_o[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
    static const uint8_t glyph_r[5] = {0x7F, 0x09, 0x19, 0x29, 0x46};
    static const uint8_t glyph_t[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
    static const uint8_t glyph_y[5] = {0x03, 0x04, 0x78, 0x04, 0x03};

    switch (c) {
    case 'A': return glyph_a;
    case 'B': return glyph_b;
    case 'C': return glyph_c;
    case 'D': return glyph_d;
    case 'E': return glyph_e;
    case 'I': return glyph_i;
    case 'K': return glyph_k;
    case 'L': return glyph_l;
    case 'N': return glyph_n;
    case 'O': return glyph_o;
    case 'R': return glyph_r;
    case 'T': return glyph_t;
    case 'Y': return glyph_y;
    default:  return glyph_space;
    }
}

static esp_err_t oled_write_command(uint8_t command)
{
    uint8_t packet[2] = {0x00, command};
    return i2c_master_write_to_device(OLED_I2C_PORT, OLED_I2C_ADDR, packet, sizeof(packet), pdMS_TO_TICKS(100));
}

static esp_err_t oled_write_data(const uint8_t *data, size_t len)
{
    uint8_t packet[1 + OLED_WIDTH];
    if (len > OLED_WIDTH) {
        return ESP_ERR_INVALID_SIZE;
    }
    packet[0] = 0x40;
    memcpy(&packet[1], data, len);
    return i2c_master_write_to_device(OLED_I2C_PORT, OLED_I2C_ADDR, packet, len + 1, pdMS_TO_TICKS(100));
}

static void oled_clear_buffer(void)
{
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

static void oled_draw_pixel(int x, int y)
{
    if ((x < 0) || (x >= OLED_WIDTH) || (y < 0) || (y >= OLED_HEIGHT)) {
        return;
    }
    oled_buffer[(y / 8) * OLED_WIDTH + x] |= (1U << (y & 7));
}

static void oled_draw_text(const char *text, int x, int y, int scale)
{
    for (size_t idx = 0; text[idx] != '\0'; ++idx) {
        const uint8_t *glyph = font5x7_get_glyph(text[idx]);
        for (int col = 0; col < 5; ++col) {
            for (int row = 0; row < 7; ++row) {
                if ((glyph[col] >> row) & 0x01) {
                    for (int sx = 0; sx < scale; ++sx) {
                        for (int sy = 0; sy < scale; ++sy) {
                            oled_draw_pixel(x + (idx * 6 + col) * scale + sx, y + row * scale + sy);
                        }
                    }
                }
            }
        }
    }
}

static esp_err_t oled_flush_buffer(void)
{
    for (uint8_t page = 0; page < OLED_PAGE_COUNT; ++page) {
        esp_err_t ret = oled_write_command(0x10 | (OLED_X_OFFSET >> 4));
        if (ret != ESP_OK) { return ret; }
        ret = oled_write_command(OLED_X_OFFSET & 0x0F);
        if (ret != ESP_OK) { return ret; }
        ret = oled_write_command(0xB0 | page);
        if (ret != ESP_OK) { return ret; }
        ret = oled_write_data(&oled_buffer[page * OLED_WIDTH], OLED_WIDTH);
        if (ret != ESP_OK) { return ret; }
    }
    return ESP_OK;
}

static void oled_render_status_locked(system_status_t status)
{
    const char *text = status_to_text(status);
    size_t len = strlen(text);
    int scale = 2;
    int width = (int)len * 6 * scale;
    int x = (OLED_WIDTH - width) / 2;
    int y = (OLED_HEIGHT - (7 * scale)) / 2;
    if (x < 0) {
        x = 0;
    }
    if (y < 0) {
        y = 0;
    }

    oled_clear_buffer();
    oled_draw_text(text, x, y, scale);
    esp_err_t ret = oled_flush_buffer();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "oled_flush_buffer failed: %s", esp_err_to_name(ret));
    }
}

static void set_system_status(system_status_t status)
{
    current_status = status;
    if (!oled_initialized || (oled_mutex == NULL)) {
        return;
    }
    if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to lock OLED mutex");
        return;
    }
    oled_render_status_locked(status);
    xSemaphoreGive(oled_mutex);
}

static void set_system_error(const char *reason)
{
    ESP_LOGE(TAG, "%s", reason);
    set_system_status(SYSTEM_STATUS_ERROR);
}

static esp_err_t init_oled(void)
{
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = OLED_I2C_SDA_GPIO,
        .scl_io_num = OLED_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };

    esp_err_t ret = i2c_param_config(OLED_I2C_PORT, &i2c_cfg);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_driver_install(OLED_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    oled_mutex = xSemaphoreCreateMutex();
    if (oled_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    static const uint8_t init_seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x27, 0xD3, 0x00, 0xAD, 0x30, 0x8D, 0x14,
        0x40, 0xA6, 0xA4, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xAF,
        0xD9, 0x22, 0xDB, 0x20, 0x2E, 0xAF,
    };
    for (size_t i = 0; i < sizeof(init_seq); ++i) {
        ret = oled_write_command(init_seq[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    oled_initialized = true;
    set_system_status(SYSTEM_STATUS_BOOT);
    return ESP_OK;
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

static esp_err_t write_i2c_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t value)
{
    uint8_t payload[] = {reg_addr, value};
    return i2c_master_write_to_device(I2C_NUM_0, dev_addr, payload, sizeof(payload), pdMS_TO_TICKS(100));
}

static esp_err_t write_i2c_bulk_data(uint8_t dev_addr, const uint8_t *data)
{
    while (*data != 0) {
        uint8_t width = *data++;
        if (width != 2) {
            return ESP_ERR_INVALID_ARG;
        }
        esp_err_t ret = write_i2c_register(dev_addr, data[0], data[1]);
        if (ret != ESP_OK) {
            return ret;
        }
        data += 2;
    }
    return ESP_OK;
}

static esp_err_t set_microphone_enabled(bool enabled)
{
    static const uint8_t mic_enable_sequence[] = {
        2, 0x00, 0x80,
        2, 0x01, 0xBA,
        2, 0x02, 0x18,
        2, 0x0D, 0x01,
        2, 0x0E, 0x02,
        2, 0x14, 0x10,
        2, 0x17, 0xFF,
        2, 0x1C, 0x6A,
        0,
    };
    static const uint8_t mic_disable_sequence[] = {
        2, 0x0D, 0xFC,
        2, 0x0E, 0x6A,
        2, 0x00, 0x00,
        0,
    };

    return write_i2c_bulk_data(ES8311_I2C_ADDR, enabled ? mic_enable_sequence : mic_disable_sequence);
}

static void calc_clock_div(uint32_t *div_a, uint32_t *div_b, uint32_t *div_n, uint32_t base_clock, uint32_t target_freq)
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

static void apply_mic_clock_config(i2s_port_t port, uint32_t sample_rate_hz)
{
    static const uint32_t PLL_D2_CLK = 120 * 1000 * 1000;
    static const uint32_t bits_per_sample = 16;
    static const uint32_t div_m = 8;

    uint32_t div_a = 0;
    uint32_t div_b = 0;
    uint32_t div_n = 0;
    calc_clock_div(&div_a, &div_b, &div_n, PLL_D2_CLK / (bits_per_sample * div_m), sample_rate_hz);

    bool yn1 = (div_b > (div_a >> 1));
    if (yn1) {
        div_b = div_a - div_b;
    }
    int div_y = 1;
    int div_x = 0;
    if (div_b != 0) {
        div_x = div_a / div_b - 1;
        div_y = div_a % div_b;
        if (div_y == 0) {
            div_y = 1;
            div_b = 511;
        }
    }

    i2s_dev_t *dev = (port == I2S_NUM_1) ? &I2S1 : &I2S0;
    dev->rx_conf.rx_pdm_en = 0;
    dev->rx_conf.rx_tdm_en = 1;
#if defined(I2S_RX_PDM2PCM_CONF_REG)
    dev->rx_pdm2pcm_conf.rx_pdm2pcm_en = 0;
    dev->rx_pdm2pcm_conf.rx_pdm_sinc_dsr_16_en = 1;
#elif defined(I2S_RX_PDM2PCM_EN)
    dev->rx_conf.rx_pdm2pcm_en = 0;
    dev->rx_conf.rx_pdm_sinc_dsr_16_en = 1;
#endif
    dev->rx_conf.rx_update = 1;
    dev->rx_conf1.rx_bck_div_num = div_m - 1;
    i2s_ll_rx_set_raw_clk_div(dev, div_n, div_x, div_y, div_b, yn1);
    dev->rx_clkm_conf.rx_clkm_div_num = div_n;
    dev->rx_clkm_conf.rx_clk_sel = 1;
    dev->tx_clkm_conf.clk_en = 1;
    dev->rx_clkm_conf.rx_clk_active = 1;
    dev->rx_conf.rx_update = 1;
    dev->rx_conf.rx_update = 0;
    dev->rx_conf.rx_reset = 1;
    dev->rx_conf.rx_fifo_reset = 1;
    dev->rx_conf.rx_reset = 0;
    dev->rx_conf.rx_fifo_reset = 0;
}

// ============================================================================
// BLE GATT Server Callbacks
// ============================================================================

static int
audio_gatt_svr_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ble_uuid_cmp(ctxt->chr->uuid, &gatt_svr_rx_uuid.u) == 0) {
        ESP_LOGD(TAG, "Received control command, len=%d", OS_MBUF_PKTLEN(ctxt->om));
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
            }
        }
        return 0;
    }

    if (ble_uuid_cmp(ctxt->chr->uuid, &gatt_svr_tx_uuid.u) == 0) {
        ESP_LOGD(TAG, "TX characteristic accessed");
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
                0, /* No more characteristics */
            },
        },
    },
    {
        0, /* No more services */
    },
};

// ============================================================================
// BLE GAP Callbacks
// ============================================================================

static void
ble_app_advertise(void)
{
    ble_svc_gap_device_name_set(BLE_ADV_NAME);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    /* Adv holds flags + service UUID; full name is in the scan response
     * (31-byte adv limit cannot fit both "HarnessNode-Echo" and UUID128). */
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
        set_system_status(SYSTEM_STATUS_ERROR);
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
        set_system_status(SYSTEM_STATUS_ERROR);
        return;
    }

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, ble_app_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting advertising: %d", rc);
        set_system_status(SYSTEM_STATUS_ERROR);
        return;
    }

    ESP_LOGI(TAG, "Advertising started: %s", BLE_ADV_NAME);
    if (!is_recording) {
        set_system_status(SYSTEM_STATUS_READY);
    }
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
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE connection %s", event->connect.status == 0 ? "established" : "failed");
        if (event->connect.status != 0) {
            ble_app_advertise();
            return 0;
        }
        audio_conn_handle = event->connect.conn_handle;
        if (!is_recording) {
            set_system_status(SYSTEM_STATUS_CONNECTED);
        }

        ble_att_set_preferred_mtu(BLE_MTU_SIZE);
        ble_gattc_exchange_mtu(audio_conn_handle, NULL, NULL);
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected");
        audio_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        is_recording = false;
        recording_requested = false;
        stop_requested = false;
        stop_event_pending = false;
        apply_pending_operation_mode();
        set_system_status(SYSTEM_STATUS_READY);
        ble_app_advertise();
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU exchange: mtu=%d", event->mtu.value);
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

// ============================================================================
// Audio Initialization (I2C + ES8311 codec + I2S)
// ============================================================================

static esp_err_t init_audio(void)
{
    ESP_LOGI(TAG, "Initializing audio...");
    esp_err_t ret;

    // Step A: I2C master init (legacy driver, required by espressif/es8311)
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ret = i2c_param_config(I2C_NUM_0, &i2c_cfg);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret)); return ret; }
    ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret)); return ret; }
    ESP_LOGI(TAG, "I2C master initialized");
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1ULL << PA_EN_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&gpio_cfg);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(ret)); return ret; }
    gpio_set_level(PA_EN_GPIO, 0);

    // Step B: I2S RX channel init matching M5Unified Mic_Class pin usage.
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT_NUM, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = DMA_BUFFER_COUNT;
    chan_cfg.dma_frame_num = DMA_FRAME_COUNT;
    ret = i2s_new_channel(&chan_cfg, NULL, &i2s_rx_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(ret)); return ret; }

    i2s_std_config_t std_cfg = {0};
    std_cfg.clk_cfg.sample_rate_hz = 48000;
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_PLL_160M;
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_128;
    std_cfg.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;
    std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_STEREO;
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
    std_cfg.slot_cfg.ws_width = 16;
    std_cfg.slot_cfg.bit_shift = true;
    std_cfg.slot_cfg.left_align = true;
    std_cfg.slot_cfg.big_endian = false;
    std_cfg.slot_cfg.bit_order_lsb = false;
    std_cfg.gpio_cfg.bclk = I2S_BCLK_GPIO;
    std_cfg.gpio_cfg.ws = I2S_WS_GPIO;
    std_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.din = I2S_DIN_GPIO;
    std_cfg.gpio_cfg.mclk = I2S_MCLK_GPIO;
    ret = i2s_channel_init_std_mode(i2s_rx_handle, &std_cfg);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "i2s_channel_init_std_mode failed: %s", esp_err_to_name(ret)); return ret; }
    apply_mic_clock_config(I2S_PORT_NUM, I2S_SAMPLE_RATE * MIC_OVERSAMPLING);
    ret = i2s_channel_enable(i2s_rx_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "i2s_channel_enable failed: %s", esp_err_to_name(ret)); return ret; }

    // Step C: ES8311 microphone path init using the same register sequence as M5Unified.
    ret = set_microphone_enabled(true);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "set_microphone_enabled failed: %s", esp_err_to_name(ret)); return ret; }
    ESP_LOGI(TAG, "Microphone path initialized");

    return ESP_OK;
}

static esp_err_t init_button(void)
{
    gpio_config_t button_cfg = {
        .pin_bit_mask = 1ULL << BUTTON_A_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&button_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "button gpio_config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Button A initialized on GPIO %d (active low)", BUTTON_A_GPIO);
    return ESP_OK;
}

// ============================================================================
// Audio Streaming Task
// ============================================================================

static void audio_stream_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Audio streaming task started");

    // Initialize audio hardware
    esp_err_t audio_ret = init_audio();
    if (audio_ret != ESP_OK) {
        set_system_error("Audio init failed");
        ESP_LOGE(TAG, "Audio init failed (%s) — audio task exiting", esp_err_to_name(audio_ret));
        vTaskDelete(NULL);
        return;
    }

    // Allocate buffers
    size_t i2s_buffer_bytes = DMA_FRAME_COUNT * sizeof(int16_t);
    int16_t *i2s_buffer = (int16_t *)malloc(i2s_buffer_bytes);
    int16_t *pcm_buffer = (int16_t *)malloc(i2s_buffer_bytes / MIC_OVERSAMPLING);
    uint8_t *tx_packet = (uint8_t *)malloc(RECORDING_PACKET_SIZE + 2);  // +2 for header

    if (i2s_buffer == NULL || pcm_buffer == NULL || tx_packet == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffers");
        vTaskDelete(NULL);
    }

    size_t bytes_read;
    int32_t prev_value[2] = {0, 0};
    int32_t offset = 0;

    // Match Mic_Class warm-up reads before actual capture.
    (void)i2s_channel_read(i2s_rx_handle, i2s_buffer, i2s_buffer_bytes, &bytes_read, pdMS_TO_TICKS(1));
    (void)i2s_channel_read(i2s_rx_handle, i2s_buffer, i2s_buffer_bytes, &bytes_read, pdMS_TO_TICKS(1));

    ESP_LOGI(TAG, "Waiting for audio data...");

    while (1) {
        // Check for recording start command
        if (recording_requested && !is_recording) {
            ESP_LOGI(TAG, "Starting recording...");
            is_recording = true;
            recording_requested = false;
            seq_num = 0;
            set_system_status(SYSTEM_STATUS_RECORDING);
            send_status_event(EVENT_RECORDING_STARTED, "recording_started");
        }

        // Check for recording stop command
        if (stop_requested) {
            if (stop_event_pending) {
                ESP_LOGI(TAG, "Stopping recording...");
                set_system_status(audio_conn_handle == BLE_HS_CONN_HANDLE_NONE ? SYSTEM_STATUS_READY : SYSTEM_STATUS_CONNECTED);
                send_status_event(EVENT_RECORDING_STOPPED, "recording_stopped");
            }
            stop_requested = false;
            stop_event_pending = false;
            apply_pending_operation_mode();
        }

        // Read from I2S RX
        esp_err_t ret = i2s_channel_read(i2s_rx_handle, i2s_buffer, i2s_buffer_bytes,
                                         &bytes_read, portMAX_DELAY);

        if (ret != ESP_OK || bytes_read == 0) {
            continue;
        }

        if (stop_requested || !is_recording || audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            continue;
        }

        // Match Mic_Class processing: stereo input, zero-level adjustment, optional noise filter, mono output.
        size_t samples_read = 0;
        size_t raw_samples = bytes_read / sizeof(int16_t);
        float f_gain = (float)MIC_MAGNIFICATION / (MIC_OVERSAMPLING << 1);
        int os_remain = MIC_OVERSAMPLING;
        int32_t sum_value[2] = {0, 0};

        for (size_t i = 0; i + 1 < raw_samples; ) {
            do {
                sum_value[0] += i2s_buffer[i++];
                sum_value[1] += i2s_buffer[i++];
            } while (--os_remain && (i + 1 < raw_samples));

            if (os_remain) {
                continue;
            }
            os_remain = MIC_OVERSAMPLING;

            int32_t value_tmp = (sum_value[0] + sum_value[1]) << 3;
            offset -= (value_tmp + offset + 16) >> 5;
            int32_t shifted_offset = (offset + 8) >> 4;
            sum_value[0] += shifted_offset;
            sum_value[1] += shifted_offset;

            if (MIC_NOISE_FILTER) {
                for (int ch = 0; ch < 2; ++ch) {
                    int32_t v = (sum_value[ch] * (256 - MIC_NOISE_FILTER) + prev_value[ch] * MIC_NOISE_FILTER + 128) >> 8;
                    prev_value[ch] = v;
                    sum_value[ch] = (int32_t)(v * f_gain);
                }
            } else {
                sum_value[0] = (int32_t)(sum_value[0] * f_gain);
                sum_value[1] = (int32_t)(sum_value[1] * f_gain);
            }

            int32_t mono = (sum_value[0] + sum_value[1] + 1) >> 1;
            if (mono < INT16_MIN + 16) {
                mono = INT16_MIN + 16;
            } else if (mono > INT16_MAX - 16) {
                mono = INT16_MAX - 16;
            }
            pcm_buffer[samples_read++] = (int16_t)mono;
            sum_value[0] = 0;
            sum_value[1] = 0;
        }

        // Process and send PCM data via BLE
        size_t sample_index = 0;

        while (sample_index < samples_read) {
            size_t samples_to_send = (samples_read - sample_index < RECORDING_PACKET_SIZE / 2) ?
                                     (samples_read - sample_index) : (RECORDING_PACKET_SIZE / 2);

            // Create packet: [seq_num][0xAA][PCM data...]
            tx_packet[0] = seq_num++;
            tx_packet[1] = AUDIO_PACKET_SYNC_BYTE;

            memcpy(&tx_packet[2], &pcm_buffer[sample_index], samples_to_send * 2);

            size_t packet_size = 2 + (samples_to_send * 2);

            struct os_mbuf *om = ble_hs_mbuf_from_flat(tx_packet, packet_size);
            if (om != NULL) {
                ble_gattc_notify_custom(audio_conn_handle, audio_tx_char_handle, om);
            }

            sample_index += samples_to_send;

            // Small delay to avoid overwhelming BLE
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    free(i2s_buffer);
    free(pcm_buffer);
    free(tx_packet);
    vTaskDelete(NULL);
}

static void button_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Button task started");

    int stable_level = gpio_get_level(BUTTON_A_GPIO);
    int last_level = stable_level;
    TickType_t last_change_tick = xTaskGetTickCount();
    TickType_t first_click_tick = 0;
    uint8_t click_count = 0;

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
            }
        }

        if (click_count == 1 &&
            (now - first_click_tick) > pdMS_TO_TICKS(BUTTON_DOUBLE_CLICK_MS)) {
            click_count = 0;
            first_click_tick = 0;

            emulate_single_click("Button A");
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

// ============================================================================
// UART Task (for serial commands)
// ============================================================================

static void uart_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Serial command task started");

    uint8_t uart_buf[256];
    uint8_t usb_buf[64];

    while (1) {
        int usb_len = usb_serial_jtag_read_bytes(usb_buf, sizeof(usb_buf), pdMS_TO_TICKS(20));
        if (usb_len > 0) {
            for (int i = 0; i < usb_len; i++) {
                handle_serial_command(usb_buf[i], "USB Serial/JTAG");
            }
        }

        int len = uart_read_bytes(UART_PORT_NUM, uart_buf, sizeof(uart_buf), pdMS_TO_TICKS(100));

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                handle_serial_command(uart_buf[i], "UART");
            }
        }
    }
}

// ============================================================================
// BLE Host Task
// ============================================================================

static void
ble_host_task(void *pvParameters)
{
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
}

// ============================================================================
// Application Entry Point
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "Voice Bridge BLE (Atom Echo S3R) - Starting");

    // Initialize ADPCM encoder state (for future use)
    adpcm_init_state(&adpcm_enc_state);

    // Initialize UART
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
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));

    ESP_LOGI(TAG, "UART and USB Serial/JTAG initialized");

    esp_err_t oled_ret = init_oled();
    if (oled_ret != ESP_OK) {
        ESP_LOGW(TAG, "Mini OLED init failed: %s", esp_err_to_name(oled_ret));
    }

    esp_err_t button_ret = init_button();
    if (button_ret != ESP_OK) {
        set_system_error("Button init failed");
        ESP_ERROR_CHECK(button_ret);
    }

    // Initialize BLE
    ESP_LOGI(TAG, "Initializing NimBLE...");

    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;
    ble_hs_cfg.gatts_register_cb = NULL;
    ble_hs_cfg.store_status_cb = NULL;

    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;

    ESP_LOGI(TAG, "Calling nimble_port_init...");
    nimble_port_init();

    ESP_LOGI(TAG, "Initializing GATT server...");
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    ESP_LOGI(TAG, "Starting BLE host task...");
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "Creating audio streaming task...");
    xTaskCreatePinnedToCore(audio_stream_task, "audio_stream", 8192, NULL, 5,
                           &audio_task_handle, 1);

    ESP_LOGI(TAG, "Creating button task...");
    xTaskCreate(button_task, "button_task", 3072, NULL, 4, &button_task_handle);

    ESP_LOGI(TAG, "Creating UART task...");
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "Voice Bridge BLE - Initialization complete");
    ESP_LOGI(TAG,
             "Adv name %s; single-click toggles recording; serial: "
             "'r'=start, 's'=stop, 'c'/'1'=single-click, 'd'/'2'=double-click, 'h'=help",
             BLE_ADV_NAME);
}
