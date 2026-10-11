/*
 * Audio Capture for Voice Bridge BLE
 * PDM Microphone via Zephyr DMIC API on nRF52840
 *
 * Ported from nrf54l15/src/audio_capture.c:
 *   - DT alias: dmic20 → dmic0
 *   - Fallback node: pdm20 → pdm0
 *   - PDM CLK: P1.12 → P1.00
 *   - PDM DIN: P1.13 → P0.16  (configured in board overlay)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>

#include "audio_capture.h"
#include "app_diagnostics.h"

LOG_MODULE_REGISTER(audio_capture, LOG_LEVEL_INF);

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define SAMPLE_RATE         16000
#define SAMPLE_BITS         16
#define CHANNELS            1
#define BYTES_PER_SAMPLE    sizeof(int16_t)

/* 20ms frame = 320 samples = 640 bytes */
#define FRAME_MS            20
#define FRAME_SAMPLES       (SAMPLE_RATE * FRAME_MS / 1000)
#define FRAME_BYTES         (FRAME_SAMPLES * BYTES_PER_SAMPLE * CHANNELS)

/*
 * DMIC driver slab. Keep well above one BLE backpressure stall so the PDM
 * DMA path can continue while the sender thread drains Notify slots.
 * 12 × 20 ms = 240 ms of hardware buffering.
 */
#define BLOCK_COUNT         12
K_MEM_SLAB_DEFINE_STATIC(pdm_mem_slab, FRAME_BYTES, BLOCK_COUNT, 4);

/* ============================================================================
 * Global Variables
 * ============================================================================ */

static const struct device *dmic_dev;
static bool dmic_configured = false;
static bool dmic_started = false;

/* Current audio buffer from dmic_read */
static void *current_block = NULL;
static bool pdm_hw_running = false;

/* ============================================================================
 * Public API
 * ============================================================================ */

int audio_capture_init(void)
{
    if (dmic_configured) {
        return 0;
    }

#ifdef DT_N_ALIAS_dmic0
    dmic_dev = DEVICE_DT_GET(DT_ALIAS(dmic0));
#else
    dmic_dev = DEVICE_DT_GET(DT_NODELABEL(pdm0));
#endif
    if (!device_is_ready(dmic_dev)) {
        LOG_ERR("DMIC device not ready: %s", dmic_dev->name);
        return -ENODEV;
    }

    struct pcm_stream_cfg stream = {
        .pcm_width = SAMPLE_BITS,
        .pcm_rate  = SAMPLE_RATE,
        .block_size = FRAME_BYTES,
        .mem_slab  = &pdm_mem_slab,
    };

    struct dmic_cfg cfg = {
        .io = {
            .min_pdm_clk_freq = 1000000,
            .max_pdm_clk_freq = 3500000,
            .min_pdm_clk_dc   = 40,
            .max_pdm_clk_dc   = 60,
        },
        .streams = &stream,
        .channel = {
            .req_num_streams = 1,
            .req_num_chan = CHANNELS,
            .req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT),
        },
    };

    int ret = dmic_configure(dmic_dev, &cfg);
    if (ret < 0) {
        LOG_ERR("DMIC configure failed: %d", ret);
        return ret;
    }

    dmic_configured = true;

    APP_INFO("Audio capture initialized: %d Hz, %d-bit, %d ch, %d ms frames",
            SAMPLE_RATE, SAMPLE_BITS, CHANNELS, FRAME_MS);
    APP_TRACE("Audio init OK: %d Hz, %d samples/frame\n", SAMPLE_RATE, FRAME_SAMPLES);
    return 0;
}

int audio_capture_start(void)
{
    if (!dmic_configured) {
        return -EIO;
    }

    if (dmic_started) {
        return 0;
    }

    int ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
    if (ret < 0) {
        LOG_ERR("DMIC start failed: %d", ret);
        return ret;
    }
    pdm_hw_running = true;

    /*
     * Discard frames until the PDM CIC filter's DC offset settles.
     * Fresh start produces a large negative offset (~-30000) that decays
     * ~7.5% per 20ms frame, converging to the mic's bias (~-8).
     * Discard up to 80 frames (1.6s), stopping early when |DC| < 200.
     */
    for (int i = 0; i < 80; i++) {
        void *blk = NULL;
        uint32_t sz;
        if (dmic_read(dmic_dev, 0, &blk, &sz, 100) < 0 || blk == NULL) {
            break;
        }
        int16_t *s = (int16_t *)blk;
        int32_t dc = 0;
        uint32_t n = sz / sizeof(int16_t);
        for (uint32_t j = 0; j < n; j++) {
            dc += s[j];
        }
        dc /= (int32_t)n;
        k_mem_slab_free(&pdm_mem_slab, blk);
        if (dc > -200 && dc < 200) {
            APP_TRACE("PDM settled after %d frames (DC=%d)\n", i + 1, (int)dc);
            break;
        }
    }

    dmic_started = true;
    APP_INFO("Audio capture started");
    APP_TRACE("Audio started\n");
    return 0;
}

int audio_capture_stop(void)
{
    if (!dmic_started) {
        return 0;
    }

    int ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
    if (ret < 0) {
        LOG_ERR("DMIC stop failed: %d", ret);
        return ret;
    }
    pdm_hw_running = false;

    if (current_block) {
        k_mem_slab_free(&pdm_mem_slab, current_block);
        current_block = NULL;
    }

    dmic_started = false;
    APP_INFO("Audio capture stopped");
    return 0;
}

int audio_capture_get_data(int16_t **buffer, size_t *size)
{
    if (!dmic_started) {
        return -EAGAIN;
    }

    if (current_block) {
        k_mem_slab_free(&pdm_mem_slab, current_block);
        current_block = NULL;
    }

    uint32_t read_size;
    int ret = dmic_read(dmic_dev, 0, &current_block, &read_size, 200);
    if (ret < 0) {
        LOG_WRN("DMIC read failed: %d", ret);
        return ret;
    }

    if (buffer) {
        *buffer = (int16_t *)current_block;
    }
    if (size) {
        *size = read_size;
    }

    return 0;
}

size_t audio_capture_get_frame_size(void)
{
    return FRAME_BYTES;
}

uint32_t audio_capture_get_sample_rate(void)
{
    return SAMPLE_RATE;
}
