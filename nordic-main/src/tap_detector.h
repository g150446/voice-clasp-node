#ifndef HARNESS_TAP_DETECTOR_H
#define HARNESS_TAP_DETECTOR_H

#include <stdbool.h>
#include <stdint.h>

/* Fixed 416 Hz input, acceleration in g, board-normal Z axis. */
#define TAP_SAMPLE_HZ 416
enum tap_result { TAP_NONE, TAP_SINGLE, TAP_DOUBLE };

struct tap_config {
    float quiet_g, peak_g, edge_g, dominance, gravity_alpha, noise_multiplier;
    uint16_t pre_quiet_ms, max_rise_ms, post_quiet_ms, max_pulse_ms;
    uint16_t min_gap_ms, max_gap_ms, cooldown_ms, suppress_ms;
};
#define TAP_CONFIG_DEFAULT {0.35f, 1.0f, 0.30f, 1.2f, 0.015f, 4.0f, \
                           15, 15, 20, 40, 80, 350, 700, 1500}
extern const struct tap_config tap_default_config;
bool tap_config_valid(const struct tap_config *c);

struct tap_detector {
    uint64_t sample, pulse_start, first_strike, last_strike, blocked_until;
    uint64_t single_suppress_until;
    uint64_t last_quiet_sample;
    float gravity[3], noise, previous_z, peak_z, peak_lateral;
    unsigned warmup, quiet, pulse_quiet;
    bool initialized, pulse, pulse_rejected, pending;
};

/* Reset on FIFO loss: never classify two strikes across missing data. */
void tap_detector_reset(struct tap_detector *d);
enum tap_result tap_detector_push(struct tap_detector *d, float x, float y, float z);
enum tap_result tap_detector_push_config(struct tap_detector *d, float x, float y,
                                         float z, const struct tap_config *c);

#endif
