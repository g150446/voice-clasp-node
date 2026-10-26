#ifndef HARNESS_TAP_DETECTOR_H
#define HARNESS_TAP_DETECTOR_H

#include <stdbool.h>
#include <stdint.h>

/* Fixed 416 Hz input, acceleration in g, orientation-independent.
 * A tap is a short, sharp impulse of acceleration MAGNITUDE regardless of
 * which board axis it lands on, so the classifier works at any wrist angle.
 * Known limit: at +/-4 g a maximal wrist jerk and a hard tap both ride the
 * ceiling (measured 3.997 g vs 3.996 g) and are not separable. Widening to
 * +/-8 g was rejected because the shared accelerometer feeds the gesture path,
 * whose sub-0.05 g thresholds fall below an 8 g LSB. Deliberate taps peak
 * 4.97-7.57 g and normal/brisk motion stays <=0.73 g, so everyday accuracy is
 * unaffected; only an all-out jerk can false-trigger. */
#define TAP_SAMPLE_HZ 416
enum tap_result { TAP_NONE, TAP_SINGLE, TAP_DOUBLE };

/* `dominance` is retained for wire/format compatibility but no longer used:
 * an axis-dominance gate rejected real taps whose impulse was lateral, which
 * was the root cause of posture-dependent double-tap misses. */
struct tap_config {
    float quiet_g, peak_g, edge_g, dominance, gravity_alpha, noise_multiplier;
    uint16_t pre_quiet_ms, max_rise_ms, post_quiet_ms, max_pulse_ms;
    uint16_t min_gap_ms, max_gap_ms, cooldown_ms, suppress_ms;
};
#define TAP_CONFIG_DEFAULT {0.50f, 1.0f, 0.30f, 0.0f, 0.015f, 4.0f, \
                           15, 15, 20, 40, 80, 350, 700, 1500}
extern const struct tap_config tap_default_config;
bool tap_config_valid(const struct tap_config *c);

struct tap_detector {
    uint64_t sample, pulse_start, first_strike, last_strike, blocked_until;
    uint64_t single_suppress_until;
    uint64_t last_quiet_sample;
    float gravity[3], noise, previous_magnitude, peak_magnitude;
    unsigned warmup, quiet, pulse_quiet;
    bool initialized, pulse, pulse_rejected, pending;
};

/* Reset on FIFO loss: never classify two strikes across missing data. */
void tap_detector_reset(struct tap_detector *d);
enum tap_result tap_detector_push(struct tap_detector *d, float x, float y, float z);
enum tap_result tap_detector_push_config(struct tap_detector *d, float x, float y,
                                         float z, const struct tap_config *c);

#endif
