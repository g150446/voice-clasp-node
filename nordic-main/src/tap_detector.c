#include "tap_detector.h"

#include <math.h>
#include <string.h>

#define SAMPLES(ms) (((ms) * TAP_SAMPLE_HZ + 999) / 1000)
#define WARMUP SAMPLES(500)
const struct tap_config tap_default_config = TAP_CONFIG_DEFAULT;

bool tap_config_valid(const struct tap_config *c)
{
    return isfinite(c->quiet_g) && c->quiet_g >= 0.05f && c->quiet_g <= 1.0f &&
        isfinite(c->peak_g) && c->peak_g > c->quiet_g && c->peak_g <= 4.0f &&
        isfinite(c->edge_g) && c->edge_g >= 0.05f && c->edge_g <= 4.0f &&
        isfinite(c->dominance) && c->dominance >= 0.0f && c->dominance <= 4.0f &&
        isfinite(c->gravity_alpha) && c->gravity_alpha >= 0.001f && c->gravity_alpha <= 0.1f &&
        isfinite(c->noise_multiplier) && c->noise_multiplier >= 1.0f && c->noise_multiplier <= 10.0f &&
        c->pre_quiet_ms >= 5 && c->pre_quiet_ms <= 200 &&
        c->max_rise_ms >= 5 && c->max_rise_ms <= 100 &&
        c->post_quiet_ms >= 5 && c->post_quiet_ms <= 200 &&
        c->max_pulse_ms >= 5 && c->max_pulse_ms <= 300 &&
        c->min_gap_ms >= 30 && c->min_gap_ms < c->max_gap_ms &&
        c->max_gap_ms <= 700 && c->cooldown_ms >= 100 && c->cooldown_ms <= 3000 &&
        c->suppress_ms >= c->cooldown_ms && c->suppress_ms <= 5000;
}

void tap_detector_reset(struct tap_detector *d)
{
    memset(d, 0, sizeof(*d));
}

enum tap_result tap_detector_push_config(struct tap_detector *d, float x, float y, float z,
                                         const struct tap_config *c)
{
    const float a[3] = {x, y, z};
    float linear[3];
    enum tap_result result = TAP_NONE;
    d->sample++;
    if (!d->initialized) {
        memcpy(d->gravity, a, sizeof(a));
        d->initialized = true;
    }
    for (unsigned i = 0; i < 3; i++) {
        linear[i] = a[i] - d->gravity[i];
        d->gravity[i] += c->gravity_alpha * linear[i];
    }
    /* Orientation-independent impact: the magnitude of the gravity-free
     * acceleration, not any single board axis. The previous axis-dominance
     * gate rejected real taps whose impulse was lateral (posture-dependent
     * double-tap misses), so it is gone; tap-vs-motion separation now rests on
     * the rise rate, edge, peak gate, and pulse-width cap. */
    float magnitude = sqrtf(linear[0] * linear[0] + linear[1] * linear[1] +
                            linear[2] * linear[2]);
    float edge = fabsf(magnitude - d->previous_magnitude);
    d->previous_magnitude = magnitude;
    float threshold = fmaxf(c->peak_g, c->noise_multiplier * d->noise);
    /* Slow background estimate; a short strike must not raise its own gate. */
    if (!d->pulse) {
        d->noise += 0.005f * (fminf(magnitude, 1.0f) - d->noise);
    }
    if (d->quiet >= SAMPLES(c->pre_quiet_ms)) {
        d->last_quiet_sample = d->sample - 1;
    }
    d->quiet = magnitude < c->quiet_g ? d->quiet + (d->quiet < WARMUP) : 0;
    if (d->warmup < WARMUP) {
        d->warmup++;
        return TAP_NONE;
    }

    if (!d->pulse && d->sample >= d->blocked_until &&
        d->last_quiet_sample != 0 &&
        d->sample - d->last_quiet_sample <= SAMPLES(c->max_rise_ms) &&
        magnitude >= threshold && edge >= c->edge_g &&
        (d->last_strike == 0 || d->sample - d->last_strike >= SAMPLES(c->min_gap_ms))) {
        d->pulse = true;
        d->pulse_rejected = false;
        /* A real impulse rises through the quiet threshold BEFORE reaching
         * the peak gate. Retain the preceding quiet window briefly, and count
         * the rise in pulse width rather than demanding a one-sample jump. */
        d->pulse_start = d->last_quiet_sample + 1;
        d->pulse_quiet = 0;
        d->peak_magnitude = magnitude;
    }
    if (d->pulse) {
        d->peak_magnitude = fmaxf(d->peak_magnitude, magnitude);
        d->pulse_quiet = magnitude < c->quiet_g ? d->pulse_quiet + 1 : 0;
        if (d->sample - d->pulse_start - d->pulse_quiet > SAMPLES(c->max_pulse_ms)) {
            d->pulse_rejected = true;
        }
        if (d->pulse_quiet >= SAMPLES(c->post_quiet_ms)) {
            /* The pulse closes once the magnitude returns below the quiet
             * gate; only the width cap can invalidate an impact. */
            bool valid = !d->pulse_rejected;
            d->pulse = false;
            if (valid) {
                uint64_t strike = d->pulse_start;
                d->last_strike = strike;
                if (d->pending && strike - d->first_strike >= SAMPLES(c->min_gap_ms) &&
                    strike - d->first_strike <= SAMPLES(c->max_gap_ms)) {
                    d->pending = false;
                    d->blocked_until = d->sample + SAMPLES(c->cooldown_ms);
                    d->single_suppress_until = d->sample + SAMPLES(c->suppress_ms);
                    result = TAP_DOUBLE;
                } else if (!d->pending) {
                    d->pending = true;
                    d->first_strike = strike;
                }
            }
        }
    }
    /* Allow a second strike at the edge of SAMPLES(c->max_gap_ms) to finish before single.
     * Waiting uses sample time, unaffected by FIFO batch / main-loop jitter. */
    if (d->pending && d->sample - d->first_strike > SAMPLES(c->max_gap_ms) &&
        (!d->pulse || d->pulse_rejected ||
         d->pulse_start - d->first_strike > SAMPLES(c->max_gap_ms))) {
        d->pending = false;
        if (d->sample >= d->single_suppress_until) {
            d->blocked_until = d->sample + SAMPLES(c->cooldown_ms);
            result = TAP_SINGLE;
        }
    }
    return result;
}

enum tap_result tap_detector_push(struct tap_detector *d, float x, float y, float z)
{
    return tap_detector_push_config(d, x, y, z, &tap_default_config);
}
