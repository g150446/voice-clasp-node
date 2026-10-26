#include "tap_processing.h"
#include <math.h>
#include <stddef.h>
enum tap_result tap_process_sample(struct tap_detector *d, const struct tap_config *cfg,
                                   const int16_t raw[3], struct tap_metrics *metrics,
                                   uint16_t *flags)
{
    float accel[3] = {raw[0] * TAP_RAW_TO_G, raw[1] * TAP_RAW_TO_G, raw[2] * TAP_RAW_TO_G};
    bool was_pulse = false, was_rejected = false;
    uint64_t previous_strike = 0;
    if (metrics || flags) {
        previous_strike = d->last_strike;
        if (metrics) {
            was_pulse = d->pulse;
            was_rejected = d->pulse_rejected;
            if (d->initialized) {
                float magnitude = 0.0f;
                for (unsigned axis = 0; axis < 3; axis++) {
                    float linear = accel[axis] - d->gravity[axis];
                    metrics->peak[axis] = fmaxf(metrics->peak[axis], fabsf(linear));
                    magnitude += linear * linear;
                }
                magnitude = sqrtf(magnitude);
                metrics->edge = fmaxf(metrics->edge, fabsf(magnitude - d->previous_magnitude));
            }
            metrics->samples++;
        }
    }
    enum tap_result result = tap_detector_push_config(d, accel[0], accel[1], accel[2], cfg);
    if (metrics) {
        if (!was_pulse && d->pulse) metrics->candidates++;
        if (!was_rejected && d->pulse_rejected) metrics->broad++;
        if (d->last_strike != previous_strike) metrics->strikes++;
        if (was_pulse && !d->pulse && !d->pulse_rejected &&
            d->last_strike == previous_strike) metrics->lateral++;
    }
    if (flags) {
        *flags = (d->pulse ? 1 : 0) | (d->pulse_rejected ? 2 : 0) |
            (d->pending ? 4 : 0) | (d->last_strike != previous_strike ? 8 : 0) |
            (result == TAP_SINGLE ? 16 : 0) | (result == TAP_DOUBLE ? 32 : 0);
        for (unsigned axis = 0; axis < 3; axis++) {
            if (raw[axis] >= 32760 || raw[axis] <= -32760) *flags |= 128;
        }
    }
    return result;
}
