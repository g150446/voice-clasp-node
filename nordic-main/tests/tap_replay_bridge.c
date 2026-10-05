/* PC replay uses the production detector and the same raw conversion. */
#include "tap_detector.h"
#include <stddef.h>

int tap_replay(const int16_t *raw, const uint16_t *input_flags, size_t count,
               const struct tap_config *config, uint16_t *output_flags)
{
    if (!tap_config_valid(config)) return -1;
    struct tap_detector d;
    tap_detector_reset(&d);
    for (size_t i = 0; i < count; i++) {
        if (input_flags[i] & 64) tap_detector_reset(&d);
        uint64_t previous = d.last_strike;
        enum tap_result result = tap_detector_push_config(&d,
            raw[3*i] * 0.000122f, raw[3*i+1] * 0.000122f,
            raw[3*i+2] * 0.000122f, config);
        output_flags[i] = (d.pulse ? 1 : 0) | (d.pulse_rejected ? 2 : 0) |
            (d.pending ? 4 : 0) | (d.last_strike != previous ? 8 : 0) |
            (result == TAP_SINGLE ? 16 : 0) | (result == TAP_DOUBLE ? 32 : 0) |
            (input_flags[i] & (64 | 128));
    }
    return 0;
}
