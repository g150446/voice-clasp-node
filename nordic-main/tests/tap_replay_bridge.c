/* PC replay uses the production detector and the same raw conversion. */
#include "tap_processing.h"
#include <stddef.h>

int tap_replay(const int16_t *raw, const uint16_t *input_flags, size_t count,
               const struct tap_config *config, uint16_t *output_flags)
{
    if (!tap_config_valid(config)) return -1;
    struct tap_detector d;
    tap_detector_reset(&d);
    for (size_t i = 0; i < count; i++) {
        if (input_flags[i] & 64) tap_detector_reset(&d);
        uint16_t flags;
        tap_process_sample(&d, config, raw + 3*i, NULL, &flags);
        output_flags[i] = flags | (input_flags[i] & (64 | 128));
    }
    return 0;
}
