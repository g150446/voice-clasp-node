#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "tap_processing.h"
int main(void) {
    struct tap_detector normal, debug;
    struct tap_metrics metrics = {0}, frozen;
    struct tap_config cfg = TAP_CONFIG_DEFAULT;
    tap_detector_reset(&normal); tap_detector_reset(&debug);
    unsigned doubles = 0, singles = 0;
    for (unsigned i = 0; i < 1700; i++) {
        /* double then single; switch diagnostics during a pending pair */
        int16_t raw[3] = {0, 0, 8197};
        if (i == 300 || i == 301 || i == 385 || i == 386 || i == 1400 || i == 1401) raw[2] = 21311;
        bool enabled = i < 340 || i >= 700;
        if (i == 340) frozen = metrics;
        uint16_t flags;
        enum tap_result a = tap_process_sample(&normal, &cfg, raw, NULL, NULL);
        enum tap_result b = tap_process_sample(&debug, &cfg, raw, enabled ? &metrics : NULL, enabled ? &flags : NULL);
        assert(a == b);
        assert(memcmp(&normal, &debug, sizeof(normal)) == 0);
        if (i >= 340 && i < 700) assert(memcmp(&metrics, &frozen, sizeof(metrics)) == 0);
        if (enabled) assert(((flags & 16) != 0) == (a == TAP_SINGLE));
        if (enabled) assert(((flags & 32) != 0) == (a == TAP_DOUBLE));
        if (a == TAP_DOUBLE) doubles++;
        if (a == TAP_SINGLE) singles++;
    }
    assert(doubles == 1 && singles == 1);
    assert(metrics.samples == 1340);
    puts("PASS: normal/debug event and state parity; metrics freeze across mode changes");
}
