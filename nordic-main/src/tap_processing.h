#ifndef TAP_PROCESSING_H
#define TAP_PROCESSING_H
#include "tap_detector.h"
#define TAP_RAW_TO_G 0.000122f  /* +/-4 g full scale -> 0.122 mg/LSB (shared with gesture path) */
struct tap_metrics {
    uint32_t samples, discontinuities, candidates, strikes, broad, lateral;
    float peak[3], edge;
};
/* NULL metrics/flags remove diagnostic work without changing detector state. */
enum tap_result tap_process_sample(struct tap_detector *d, const struct tap_config *cfg,
                                   const int16_t raw[3], struct tap_metrics *metrics,
                                   uint16_t *flags);
#endif
