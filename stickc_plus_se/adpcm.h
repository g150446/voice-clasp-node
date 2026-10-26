#ifndef ADPCM_H
#define ADPCM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// IMA ADPCM state structure
typedef struct {
    int32_t predictor;
    int32_t step_size;
} adpcm_state_t;

// Initialize ADPCM state
void adpcm_init_state(adpcm_state_t *state);

// Encode 16-bit PCM to 4-bit ADPCM
// input: 16-bit PCM samples
// num_samples: number of input samples
// output: 4-bit ADPCM data (packed, 2 samples per byte)
// Returns: number of bytes written to output
size_t adpcm_encode(adpcm_state_t *state, const int16_t *input, 
                    size_t num_samples, uint8_t *output);

// Decode 4-bit ADPCM to 16-bit PCM
// input: 4-bit ADPCM data (packed)
// num_bytes: number of input bytes (contains num_bytes * 2 samples)
// output: 16-bit PCM samples
// Returns: number of samples written to output
size_t adpcm_decode(adpcm_state_t *state, const uint8_t *input,
                    size_t num_bytes, int16_t *output);

#ifdef __cplusplus
}
#endif

#endif // ADPCM_H
