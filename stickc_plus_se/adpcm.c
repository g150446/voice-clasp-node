#include "adpcm.h"
#include <string.h>

// IMA ADPCM step size table
static const int32_t step_size_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31,
    34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143,
    157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544,
    598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878,
    2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894,
    6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818,
    18500, 20350, 22385, 24623, 27086, 29794, 32767
};

// Step size adjustment table for each ADPCM value
static const int8_t step_adjust_table[8] = {
    -1, -1, -1, -1, 2, 4, 6, 8
};

void adpcm_init_state(adpcm_state_t *state)
{
    state->predictor = 0;
    state->step_size = 7;  // Initial step size
}

static int32_t clamp(int32_t value, int32_t min, int32_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

size_t adpcm_encode(adpcm_state_t *state, const int16_t *input,
                    size_t num_samples, uint8_t *output)
{
    if (num_samples == 0 || input == NULL || output == NULL) {
        return 0;
    }
    
    int32_t predictor = state->predictor;
    int32_t step_size = state->step_size;
    
    size_t output_index = 0;
    uint8_t current_byte = 0;
    
    for (size_t i = 0; i < num_samples; i++) {
        // Calculate difference
        int32_t diff = (int32_t)input[i] - predictor;
        int8_t sign = (diff < 0) ? 1 : 0;
        
        if (diff < 0) {
            diff = -diff;
        }
        
        // Calculate ADPCM value (0-7)
        int8_t adpcm_value = 0;
        int32_t delta = step_size >> 3;  // step_size / 8
        
        if (diff > delta) {
            adpcm_value = 1;
            diff -= delta;
            delta = step_size >> 2;  // step_size / 4
            
            if (diff > delta) {
                adpcm_value = 2;
                diff -= delta;
                delta = step_size >> 1;  // step_size / 2
                
                if (diff > delta) {
                    adpcm_value = 3;
                    diff -= delta;
                    delta = step_size;
                    
                    if (diff > delta) {
                        adpcm_value = 4;
                        diff -= delta;
                    }
                }
            }
        }
        
        // Combine sign and value
        adpcm_value = (sign << 3) | adpcm_value;
        
        // Update predictor
        int32_t step = step_size;
        if (adpcm_value & 0x04) step += step_size >> 2;
        if (adpcm_value & 0x02) step += step_size >> 1;
        if (adpcm_value & 0x01) step += step_size >> 3;
        
        if (sign) {
            predictor -= step;
        } else {
            predictor += step;
        }
        
        // Clamp predictor to 16-bit range
        predictor = clamp(predictor, -32768, 32767);
        
        // Update step size
        int8_t index_change = step_adjust_table[adpcm_value & 0x07];
        int32_t new_step_index = 0;
        
        // Find current step index
        for (int32_t j = 0; j < 89; j++) {
            if (step_size_table[j] >= step_size) {
                new_step_index = j;
                break;
            }
        }
        
        new_step_index += index_change;
        new_step_index = clamp(new_step_index, 0, 88);
        step_size = step_size_table[new_step_index];
        
        // Pack two 4-bit ADPCM values into one byte
        if (i % 2 == 0) {
            current_byte = (adpcm_value & 0x0F);
        } else {
            current_byte |= ((adpcm_value & 0x0F) << 4);
            output[output_index++] = current_byte;
        }
    }
    
    // Handle odd number of samples
    if (num_samples % 2 != 0) {
        output[output_index++] = current_byte;
    }
    
    // Save state
    state->predictor = predictor;
    state->step_size = step_size;
    
    return output_index;
}

size_t adpcm_decode(adpcm_state_t *state, const uint8_t *input,
                    size_t num_bytes, int16_t *output)
{
    if (num_bytes == 0 || input == NULL || output == NULL) {
        return 0;
    }
    
    int32_t predictor = state->predictor;
    int32_t step_size = state->step_size;
    
    size_t output_index = 0;
    
    for (size_t i = 0; i < num_bytes; i++) {
        // Process two 4-bit values per byte
        for (int nibble = 0; nibble < 2; nibble++) {
            uint8_t adpcm_value = (input[i] >> (nibble * 4)) & 0x0F;
            
            // Extract sign and value
            int8_t sign = (adpcm_value >> 3) & 0x01;
            uint8_t value = adpcm_value & 0x07;
            
            // Calculate step
            int32_t step = step_size;
            if (value & 0x04) step += step_size >> 2;
            if (value & 0x02) step += step_size >> 1;
            if (value & 0x01) step += step_size >> 3;
            
            // Update predictor
            if (sign) {
                predictor -= step;
            } else {
                predictor += step;
            }
            
            // Clamp predictor to 16-bit range
            predictor = clamp(predictor, -32768, 32767);
            
            // Update step size
            int8_t index_change = step_adjust_table[value];
            int32_t new_step_index = 0;
            
            // Find current step index
            for (int32_t j = 0; j < 89; j++) {
                if (step_size_table[j] >= step_size) {
                    new_step_index = j;
                    break;
                }
            }
            
            new_step_index += index_change;
            new_step_index = clamp(new_step_index, 0, 88);
            step_size = step_size_table[new_step_index];
            
            // Output decoded sample
            output[output_index++] = (int16_t)predictor;
        }
    }
    
    // Save state
    state->predictor = predictor;
    state->step_size = step_size;
    
    return output_index;
}
