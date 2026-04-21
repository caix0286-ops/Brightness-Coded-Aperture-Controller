/**
 * @file brightness_encoder.c
 * @brief Implementation of brightness encoding/decoding model
 * 
 * This module implements the mathematical model linking aperture size to
 * image brightness as described in Section 4.2 of the IERG5230 report.
 * 
 * The brightness coding principle:
 * 1. Aperture size A(t) varies continuously during exposure
 * 2. Light entering camera is proportional to A(t)
 * 3. For a moving point object, brightness at trace position s encodes time t
 * 4. If A(t) is monotonic, brightness provides unique temporal marker
 * 
 * The module supports:
 * - Theoretical model with configurable gamma
 * - Calibration table for measured non-linear response
 * - Real-time lookup tables for fast decoder operation
 * - Forward and inverse brightness mappings
 * 
 * @see Section 4.2: Brightness Coding Model
 * @see Section 4.3: Aperture Pattern
 * @see Section 5.4: Brightness-to-Time Decoding
 */

#include "brightness_encoder.h"
#include "aperture_controller.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define TAG "BRIGHT_ENC"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*============================================================================
 * STATIC STATE
 *===========================================================================*/

static brightness_encoder_t s_encoder = {0};
static float *s_lookup_table = NULL;  /**< Precomputed brightness->time LUT */

/*============================================================================
 * INTERNAL FUNCTIONS
 *===========================================================================*/

/**
 * @brief Theoretical brightness from aperture transmittance
 * 
 * Models the physical relationship between aperture area and image
 * brightness, including gamma correction for non-linear sensor response.
 * 
 * For a circular aperture: A ∝ r^2, so brightness ∝ (normalized_position)^2
 * with additional gamma correction for display/sensor characteristics.
 */
static float theoretical_brightness(float normalized_aperture)
{
    if (normalized_aperture <= 0.0f) {
        return s_encoder.min_transmittance;
    }
    if (normalized_aperture >= 1.0f) {
        return s_encoder.max_transmittance;
    }
    
    // Physical model: brightness proportional to aperture area
    // For iris-type aperture, area ≈ πr², and r ∝ position
    // So brightness ∝ position^2 with possible gamma correction
    float linear_brightness = normalized_aperture * normalized_aperture;
    
    // Apply gamma correction for non-linear response
    // Typical camera sensors have gamma ≈ 2.2, but we use configurable value
    if (s_encoder.gamma != 1.0f && s_encoder.gamma > 0.0f) {
        linear_brightness = powf(linear_brightness, 1.0f / s_encoder.gamma);
    }
    
    // Scale to transmittance range
    float brightness = s_encoder.min_transmittance + 
                       linear_brightness * (s_encoder.max_transmittance - s_encoder.min_transmittance);
    
    return brightness;
}

/**
 * @brief Inverse theoretical mapping: brightness -> aperture position
 * 
 * Solves the brightness equation for normalized aperture position.
 * Required for the decoder to map brightness back to time.
 */
static float brightness_to_aperture(float brightness)
{
    // Clamp to valid range
    if (brightness <= s_encoder.min_transmittance) return 0.0f;
    if (brightness >= s_encoder.max_transmittance) return 1.0f;
    
    // Normalize to [0,1]
    float normalized = (brightness - s_encoder.min_transmittance) / 
                     (s_encoder.max_transmittance - s_encoder.min_transmittance);
    
    // Invert gamma
    if (s_encoder.gamma != 1.0f && s_encoder.gamma > 0.0f) {
        normalized = powf(normalized, s_encoder.gamma);
    }
    
    // Invert square relationship (aperture area -> radius)
    float aperture = sqrtf(normalized);
    
    // Clamp result
    if (aperture < 0.0f) aperture = 0.0f;
    if (aperture > 1.0f) aperture = 1.0f;
    
    return aperture;
}

/*============================================================================
 * PUBLIC API
 *===========================================================================*/

void brightness_encoder_init(uint16_t max_steps, uint16_t levels)
{
    ESP_LOGI(TAG, "Initializing brightness encoder: steps=%d, levels=%d",
             max_steps, levels);
    
    // Clear state
    memset(&s_encoder, 0, sizeof(brightness_encoder_t));
    
    // Set defaults
    s_encoder.max_aperture_steps = max_steps;
    s_encoder.brightness_levels = levels;
    s_encoder.gamma = 2.2f;  // Standard display gamma
    s_encoder.min_transmittance = 0.05f;  // 5% minimum (not fully opaque)
    s_encoder.max_transmittance = 1.0f;   // 100% maximum (fully open)
    s_encoder.use_calibration = false;
    
    ESP_LOGI(TAG, "Default gamma=%.2f, transmittance range [%.3f, %.3f]",
             s_encoder.gamma, s_encoder.min_transmittance, s_encoder.max_transmittance);
}

void brightness_encoder_deinit(void)
{
    if (s_encoder.calibration_table != NULL) {
        free(s_encoder.calibration_table);
        s_encoder.calibration_table = NULL;
    }
    if (s_lookup_table != NULL) {
        free(s_lookup_table);
        s_lookup_table = NULL;
    }
    
    memset(&s_encoder, 0, sizeof(brightness_encoder_t));
    ESP_LOGI(TAG, "Brightness encoder deinitialized");
}

bool brightness_encoder_load_calibration(const float *table, uint16_t size)
{
    if (table == NULL || size == 0) {
        ESP_LOGE(TAG, "Invalid calibration data");
        return false;
    }
    
    // Free existing table
    if (s_encoder.calibration_table != NULL) {
        free(s_encoder.calibration_table);
    }
    
    // Allocate and copy
    s_encoder.calibration_table = (float *)malloc(size * sizeof(float));
    if (s_encoder.calibration_table == NULL) {
        ESP_LOGE(TAG, "Failed to allocate calibration table");
        return false;
    }
    
    memcpy(s_encoder.calibration_table, table, size * sizeof(float));
    s_encoder.table_size = size;
    s_encoder.use_calibration = true;
    
    ESP_LOGI(TAG, "Loaded calibration table with %d entries", size);
    return true;
}

float brightness_encoder_get_expected_brightness(float normalized_aperture)
{
    if (s_encoder.use_calibration && s_encoder.calibration_table != NULL) {
        // Use measured calibration data with interpolation
        float idx_f = normalized_aperture * (s_encoder.table_size - 1);
        uint16_t idx_low = (uint16_t)floorf(idx_f);
        uint16_t idx_high = (uint16_t)ceilf(idx_f);
        
        if (idx_high >= s_encoder.table_size) idx_high = s_encoder.table_size - 1;
        
        float frac = idx_f - idx_low;
        float brightness = s_encoder.calibration_table[idx_low] * (1.0f - frac) +
                          s_encoder.calibration_table[idx_high] * frac;
        
        return brightness;
    }
    
    // Use theoretical model
    return theoretical_brightness(normalized_aperture);
}

float brightness_encoder_get_brightness_for_position(uint16_t position)
{
    float normalized = (float)position / (float)s_encoder.max_aperture_steps;
    return brightness_encoder_get_expected_brightness(normalized);
}

float brightness_encoder_brightness_to_time(float measured_brightness,
                                             int pattern,
                                             float exposure_time_ms)
{
    // Map brightness to aperture position
    float aperture_pos = brightness_to_aperture(measured_brightness);
    
    // Map aperture position to time based on pattern
    float normalized_time = 0.0f;
    
    switch (pattern) {
        case PATTERN_CLOSING:  // A(t) = A_max - k*t, so t ∝ (1 - A)
            normalized_time = 1.0f - aperture_pos;
            break;
            
        case PATTERN_OPENING:  // A(t) = A_min + k*t, so t ∝ A
            normalized_time = aperture_pos;
            break;
            
        case PATTERN_TRIANGULAR:
            // Triangular is piecewise: first half closing, second half opening
            // For decoding, we need to know which half - assume first half
            // In practice, decoder uses frame timing to determine phase
            if (normalized_time < 0.5f) {
                normalized_time = 1.0f - aperture_pos;  // Closing phase
            } else {
                normalized_time = 0.5f + aperture_pos;  // Opening phase
            }
            break;
            
        default:
            // Default to linear mapping
            normalized_time = aperture_pos;
            break;
    }
    
    // Clamp and scale to actual time
    if (normalized_time < 0.0f) normalized_time = 0.0f;
    if (normalized_time > 1.0f) normalized_time = 1.0f;
    
    return normalized_time * exposure_time_ms;
}

const float* brightness_encoder_generate_lookup_table(int pattern,
                                                       float exposure_time_ms)
{
    // Free existing table
    if (s_lookup_table != NULL) {
        free(s_lookup_table);
    }
    
    // Allocate new table
    s_lookup_table = (float *)malloc(s_encoder.brightness_levels * sizeof(float));
    if (s_lookup_table == NULL) {
        ESP_LOGE(TAG, "Failed to allocate lookup table");
        return NULL;
    }
    
    // Precompute mapping for all possible brightness values
    for (uint16_t i = 0; i < s_encoder.brightness_levels; i++) {
        float brightness = (float)i / (float)(s_encoder.brightness_levels - 1);
        s_lookup_table[i] = brightness_encoder_brightness_to_time(brightness,
                                                                     pattern,
                                                                     exposure_time_ms);
    }
    
    ESP_LOGI(TAG, "Generated lookup table for pattern %d, exposure %.3f ms",
             pattern, exposure_time_ms);
    
    return s_lookup_table;
}

void brightness_encoder_get_status(brightness_encoder_t *encoder)
{
    if (encoder != NULL) {
        memcpy(encoder, &s_encoder, sizeof(brightness_encoder_t));
    }
}