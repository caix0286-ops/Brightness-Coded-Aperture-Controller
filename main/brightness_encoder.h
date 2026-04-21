/**
 * @file brightness_encoder.h
 * @brief Brightness encoding model for aperture-to-brightness mapping
 * 
 * This module implements the theoretical brightness coding model described
 * in Section 4.2 of the report. It provides the mathematical relationship
 * between aperture size and expected image brightness, enabling the
 * decoder to map brightness back to time.
 * 
 * The key equation from the report:
 *   alpha(t) = (A(t) - A_min) / (A_max - A_min)
 *   b(s) ≈ alpha(t)
 *   t ≈ alpha^(-1)(b(s))
 * 
 * Where:
 *   A(t) is the aperture area at time t
 *   alpha(t) is the normalized aperture function
 *   b(s) is the normalized brightness along the motion trace
 * 
 * This module provides both the forward mapping (aperture -> brightness)
 * and the inverse mapping (brightness -> time) for decoder synchronization.
 * 
 * @see Section 4.2: Brightness Coding Model
 * @see Section 5.4: Brightness-to-Time Decoding
 */

#ifndef BRIGHTNESS_ENCODER_H
#define BRIGHTNESS_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * CONFIGURATION
 *===========================================================================*/

/**
 * @brief Default number of discrete brightness quantization levels
 * 
 * 256 levels (8-bit) matches typical camera sensor ADC resolution and
 * provides sufficient granularity for the brightness-to-time mapping.
 */
#define DEFAULT_BRIGHTNESS_LEVELS   256

/**
 * @brief Minimum valid brightness value
 * 
 * Non-zero minimum prevents division issues in decoder and accounts
 * for sensor dark current.
 */
#define MIN_BRIGHTNESS_VALUE        1

/*============================================================================
 * DATA STRUCTURES
 *===========================================================================*/

/**
 * @brief Brightness encoder calibration data
 * 
 * Stores the measured or theoretical relationship between aperture
 * position and resulting image brightness. Used for non-linear
 * aperture response correction.
 */
typedef struct {
    uint16_t max_aperture_steps;    /**< Maximum aperture travel in steps */
    uint16_t brightness_levels;     /**< Number of quantization levels */
    
    // Calibration table (optional, for non-linear response)
    bool use_calibration;           /**< True if using measured calibration */
    float *calibration_table;       /**< Lookup table: step -> brightness */
    uint16_t table_size;            /**< Size of calibration table */
    
    // Theoretical model parameters
    float gamma;                    /**< Gamma correction factor (typically 2.2) */
    float min_transmittance;        /**< Minimum aperture transmittance [0,1] */
    float max_transmittance;        /**< Maximum aperture transmittance [0,1] */
} brightness_encoder_t;

/*============================================================================
 * FUNCTION PROTOTYPES
 *===========================================================================*/

/**
 * @brief Initialize brightness encoder
 * 
 * Sets up the theoretical model with default parameters. Call before
 * using any encoding/decoding functions.
 * 
 * @param max_steps Maximum aperture position in steps
 * @param levels Number of brightness quantization levels
 */
void brightness_encoder_init(uint16_t max_steps, uint16_t levels);

/**
 * @brief Deinitialize brightness encoder
 * 
 * Frees allocated calibration table memory.
 */
void brightness_encoder_deinit(void);

/**
 * @brief Load calibration table from flash/storage
 * 
 * For precise systems, the aperture-to-brightness relationship should be
 * measured and stored. This loads a previously saved calibration.
 * 
 * @param table Pointer to calibration data array
 * @param size Number of entries in table
 * @return true if loaded successfully
 */
bool brightness_encoder_load_calibration(const float *table, uint16_t size);

/**
 * @brief Get expected brightness for a given normalized aperture
 * 
 * Forward mapping: aperture position -> expected brightness.
 * Implements the theoretical model from Section 4.2.
 * 
 * @param normalized_aperture Normalized aperture [0.0, 1.0]
 * @return Expected normalized brightness [0.0, 1.0]
 */
float brightness_encoder_get_expected_brightness(float normalized_aperture);

/**
 * @brief Get expected brightness for absolute aperture position
 * 
 * Convenience function that normalizes position before computing brightness.
 * 
 * @param position Absolute aperture position in steps
 * @return Expected normalized brightness [0.0, 1.0]
 */
float brightness_encoder_get_brightness_for_position(uint16_t position);

/**
 * @brief Estimate time from measured brightness (decoder function)
 * 
 * Inverse mapping for the decoding pipeline (Section 5.4). Given a
 * measured brightness value and known aperture pattern, estimates the
 * time within the exposure when that brightness occurred.
 * 
 * @param measured_brightness Normalized measured brightness [0.0, 1.0]
 * @param pattern Current modulation pattern (opening or closing)
 * @param exposure_time_ms Total exposure time in milliseconds
 * @return Estimated time within exposure [0.0, exposure_time_ms]
 */
float brightness_encoder_brightness_to_time(float measured_brightness,
                                             int pattern,
                                             float exposure_time_ms);

/**
 * @brief Generate lookup table for fast decoder operation
 * 
 * Precomputes the brightness-to-time mapping for the current pattern,
 * enabling O(1) lookup during real-time decoding.
 * 
 * @param pattern Modulation pattern type
 * @param exposure_time_ms Exposure duration
 * @return Pointer to lookup table (managed internally)
 */
const float* brightness_encoder_generate_lookup_table(int pattern,
                                                       float exposure_time_ms);

/**
 * @brief Get current encoder status and parameters
 * 
 * @param encoder Pointer to structure to fill with current state
 */
void brightness_encoder_get_status(brightness_encoder_t *encoder);

#ifdef __cplusplus
}
#endif

#endif /* BRIGHTNESS_ENCODER_H */