/**
 * @file aperture_controller.h
 * @brief Main controller for brightness-coded aperture modulation system
 * 
 * This module implements the core control logic for the continuously changing
 * aperture system described in the IERG5230 Final Project report. The aperture
 * modulation encodes temporal information into brightness distribution of 
 * motion blur, enabling high-speed motion capture from a single frame.
 * 
 * The controller manages:
 * - Aperture position control (0-500 steps range)
 * - Periodic modulation patterns (opening/closing cycles)
 * - Synchronization with camera exposure timing
 * - Transition profile selection for smooth motion
 * 
 * Hardware: ESP32 + 4-wire 2-phase stepper motor + Variable aperture mechanism
 * 
 * @author IERG5230 Final Project Team
 * @date 2026
 */

#ifndef APERTURE_CONTROLLER_H
#define APERTURE_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * CONSTANTS AND CONFIGURATION
 *===========================================================================*/

/**
 * @brief Maximum aperture travel in steps
 * 
 * The aperture mechanism has a mechanical limit of 500 steps from fully closed
 * to fully open. This range provides sufficient resolution for brightness coding
 * while maintaining fast transition times compatible with 60fps camera operation.
 */
#define APERTURE_MAX_STEPS          500

/**
 * @brief Default modulation frequency in Hz
 * 
 * As described in Section 4.3 of the report, for a camera operating at 60fps
 * with exposure time 1/60s, the aperture changes at 30Hz. This means one 
 * frame corresponds to a closing aperture and the next frame corresponds to 
 * an opening aperture, creating periodic brightness coding.
 */
#define DEFAULT_MODULATION_FREQ_HZ  30

/**
 * @brief Minimum step period in microseconds (maximum speed limit)
 * 
 * This defines the fastest stepping rate to prevent motor stall and ensure
 * reliable operation when driving the aperture mass. Determined empirically
 * based on motor torque curve and aperture mechanism inertia.
 */
#define MIN_STEP_PERIOD_US          800

/**
 * @brief Maximum step period in microseconds (slowest speed)
 * 
 * Used during startup and when precise positioning is required. Slower speeds
 * provide higher torque and smoother motion for the aperture mechanism.
 */
#define MAX_STEP_PERIOD_US          5000

/**
 * @brief Number of aperture positions for discrete brightness levels
 * 
 * The brightness-to-time mapping (Section 4.2) requires sufficient quantization
 * levels along the aperture range. 256 levels provide 8-bit brightness 
 * resolution, matching typical image sensor ADC capabilities.
 */
#define BRIGHTNESS_LEVELS           256

/*============================================================================
 * DATA TYPES AND ENUMERATIONS
 *===========================================================================*/

/**
 * @brief Transition curve types for smooth aperture motion
 * 
 * As discussed in Section 4.4 and implementation considerations, smooth 
 * transitions are critical to prevent step loss when accelerating the 
 * aperture mechanism's mass. Different curves offer trade-offs between
 * computational complexity and motion smoothness.
 */
typedef enum {
    TRANSITION_LINEAR = 0,      /**< Constant velocity (no smoothing) */
    TRANSITION_SINE,            /**< Sinusoidal S-curve (smooth jerk) */
    TRANSITION_CUBIC_SPLINE,    /**< Cubic spline interpolation (continuous acceleration) */
    TRANSITION_QUINTIC_SPLINE,  /**< Quintic spline (continuous jerk, smoothest) */
    TRANSITION_TRAPEZOIDAL,     /**< Trapezoidal velocity profile (common industrial) */
    TRANSITION_TRANSITION_COUNT /**< Number of available transition types */
} transition_type_t;

/**
 * @brief Aperture modulation pattern types
 * 
 * Corresponds to the aperture patterns described in Section 4.3:
 * - Closing aperture: brightness decreases along time (A(t) = A_max - k*t)
 * - Opening aperture: brightness increases along time (A(t) = A_min + k*t)
 */
typedef enum {
    PATTERN_CLOSING = 0,        /**< Aperture closes during exposure (bright-to-dark) */
    PATTERN_OPENING,            /**< Aperture opens during exposure (dark-to-bright) */
    PATTERN_TRIANGULAR,         /**< Continuous triangular wave (alternating) */
    PATTERN_SAWTOOTH,           /**< Sawtooth wave (resetting) */
    PATTERN_CUSTOM              /**< User-defined arbitrary pattern */
} modulation_pattern_t;

/**
 * @brief Aperture controller state machine states
 */
typedef enum {
    STATE_IDLE = 0,             /**< Controller idle, motor disabled */
    STATE_HOMING,               /**< Seeking home position (fully closed) */
    STATE_READY,                /**< Homed and ready for modulation */
    STATE_MODULATING,           /**< Active brightness coding modulation */
    STATE_STOPPING,             /**< Decelerating to stop */
    STATE_ERROR                 /**< Error condition (requires reset) */
} controller_state_t;

/**
 * @brief Aperture controller configuration structure
 */
typedef struct {
    uint32_t modulation_freq_hz;        /**< Modulation frequency (typically 30Hz) */
    uint16_t max_steps;                 /**< Maximum step range (default 500) */
    uint16_t current_position;          /**< Current aperture position in steps */
    transition_type_t transition_type;  /**< Selected motion profile curve */
    modulation_pattern_t pattern;       /**< Brightness coding pattern */
    bool invert_direction;              /**< Reverse motor direction if needed */
    uint32_t home_speed_us;             /**< Step period for homing sequence */
    float exposure_time_ms;             /**< Camera exposure time for sync */
} aperture_config_t;

/**
 * @brief Real-time aperture status for monitoring and synchronization
 */
typedef struct {
    controller_state_t state;           /**< Current controller state */
    uint16_t position;                  /**< Current position (0-500) */
    uint16_t target_position;           /**< Commanded target position */
    uint32_t step_period_us;            /**< Current step period */
    float normalized_aperture;          /**< Normalized value [0.0, 1.0] */
    float normalized_brightness;          /**< Expected brightness [0.0, 1.0] */
    int32_t modulation_cycle_count;     /**< Completed modulation cycles */
    bool is_moving;                       /**< True if motor is active */
    float instantaneous_velocity;       /**< Steps per second */
    float instantaneous_acceleration;   /**< Steps per second squared */
} aperture_status_t;

/*============================================================================
 * FUNCTION PROTOTYPES
 *===========================================================================*/

/**
 * @brief Initialize the aperture controller system
 * 
 * Sets up GPIO, timer interrupts, stepper driver, and default configuration.
 * Must be called before any other controller functions.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aperture_controller_init(void);

/**
 * @brief Configure the aperture controller parameters
 * 
 * Applies user-specified configuration including modulation frequency,
 * transition curve type, and pattern selection. Validates parameters
 * against hardware limits.
 * 
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if parameters invalid
 */
esp_err_t aperture_controller_configure(const aperture_config_t *config);

/**
 * @brief Start the brightness-coded aperture modulation
 * 
 * Begins periodic aperture modulation according to the configured pattern.
 * The aperture will oscillate between fully open and fully closed positions,
 * generating the brightness coding described in Section 4.2 of the report.
 * 
 * @return ESP_OK on success, error if not in READY state
 */
esp_err_t aperture_controller_start_modulation(void);

/**
 * @brief Stop aperture modulation with smooth deceleration
 * 
 * Initiates a controlled deceleration sequence using the selected transition
 * curve to bring the aperture to rest without step loss.
 * 
 * @return ESP_OK on success
 */
esp_err_t aperture_controller_stop(void);

/**
 * @brief Emergency stop (immediate halt)
 * 
 * Immediately disables motor drivers. May cause step loss - use only in
 * emergency situations. Requires re-homing before resuming operation.
 * 
 * @return ESP_OK on success
 */
esp_err_t aperture_controller_emergency_stop(void);

/**
 * @brief Perform homing sequence to establish zero position
 * 
 * Drives the aperture to the fully closed position (step 0) using a
 * limit switch or stall detection. Required after power-up or emergency stop.
 * 
 * @return ESP_OK on success, error if homing fails
 */
esp_err_t aperture_controller_home(void);

/**
 * @brief Move aperture to specific position with smooth transition
 * 
 * Commands the aperture to move to an absolute position using the currently
 * configured transition curve for acceleration/deceleration.
 * 
 * @param position Target position (0 to APERTURE_MAX_STEPS)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if position out of range
 */
esp_err_t aperture_controller_move_to(uint16_t position);

/**
 * @brief Set the transition curve type for smooth motion
 * 
 * Allows runtime selection of motion profile as described in the project
 * requirements. Each curve provides different smoothness characteristics:
 * - LINEAR: Fastest computation, no smoothing (for testing only)
 * - SINE: Good smoothness, moderate computation
 * - CUBIC_SPLINE: Excellent smoothness, continuous acceleration
 * - QUINTIC_SPLINE: Best smoothness, continuous jerk (recommended)
 * - TRAPEZOIDAL: Industrial standard, good performance/computation balance
 * 
 * @param type Transition curve type enum value
 * @return ESP_OK on success
 */
esp_err_t aperture_controller_set_transition_type(transition_type_t type);

/**
 * @brief Get current aperture status
 * 
 * Provides real-time status information for monitoring, debugging, and
 * synchronization with camera trigger signals.
 * 
 * @param status Pointer to status structure to fill
 * @return ESP_OK on success
 */
esp_err_t aperture_controller_get_status(aperture_status_t *status);

/**
 * @brief Get normalized aperture value for brightness coding
 * 
 * Returns the current aperture position normalized to [0.0, 1.0] range,
 * corresponding to the alpha(t) function in Section 4.2 of the report.
 * This value directly maps to expected image brightness for decoding.
 * 
 * @return Normalized aperture value [0.0, 1.0]
 */
float aperture_controller_get_normalized_aperture(void);

/**
 * @brief Register callback for modulation cycle completion
 * 
 * Allows camera synchronization - callback fires at start of each modulation
 * cycle (e.g., beginning of closing or opening phase).
 * 
 * @param callback Function pointer to call, or NULL to unregister
 * @param user_data User context pointer passed to callback
 * @return ESP_OK on success
 */
esp_err_t aperture_controller_register_cycle_callback(
    void (*callback)(modulation_pattern_t pattern, void *user_data),
    void *user_data
);

/**
 * @brief Deinitialize the aperture controller
 * 
 * Releases hardware resources, disables interrupts, and frees memory.
 * Controller must be reinitialized before reuse.
 * 
 * @return ESP_OK on success
 */
esp_err_t aperture_controller_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* APERTURE_CONTROLLER_H */