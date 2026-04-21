/**
 * @file stepper_driver.h
 * @brief Low-level 4-wire 2-phase stepper motor driver interface
 * 
 * Hardware abstraction layer for controlling a bipolar stepper motor using
 * two H-bridges or dedicated driver chips (e.g., DRV8825, A4988). The driver
 * implements full-step and half-step modes with configurable speed through
 * step period timing.
 * 
 * The stepper motor drives the variable aperture mechanism in the brightness-
 * coded motion capture system. Precise position control is essential for
 * accurate brightness-to-time encoding.
 * 
 * @see Section 4.1: System Design - Hardware implementation
 */

#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * HARDWARE CONFIGURATION
 *===========================================================================*/

/**
 * @brief GPIO pin definitions for 4-wire 2-phase stepper
 * 
 * Phase A: Coil 1 (A+, A-)
 * Phase B: Coil 2 (B+, B-)
 * 
 * Using standard step/dir interface with separate enable control.
 * Alternative: direct phase control for microstepping.
 */
#define STEPPER_STEP_GPIO       GPIO_NUM_18     /**< Step pulse output */
#define STEPPER_DIR_GPIO        GPIO_NUM_19     /**< Direction output */
#define STEPPER_ENABLE_GPIO     GPIO_NUM_21     /**< Driver enable (active low) */
#define STEPPER_HOME_GPIO       GPIO_NUM_22     /**< Home limit switch input */

/**
 * @brief Step timing constants
 * 
 * Minimum step period determines maximum speed. Values based on motor
 * specifications (step angle, torque curve) and load inertia (aperture mass).
 */
#define STEPPER_MIN_PERIOD_US   800     /**< Minimum safe step period */
#define STEPPER_MAX_PERIOD_US   10000   /**< Slowest step period for homing */

/*============================================================================
 * FUNCTION PROTOTYPES
 *===========================================================================*/

/**
 * @brief Initialize stepper driver hardware
 * 
 * Configures GPIO pins, sets default state (disabled), and initializes
 * position tracking. Must be called before any motion commands.
 * 
 * @return ESP_OK on success
 */
esp_err_t stepper_driver_init(void);

/**
 * @brief Move to absolute position with speed control
 * 
 * The driver internally manages step generation to reach the target position
 * at the specified step period. Non-blocking - returns immediately, motion
 * continues in background.
 * 
 * @param position Target absolute position in steps
 * @return ESP_OK on success
 */
esp_err_t stepper_driver_move_to(int16_t position);

/**
 * @brief Get current absolute position
 * 
 * Returns the tracked position based on step commands issued. Note: this
 * is open-loop tracking; actual position may differ if step loss occurred.
 * 
 * @return Current position in steps
 */
int16_t stepper_driver_get_position(void);

/**
 * @brief Get current step period
 * 
 * @return Current step period in microseconds
 */
uint32_t stepper_driver_get_current_period(void);

/**
 * @brief Perform homing sequence
 * 
 * Drives motor toward home position until limit switch activates or stall
 * is detected. Resets position counter to zero at home.
 * 
 * @param step_period_us Step period for homing motion
 * @param timeout_ms Maximum time allowed for homing
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if homing fails
 */
esp_err_t stepper_driver_home(uint32_t step_period_us, uint32_t timeout_ms);

/**
 * @brief Emergency disable motor driver
 * 
 * Immediately disables driver outputs. Motor will coast or hold depending
 * on driver configuration. Use only for emergency stops.
 */
void stepper_driver_emergency_disable(void);

/**
 * @brief Set motor enable state
 * 
 * @param enable true to enable driver, false to disable
 */
void stepper_driver_set_enable(bool enable);

/**
 * @brief Deinitialize stepper driver
 * 
 * Releases GPIO resources and disables motor.
 * 
 * @return ESP_OK on success
 */
esp_err_t stepper_driver_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_DRIVER_H */