/**
 * @file motion_profile.h
 * @brief Motion profile generator for smooth stepper acceleration/deceleration
 * 
 * This module generates smooth motion profiles for the aperture mechanism,
 * implementing the transition curves discussed in the project requirements.
 * The profile generator plans trajectories that respect velocity and 
 * acceleration limits while providing smooth motion to prevent step loss
 * when driving the aperture's inertial load.
 * 
 * Supported profiles:
 * - Linear: Constant velocity (no acceleration limiting)
 * - Sine S-curve: Smooth jerk-limited motion using sinusoidal transitions
 * - Cubic spline: Continuous acceleration profile
 * - Quintic spline: Continuous jerk profile (smoothest, recommended)
 * - Trapezoidal: Standard industrial profile with linear accel/decel
 * 
 * @see Section 4.4: Assumptions - Smooth motion requirements
 */

#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdint.h>
#include <stdbool.h>
#include "transition_curves.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * DATA STRUCTURES
 *===========================================================================*/

/**
 * @brief Motion profile state structure
 * 
 * Maintains all state variables for trajectory generation including
 * current kinematic state (position, velocity, acceleration) and
 * profile phase information.
 */
typedef struct {
    // Profile configuration
    transition_type_t type;     /**< Selected transition curve type */
    float max_velocity;         /**< Velocity limit in steps/sec */
    float max_acceleration;     /**< Acceleration limit in steps/sec^2 */
    
    // Motion plan
    float start_position;       /**< Initial position for current move */
    float target_position;      /**< Destination position */
    float total_distance;       /**< Absolute distance to travel */
    float direction;            /**< +1.0 or -1.0 */
    
    // Profile timing
    float accel_time;           /**< Duration of acceleration phase */
    float cruise_time;          /**< Duration of constant velocity phase */
    float decel_time;           /**< Duration of deceleration phase */
    float total_time;           /**< Total move duration */
    
    // Current state
    float current_time;         /**< Elapsed time in current move */
    float command_position;     /**< Computed position for this cycle */
    float current_velocity;     /**< Instantaneous velocity */
    float current_acceleration; /**< Instantaneous acceleration */
    
    // Phase tracking
    bool is_active;             /**< True if profile is executing */
    uint8_t phase;              /**< 0=accel, 1=cruise, 2=decel, 3=done */
    
    // Spline coefficients (for cubic/quintic)
    float coeffs[6];            /**< Polynomial coefficients */
} motion_profile_t;

/*============================================================================
 * FUNCTION PROTOTYPES
 *===========================================================================*/

/**
 * @brief Initialize motion profile structure
 * 
 * Sets default limits and clears state. Must be called before planning
 * any motion.
 * 
 * @param profile Pointer to profile structure
 * @param type Transition curve type for this profile
 */
void motion_profile_init(motion_profile_t *profile, transition_type_t type);

/**
 * @brief Set velocity and acceleration limits
 * 
 * @param profile Pointer to profile structure
 * @param max_velocity Maximum velocity in steps per second
 * @param max_acceleration Maximum acceleration in steps per second squared
 */
void motion_profile_set_limits(motion_profile_t *profile, 
                                float max_velocity, 
                                float max_acceleration);

/**
 * @brief Plan a new point-to-point motion
 * 
 * Computes the trajectory parameters based on the selected profile type,
 * distance to travel, and motion limits. This is a non-real-time planning
 * function that prepares the profile for execution.
 * 
 * @param profile Pointer to profile structure
 * @param current_pos Starting position in steps
 * @param target_pos Destination position in steps
 */
void motion_profile_plan_move(motion_profile_t *profile,
                               int16_t current_pos,
                               int16_t target_pos);

/**
 * @brief Compute profile update for current time step
 * 
 * Real-time function called at the control loop frequency. Updates the
 * commanded position, velocity, and acceleration based on elapsed time.
 * 
 * @param profile Pointer to profile structure
 * @param actual_position Current actual position (for feedback if needed)
 * @param dt_us Time step in microseconds
 */
void motion_profile_compute(motion_profile_t *profile,
                             int16_t actual_position,
                             uint32_t dt_us);

/**
 * @brief Check if profile has completed
 * 
 * @param profile Pointer to profile structure
 * @return true if motion is complete
 */
bool motion_profile_is_complete(const motion_profile_t *profile);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_PROFILE_H */