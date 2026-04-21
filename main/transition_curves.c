/**
 * @file transition_curves.c
 * @brief Implementation of mathematical transition curves for motion control
 * 
 * This module provides the mathematical foundation for smooth motion profiles.
 * Each transition curve is a normalized function mapping [0,1] -> [0,1]
 * with controlled derivative behavior at boundaries.
 * 
 * The mathematical formulations are derived from boundary condition constraints:
 * - Position: f(0)=0, f(1)=1 (start at 0, end at 1)
 * - Velocity: f'(0)=0, f'(1)=0 (start/stop from rest)
 * - Acceleration: f''(0)=0, f''(1)=0 (smooth start/stop, for cubic+)
 * - Jerk: f'''(0)=0, f'''(1)=0 (continuous jerk, for quintic)
 * 
 * These constraints ensure that when concatenated with constant velocity
 * segments, the complete motion profile has no discontinuities in the
 * specified derivatives, preventing mechanical shock and vibration.
 * 
 * @see Section 4.2: Brightness Coding Model - Motion smoothness requirements
 */

#include "transition_curves.h"
#include "aperture_controller.h"
#include "esp_log.h"
#include <math.h>

#define TAG "TRANS_CURVES"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*============================================================================
 * INDIVIDUAL CURVE IMPLEMENTATIONS
 *===========================================================================*/

float transition_linear(float t)
{
    // Clamp to valid range
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    
    // f(t) = t
    return t;
}

float transition_sine(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    
    // f(t) = 0.5 * (1 - cos(pi * t))
    // This is the raised cosine, providing smooth S-curve shape
    return 0.5f * (1.0f - cosf(M_PI * t));
}

float transition_cubic(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    
    // f(t) = 3t^2 - 2t^3 = t^2 * (3 - 2t)
    // Derivatives:
    // f'(t) = 6t - 6t^2 = 6t(1-t)
    // f''(t) = 6 - 12t = 6(1-2t)
    float t2 = t * t;
    return t2 * (3.0f - 2.0f * t);
}

float transition_quintic(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    
    // f(t) = 10t^3 - 15t^4 + 6t^5 = t^3 * (10 - 15t + 6t^2)
    // Derivatives:
    // f'(t) = 30t^2 - 60t^3 + 30t^4 = 30t^2(1-t)^2
    // f''(t) = 60t - 180t^2 + 120t^3 = 60t(1-t)(1-2t)
    // f'''(t) = 60 - 360t + 360t^2 = 60(1-6t+6t^2)
    float t2 = t * t;
    float t3 = t2 * t;
    return t3 * (10.0f - 15.0f * t + 6.0f * t2);
}

float transition_trapezoidal(float t, float accel_ratio)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    
    // Clamp accel_ratio to valid range
    if (accel_ratio < 0.0f) accel_ratio = 0.0f;
    if (accel_ratio > 0.5f) accel_ratio = 0.5f;
    
    float t_acc = accel_ratio;      // End of acceleration phase
    float t_dec = 1.0f - accel_ratio; // Start of deceleration phase
    
    if (accel_ratio == 0.0f) {
        // Degenerate case - pure constant velocity (same as linear)
        return t;
    }
    
    // Peak velocity (area under curve must equal 1)
    // v_peak * (t_dec - t_acc) + 0.5 * v_peak * t_acc * 2 = 1
    // v_peak * (1 - 2*accel_ratio + accel_ratio) = 1
    // v_peak = 1 / (1 - accel_ratio)
    float v_peak = 1.0f / (1.0f - accel_ratio);
    
    if (t < t_acc) {
        // Acceleration phase: quadratic
        // s(t) = 0.5 * a * t^2, where a = v_peak / t_acc
        float a = v_peak / t_acc;
        return 0.5f * a * t * t;
    } else if (t < t_dec) {
        // Cruise phase: linear
        // s(t) = s(t_acc) + v_peak * (t - t_acc)
        float s_acc = 0.5f * v_peak * t_acc;
        return s_acc + v_peak * (t - t_acc);
    } else {
        // Deceleration phase: quadratic
        // s(t) = 1 - 0.5 * a * (1-t)^2
        float t_rem = 1.0f - t;
        float a = v_peak / accel_ratio;
        return 1.0f - 0.5f * a * t_rem * t_rem;
    }
}

/*============================================================================
 * COMPOSITE AND ANALYSIS FUNCTIONS
 *===========================================================================*/

void transition_compute_all(float t, int type,
                              float *pos, float *vel,
                              float *accel, float *jerk)
{
    // Initialize outputs
    *pos = 0.0f;
    *vel = 0.0f;
    *accel = 0.0f;
    *jerk = 0.0f;
    
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    
    switch (type) {
        case TRANSITION_LINEAR: {
            *pos = t;
            *vel = 1.0f;
            *accel = 0.0f;
            *jerk = 0.0f;
            break;
        }
        
        case TRANSITION_SINE: {
            float pi_t = M_PI * t;
            *pos = 0.5f * (1.0f - cosf(pi_t));
            *vel = 0.5f * M_PI * sinf(pi_t);
            *accel = 0.5f * M_PI * M_PI * cosf(pi_t);
            *jerk = -0.5f * M_PI * M_PI * M_PI * sinf(pi_t);
            break;
        }
        
        case TRANSITION_CUBIC_SPLINE: {
            float t2 = t * t;
            float t3 = t2 * t;
            *pos = 3.0f * t2 - 2.0f * t3;
            *vel = 6.0f * t - 6.0f * t2;
            *accel = 6.0f - 12.0f * t;
            *jerk = -12.0f;  // Constant (discontinuous at boundaries)
            break;
        }
        
        case TRANSITION_QUINTIC_SPLINE: {
            float t2 = t * t;
            float t3 = t2 * t;
            float t4 = t3 * t;
            *pos = 10.0f * t3 - 15.0f * t4 + 6.0f * t4 * t;
            *vel = 30.0f * t2 - 60.0f * t3 + 30.0f * t4;
            *accel = 60.0f * t - 180.0f * t2 + 120.0f * t3;
            *jerk = 60.0f - 360.0f * t + 360.0f * t2;
            break;
        }
        
        case TRANSITION_TRAPEZOIDAL: {
            // Use default 20% accel/decel ratio
            float accel_ratio = 0.2f;
            float v_peak = 1.0f / (1.0f - accel_ratio);
            float t_acc = accel_ratio;
            float t_dec = 1.0f - accel_ratio;
            
            *pos = transition_trapezoidal(t, accel_ratio);
            
            if (t < t_acc) {
                *vel = (v_peak / t_acc) * t;
                *accel = v_peak / t_acc;
                *jerk = 0.0f;  // Jerk is infinite at transitions (theoretical)
            } else if (t < t_dec) {
                *vel = v_peak;
                *accel = 0.0f;
                *jerk = 0.0f;
            } else {
                float t_rem = 1.0f - t;
                *vel = (v_peak / accel_ratio) * t_rem;
                *accel = -v_peak / accel_ratio;
                *jerk = 0.0f;
            }
            break;
        }
        
        default:
            ESP_LOGW(TAG, "Unknown transition type: %d", type);
            *pos = t;
            *vel = 1.0f;
            break;
    }
}

float transition_max_jerk(int type)
{
    switch (type) {
        case TRANSITION_LINEAR:
            return 0.0f;  // Infinite theoretical jerk at boundaries
        
        case TRANSITION_SINE:
            return 0.5f * M_PI * M_PI * M_PI;  // ~15.5 at t=0.5
        
        case TRANSITION_CUBIC_SPLINE:
            return 12.0f;  // Constant magnitude
        
        case TRANSITION_QUINTIC_SPLINE:
            // f'''(t) = 60(1 - 6t + 6t^2)
            // Maximum at t=0 or t=1: 60
            // Minimum at t=0.5: 60(1-3+1.5) = -30
            return 60.0f;
        
        case TRANSITION_TRAPEZOIDAL:
            return INFINITY;  // Theoretically infinite at phase transitions
        
        default:
            return 0.0f;
    }
}

bool transition_validate(int type)
{
    float pos, vel, accel, jerk;
    float epsilon = 0.001f;
    bool valid = true;
    
    ESP_LOGI(TAG, "Validating transition type %d", type);
    
    // Test boundary conditions
    // At t=0: pos=0, vel=0, accel=0 (for types >= cubic)
    transition_compute_all(0.0f, type, &pos, &vel, &accel, &jerk);
    
    if (fabsf(pos) > epsilon) {
        ESP_LOGE(TAG, "  FAIL: f(0)=%.4f, expected 0", pos);
        valid = false;
    }
    if (fabsf(vel) > epsilon) {
        ESP_LOGE(TAG, "  FAIL: f'(0)=%.4f, expected 0", vel);
        valid = false;
    }
    
    // At t=1: pos=1, vel=0, accel=0 (for types >= cubic)
    transition_compute_all(1.0f, type, &pos, &vel, &accel, &jerk);
    
    if (fabsf(pos - 1.0f) > epsilon) {
        ESP_LOGE(TAG, "  FAIL: f(1)=%.4f, expected 1", pos);
        valid = false;
    }
    if (fabsf(vel) > epsilon) {
        ESP_LOGE(TAG, "  FAIL: f'(1)=%.4f, expected 0", vel);
        valid = false;
    }
    
    // Check monotonicity (critical for brightness-to-time mapping)
    bool monotonic = true;
    for (float t = 0.0f; t <= 1.0f; t += 0.01f) {
        transition_compute_all(t, type, &pos, &vel, &accel, &jerk);
        if (vel < -epsilon) {
            monotonic = false;
            break;
        }
    }
    
    if (!monotonic) {
        ESP_LOGE(TAG, "  FAIL: Curve is not monotonically increasing");
        valid = false;
    }
    
    if (valid) {
        ESP_LOGI(TAG, "  Validation PASSED");
    }
    
    return valid;
}