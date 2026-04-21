/**
 * @file transition_curves.h
 * @brief Mathematical transition curves for smooth motion control
 * 
 * This header defines the interface for computing various smooth transition
 * curves used in motion profiling. These curves map a normalized time
 * parameter [0, 1] to a normalized position [0, 1] with specified
 * boundary conditions on derivatives (velocity, acceleration, jerk).
 * 
 * The curves are fundamental to preventing step loss in the aperture
 * mechanism by ensuring smooth acceleration profiles that avoid
 * mechanical resonance and inertial shock.
 * 
 * Mathematical properties:
 * - All curves satisfy: f(0)=0, f(1)=1, f'(0)=0, f'(1)=0
 * - Higher-order curves additionally satisfy f''(0)=f''(1)=0, etc.
 * 
 * @see Section 4.4: System assumptions - Smooth motion requirements
 */

#ifndef TRANSITION_CURVES_H
#define TRANSITION_CURVES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * CURVE EVALUATION FUNCTIONS
 *===========================================================================*/

/**
 * @brief Linear transition (no smoothing)
 * 
 * f(t) = t
 * f'(t) = 1
 * f''(t) = 0
 * 
 * Discontinuous velocity at boundaries - not recommended for actual
 * aperture motion but useful for testing and comparison.
 * 
 * @param t Normalized time [0.0, 1.0]
 * @return Normalized position [0.0, 1.0]
 */
float transition_linear(float t);

/**
 * @brief Sinusoidal S-curve transition
 * 
 * f(t) = 0.5 * (1 - cos(pi*t))
 * f'(t) = 0.5 * pi * sin(pi*t)
 * f''(t) = 0.5 * pi^2 * cos(pi*t)
 * 
 * Provides smooth velocity transitions with continuous acceleration.
 * Peak acceleration occurs at boundaries.
 * 
 * @param t Normalized time [0.0, 1.0]
 * @return Normalized position [0.0, 1.0]
 */
float transition_sine(float t);

/**
 * @brief Cubic polynomial transition
 * 
 * f(t) = 3t^2 - 2t^3
 * f'(t) = 6t - 6t^2
 * f''(t) = 6 - 12t
 * 
 * Continuous acceleration but discontinuous jerk at boundaries.
 * Good balance between smoothness and computational efficiency.
 * 
 * @param t Normalized time [0.0, 1.0]
 * @return Normalized position [0.0, 1.0]
 */
float transition_cubic(float t);

/**
 * @brief Quintic polynomial transition
 * 
 * f(t) = 10t^3 - 15t^4 + 6t^5
 * f'(t) = 30t^2 - 60t^3 + 30t^4
 * f''(t) = 60t - 180t^2 + 120t^3
 * f'''(t) = 60 - 360t + 360t^2
 * 
 * Continuous jerk (3rd derivative) - smoothest transition available
 * with polynomial formulation. Recommended for aperture control to
 * minimize mechanical vibration and step loss risk.
 * 
 * @param t Normalized time [0.0, 1.0]
 * @return Normalized position [0.0, 1.0]
 */
float transition_quintic(float t);

/**
 * @brief Trapezoidal velocity transition (normalized)
 * 
 * Piecewise profile with constant acceleration, cruise, and deceleration.
 * Not a pure function of t - requires parameterization by accel ratio.
 * 
 * @param t Normalized time [0.0, 1.0]
 * @param accel_ratio Fraction of time spent in acceleration (0.0-0.5)
 * @return Normalized position [0.0, 1.0]
 */
float transition_trapezoidal(float t, float accel_ratio);

/**
 * @brief Compute all derivatives for a given transition type
 * 
 * Returns position, velocity, acceleration, and jerk at normalized time t.
 * Useful for analysis and validation of profile characteristics.
 * 
 * @param t Normalized time [0.0, 1.0]
 * @param type Transition curve type selector
 * @param pos Output position
 * @param vel Output velocity (1st derivative)
 * @param accel Output acceleration (2nd derivative)
 * @param jerk Output jerk (3rd derivative)
 */
void transition_compute_all(float t, int type, 
                             float *pos, float *vel, 
                             float *accel, float *jerk);

/**
 * @brief Get the maximum jerk value for a transition type
 * 
 * Used for mechanical stress analysis and motor selection.
 * 
 * @param type Transition curve type
 * @return Maximum normalized jerk value
 */
float transition_max_jerk(int type);

/**
 * @brief Validate that a transition curve meets boundary conditions
 * 
 * Diagnostic function to verify mathematical correctness of implementations.
 * Checks f(0)=0, f(1)=1, and derivative continuity at boundaries.
 * 
 * @param type Transition curve type to validate
 * @return true if curve passes all tests
 */
bool transition_validate(int type);

#ifdef __cplusplus
}
#endif

#endif /* TRANSITION_CURVES_H */