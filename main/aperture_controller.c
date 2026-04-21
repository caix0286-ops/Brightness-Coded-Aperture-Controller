/**
 * @file aperture_controller.c
 * @brief Implementation of brightness-coded aperture controller
 * 
 * This module implements the core control logic for continuously changing
 * aperture modulation as described in the IERG5230 Final Project report.
 * 
 * The controller generates periodic aperture variations that encode temporal
 * information into the brightness distribution of motion blur. This enables
 * high-speed motion capture from a single camera frame by creating a unique
 * brightness-to-time mapping along the motion trace.
 * 
 * Key features:
 * - 500-step aperture range with position feedback
 * - 30Hz modulation frequency synchronized with 60fps camera
 * - Multiple smooth transition curves to prevent step loss
 * - Real-time status monitoring for decoder synchronization
 * 
 * @see Section 4: System Design
 * @see Section 4.2: Brightness Coding Model
 * @see Section 4.3: Aperture Pattern
 */

#include "aperture_controller.h"
#include "stepper_driver.h"
#include "motion_profile.h"
#include "transition_curves.h"
#include "brightness_encoder.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"

#include <string.h>
#include <math.h>

/*============================================================================
 * MODULE CONFIGURATION AND MACROS
 *===========================================================================*/

#define TAG "APERTURE_CTRL"

/**
 * @brief Control loop frequency in Hz
 * 
 * The position control loop runs at 1kHz, providing 33.3 updates per
 * modulation cycle at 30Hz. This is sufficient for smooth motion control
 * while leaving CPU time for other tasks.
 */
#define CONTROL_LOOP_FREQ_HZ        1000

/**
 * @brief Control loop period in microseconds
 */
#define CONTROL_LOOP_PERIOD_US      (1000000 / CONTROL_LOOP_FREQ_HZ)

/**
 * @brief Maximum allowed position error before declaring fault
 * 
 * If the actual position deviates from commanded position by more than
 * this threshold, a tracking error is declared (possible step loss).
 */
#define MAX_POSITION_ERROR_STEPS    5

/**
 * @brief Homing timeout in milliseconds
 * 
 * If homing sequence exceeds this duration, the operation is aborted
 * to prevent damage to the mechanism.
 */
#define HOMING_TIMEOUT_MS           10000

/**
 * @brief Default acceleration limit in steps/sec^2
 * 
 * Conservative default to ensure reliable operation with aperture mass.
 * Can be adjusted based on empirical testing with specific hardware.
 */
#define DEFAULT_ACCEL_LIMIT         50000.0f

/**
 * @brief Default velocity limit in steps/sec
 * 
 * Maximum speed during modulation. Limited by motor torque at high speeds
 * and the need to complete 500-step travel within half modulation period
 * (16.67ms at 30Hz).
 */
#define DEFAULT_VELOCITY_LIMIT      30000.0f

/*============================================================================
 * INTERNAL DATA STRUCTURES
 *===========================================================================*/

/**
 * @brief Modulation cycle timing information
 * 
 * Tracks the phase and timing of the current modulation cycle for
 * synchronization with camera exposure and brightness decoding.
 */
typedef struct {
    uint64_t cycle_start_time_us;       /**< Timestamp when current cycle began */
    float phase;                        /**< Current phase [0.0, 1.0] within cycle */
    modulation_pattern_t current_pattern; /**< Active pattern for this half-cycle */
    bool is_first_half;                 /**< True if in first half of triangular cycle */
} modulation_timing_t;

/**
 * @brief Complete controller state structure
 * 
 * Encapsulates all runtime state for the aperture controller including
 * configuration, motion profiles, timing, and synchronization data.
 */
typedef struct {
    // Configuration
    aperture_config_t config;           /**< Active configuration */
    
    // Motion control
    motion_profile_t profile;           /**< Active motion profile generator */
    transition_type_t transition_type;  /**< Selected transition curve */
    float accel_limit;                  /**< Acceleration limit steps/sec^2 */
    float velocity_limit;               /**< Velocity limit steps/sec */
    
    // Position state
    volatile int16_t current_position;  /**< Actual position (ISR updated) */
    int16_t commanded_position;         /**< Target position from trajectory */
    int16_t target_position;            /**< Final destination */
    
    // Modulation state
    modulation_timing_t timing;         /**< Current cycle timing */
    volatile bool modulation_active;      /**< True if modulation running */
    int32_t cycle_count;                /**< Completed full cycles */
    
    // Synchronization
    void (*cycle_callback)(modulation_pattern_t, void*);
    void *cycle_callback_data;
    
    // Control loop
    esp_timer_handle_t control_timer;   /**< High-resolution timer handle */
    SemaphoreHandle_t state_mutex;      /**< Protects state variables */
    
    // Status
    controller_state_t state;           /**< Current controller state */
    bool homed;                         /**< True if home position known */
    uint32_t error_flags;               /**< Bitmask of error conditions */
} controller_context_t;

/*============================================================================
 * STATIC VARIABLES
 *===========================================================================*/

/**
 * @brief Singleton controller instance
 * 
 * The aperture controller is designed as a singleton since there is only
 * one physical aperture mechanism per camera system.
 */
static controller_context_t s_controller = {0};

/**
 * @brief Flag indicating controller has been initialized
 */
static bool s_initialized = false;

/*============================================================================
 * FORWARD DECLARATIONS
 *===========================================================================*/

static void control_loop_callback(void *arg);
static esp_err_t update_motion_profile(void);
static esp_err_t execute_modulation_step(void);
static float compute_pattern_position(float phase, modulation_pattern_t pattern);
static esp_err_t check_position_fault(void);
static void update_status_variables(void);

/*============================================================================
 * PUBLIC API IMPLEMENTATION
 *===========================================================================*/

esp_err_t aperture_controller_init(void)
{
    ESP_LOGI(TAG, "Initializing brightness-coded aperture controller");
    
    if (s_initialized) {
        ESP_LOGW(TAG, "Controller already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clear context structure
    memset(&s_controller, 0, sizeof(controller_context_t));
    
    // Initialize default configuration
    s_controller.config.modulation_freq_hz = DEFAULT_MODULATION_FREQ_HZ;
    s_controller.config.max_steps = APERTURE_MAX_STEPS;
    s_controller.config.current_position = 0;
    s_controller.config.transition_type = TRANSITION_QUINTIC_SPLINE;  // Best smoothness
    s_controller.config.pattern = PATTERN_TRIANGULAR;  // Alternating open/close
    s_controller.config.invert_direction = false;
    s_controller.config.home_speed_us = MAX_STEP_PERIOD_US;
    s_controller.config.exposure_time_ms = 16.667f;  // 1/60s in ms
    
    // Initialize motion limits
    s_controller.accel_limit = DEFAULT_ACCEL_LIMIT;
    s_controller.velocity_limit = DEFAULT_VELOCITY_LIMIT;
    
    // Initialize state
    s_controller.state = STATE_IDLE;
    s_controller.homed = false;
    s_controller.modulation_active = false;
    
    // Create mutex for thread-safe state access
    s_controller.state_mutex = xSemaphoreCreateMutex();
    if (s_controller.state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create state mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize stepper motor driver hardware
    esp_err_t err = stepper_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Stepper driver init failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_controller.state_mutex);
        return err;
    }
    
    // Initialize brightness encoder (theoretical model)
    brightness_encoder_init(APERTURE_MAX_STEPS, BRIGHTNESS_LEVELS);
    
    // Create high-resolution periodic timer for control loop
    const esp_timer_create_args_t timer_args = {
        .callback = &control_loop_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,  // Callback from timer task
        .name = "aperture_control",
        .skip_unhandled_events = false
    };
    
    err = esp_timer_create(&timer_args, &s_controller.control_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create control timer: %s", esp_err_to_name(err));
        stepper_driver_deinit();
        vSemaphoreDelete(s_controller.state_mutex);
        return err;
    }
    
    s_initialized = true;
    ESP_LOGI(TAG, "Aperture controller initialized successfully");
    ESP_LOGI(TAG, "Max steps: %d, Default freq: %d Hz", 
             APERTURE_MAX_STEPS, DEFAULT_MODULATION_FREQ_HZ);
    
    return ESP_OK;
}

esp_err_t aperture_controller_configure(const aperture_config_t *config)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate parameters
    if (config->modulation_freq_hz < 1 || config->modulation_freq_hz > 100) {
        ESP_LOGE(TAG, "Invalid modulation frequency: %d Hz (must be 1-100)", 
                 config->modulation_freq_hz);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->max_steps == 0 || config->max_steps > APERTURE_MAX_STEPS) {
        ESP_LOGE(TAG, "Invalid max steps: %d (must be 1-%d)", 
                 config->max_steps, APERTURE_MAX_STEPS);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->transition_type >= TRANSITION_TRANSITION_COUNT) {
        ESP_LOGE(TAG, "Invalid transition type: %d", config->transition_type);
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(s_controller.state_mutex, portMAX_DELAY);
    
    // Copy configuration
    memcpy(&s_controller.config, config, sizeof(aperture_config_t));
    
    // Update motion profile parameters based on new config
    s_controller.transition_type = config->transition_type;
    
    // Recalculate velocity limit based on modulation frequency
    // Must complete full travel in half period (quarter period each direction)
    float half_period_ms = 500.0f / config->modulation_freq_hz;
    float required_velocity = (float)config->max_steps / (half_period_ms / 1000.0f);
    
    // Apply safety margin (80% of theoretical max)
    s_controller.velocity_limit = required_velocity * 0.8f;
    
    // Ensure velocity doesn't exceed hardware capability
    float max_hardware_velocity = 1000000.0f / MIN_STEP_PERIOD_US;
    if (s_controller.velocity_limit > max_hardware_velocity) {
        ESP_LOGW(TAG, "Velocity limit capped to hardware maximum: %.1f steps/s",
                 max_hardware_velocity);
        s_controller.velocity_limit = max_hardware_velocity;
    }
    
    ESP_LOGI(TAG, "Configuration updated:");
    ESP_LOGI(TAG, "  Modulation: %d Hz, Pattern: %d", 
             config->modulation_freq_hz, config->pattern);
    ESP_LOGI(TAG, "  Transition: %d, Max steps: %d",
             config->transition_type, config->max_steps);
    ESP_LOGI(TAG, "  Velocity limit: %.1f steps/s", s_controller.velocity_limit);
    
    xSemaphoreGive(s_controller.state_mutex);
    
    return ESP_OK;
}

esp_err_t aperture_controller_start_modulation(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(s_controller.state_mutex, portMAX_DELAY);
    
    if (s_controller.state != STATE_READY && s_controller.state != STATE_IDLE) {
        ESP_LOGE(TAG, "Cannot start modulation from state: %d", s_controller.state);
        xSemaphoreGive(s_controller.state_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!s_controller.homed) {
        ESP_LOGW(TAG, "Aperture not homed, performing auto-home");
        xSemaphoreGive(s_controller.state_mutex);
        esp_err_t err = aperture_controller_home();
        if (err != ESP_OK) {
            return err;
        }
        xSemaphoreTake(s_controller.state_mutex, portMAX_DELAY);
    }
    
    // Reset modulation timing
    s_controller.timing.cycle_start_time_us = esp_timer_get_time();
    s_controller.timing.phase = 0.0f;
    s_controller.timing.current_pattern = s_controller.config.pattern;
    s_controller.timing.is_first_half = true;
    
    // Initialize motion profile for first cycle
    motion_profile_init(&s_controller.profile, s_controller.transition_type);
    motion_profile_set_limits(&s_controller.profile, 
                              s_controller.velocity_limit, 
                              s_controller.accel_limit);
    
    // Set initial target based on pattern
    float initial_pos = compute_pattern_position(0.0f, s_controller.config.pattern);
    s_controller.target_position = (int16_t)(initial_pos * s_controller.config.max_steps);
    
    // Start control loop timer
    esp_err_t err = esp_timer_start_periodic(s_controller.control_timer, 
                                             CONTROL_LOOP_PERIOD_US);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start control timer: %s", esp_err_to_name(err));
        xSemaphoreGive(s_controller.state_mutex);
        return err;
    }
    
    s_controller.modulation_active = true;
    s_controller.state = STATE_MODULATING;
    
    ESP_LOGI(TAG, "Modulation started at %d Hz with pattern %d",
             s_controller.config.modulation_freq_hz, s_controller.config.pattern);
    
    xSemaphoreGive(s_controller.state_mutex);
    
    return ESP_OK;
}

esp_err_t aperture_controller_stop(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(s_controller.state_mutex, portMAX_DELAY);
    
    if (!s_controller.modulation_active) {
        xSemaphoreGive(s_controller.state_mutex);
        return ESP_OK;  // Already stopped
    }
    
    // Transition to stopping state - profile will decelerate smoothly
    s_controller.state = STATE_STOPPING;
    s_controller.target_position = s_controller.current_position;  // Stop at current pos
    
    ESP_LOGI(TAG, "Initiating smooth stop sequence");
    
    xSemaphoreGive(s_controller.state_mutex);
    
    // Wait for motion to complete (with timeout)
    int timeout_ms = 2000;
    while (timeout_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout_ms -= 10;
        
        xSemaphoreTake(s_controller.state_mutex, portMAX_DELAY);
        bool moving = s_controller.profile.is_active;
        xSemaphoreGive(s_controller.state_mutex);
        
        if (!moving) break;
    }
    
    // Stop control timer
    esp_timer_stop(s_controller.control_timer);
    
    xSemaphoreTake(s_controller.state_mutex, portMAX_DELAY);
    s_controller.modulation_active = false;
    s_controller.state = STATE_READY;
    xSemaphoreGive(s_controller.state_mutex);
    
    ESP_LOGI(TAG, "Modulation stopped, aperture at position %d", 
             s_controller.current_position);
    
    return ESP_OK;
}

esp_err_t aperture_controller_emergency_stop(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Immediate hardware disable - no software deceleration
    esp_timer_stop(s_controller.control_timer);
    stepper_driver_emergency_disable();
    
    xSemaphoreTake(s_controller.state_mutex, portMAX_DELAY);
    s_controller.modulation_active = false;
    s_controller.state = STATE_ERROR;
    s_controller.homed = false;  // Position unknown after emergency stop
    s_controller.error_flags |= 0x01;  // Set emergency stop flag
    xSemaphoreGive(s_controller.state_mutex);
    
    ESP_LOGW(TAG, "EMERGENCY STOP executed - position lost, re-homing required");
    
    return ESP_OK;
}

esp_err_t aperture_controller_home(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting homing sequence");
    
    xSemaphoreTake(s_controller.state_mutex, portMAX_DELAY);
    s_controller.state = STATE_HOMING;
    xSemaphoreGive(s_controller.state_mutex);
    
    // Disable any active modulation
    esp_timer_stop(s_controller.control_timer);
    
    // Perform homing using stepper driver
    esp_err_t err = stepper_driver_home(s_controller.config.home_speed_us,
                                        HOMING_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Homing failed: %s", esp_err_to_name(err));
        xSemaphoreTake(s_controller.state_mutex, portMAX_DELAY);
        s_controller.state = STATE_ERROR;
        xSemaphoreGive(s_controller.state_mutex);
        return err;
    }
    
    // Update position state
    xSemaphoreTake(s_controller.state_mutex, portMAX_DELAY);
    s_controller.current_position = 0;
    s_controller.commanded_position = 0;
    s_controller.target_position = 0;
    s_controller.homed = true;
    s_controller.state = STATE_READY;
    xSemaphoreGive(s_controller.state_mutex);
    
    ESP_LOGI(TAG, "Homing completed - aperture at position 0 (fully closed)");
    
    return ESP_OK;
}

esp_err_t aperture_controller_move_to(uint16_t position)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (position > s_controller.config.max_steps) {
        ESP_LOGE(TAG, "Position %d exceeds maximum %d", 
                 position, s_controller.config.max_steps);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_controller.homed) {
        ESP_LOGW(TAG, "Move command requires homing first");
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(s_controller.state_mutex, portMAX_DELAY);
    
    // Update target
    s_controller.target_position = (int16_t)position;
    
    // Initialize motion profile for this move
    motion_profile_init(&s_controller.profile, s_controller.transition_type);
    motion_profile_set_limits(&s_controller.profile,
                              s_controller.velocity_limit,
                              s_controller.accel_limit);
    motion_profile_plan_move(&s_controller.profile,
                             s_controller.current_position,
                             s_controller.target_position);
    
    // Start control loop if not running
    if (!s_controller.modulation_active) {
        esp_err_t err = esp_timer_start_periodic(s_controller.control_timer,
                                                 CONTROL_LOOP_PERIOD_US);
        if (err != ESP_OK) {
            xSemaphoreGive(s_controller.state_mutex);
            return err;
        }
    }
    
    ESP_LOGI(TAG, "Moving to position %d (from %d)", 
             position, s_controller.current_position);
    
    xSemaphoreGive(s_controller.state_mutex);
    
    return ESP_OK;
}

esp_err_t aperture_controller_set_transition_type(transition_type_t type)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (type >= TRANSITION_TRANSITION_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(s_controller.state_mutex, portMAX_DELAY);
    s_controller.transition_type = type;
    s_controller.config.transition_type = type;
    xSemaphoreGive(s_controller.state_mutex);
    
    const char *type_names[] = {
        "LINEAR", "SINE", "CUBIC_SPLINE", "QUINTIC_SPLINE", "TRAPEZOIDAL"
    };
    ESP_LOGI(TAG, "Transition type changed to: %s", type_names[type]);
    
    return ESP_OK;
}

esp_err_t aperture_controller_get_status(aperture_status_t *status)
{
    if (!s_initialized || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(s_controller.state_mutex, portMAX_DELAY);
    
    status->state = s_controller.state;
    status->position = (uint16_t)s_controller.current_position;
    status->target_position = (uint16_t)s_controller.target_position;
    status->step_period_us = stepper_driver_get_current_period();
    status->normalized_aperture = aperture_controller_get_normalized_aperture();
    status->normalized_brightness = brightness_encoder_get_expected_brightness(
        status->normalized_aperture);
    status->modulation_cycle_count = s_controller.cycle_count;
    status->is_moving = s_controller.profile.is_active;
    status->instantaneous_velocity = s_controller.profile.current_velocity;
    status->instantaneous_acceleration = s_controller.profile.current_acceleration;
    
    xSemaphoreGive(s_controller.state_mutex);
    
    return ESP_OK;
}

float aperture_controller_get_normalized_aperture(void)
{
    if (!s_initialized) {
        return 0.0f;
    }
    
    // Compute normalized aperture alpha(t) = (A(t) - A_min) / (A_max - A_min)
    // As defined in Section 4.2 of the report
    float pos = (float)s_controller.current_position;
    float max_pos = (float)s_controller.config.max_steps;
    
    float normalized = pos / max_pos;
    
    // Clamp to valid range [0.0, 1.0]
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    
    return normalized;
}

esp_err_t aperture_controller_register_cycle_callback(
    void (*callback)(modulation_pattern_t pattern, void *user_data),
    void *user_data)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(s_controller.state_mutex, portMAX_DELAY);
    s_controller.cycle_callback = callback;
    s_controller.cycle_callback_data = user_data;
    xSemaphoreGive(s_controller.state_mutex);
    
    ESP_LOGI(TAG, "Cycle callback %s", callback ? "registered" : "unregistered");
    
    return ESP_OK;
}

esp_err_t aperture_controller_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }
    
    // Stop all motion
    esp_timer_stop(s_controller.control_timer);
    stepper_driver_emergency_disable();
    
    // Clean up resources
    esp_timer_delete(s_controller.control_timer);
    vSemaphoreDelete(s_controller.state_mutex);
    stepper_driver_deinit();
    brightness_encoder_deinit();
    
    memset(&s_controller, 0, sizeof(controller_context_t));
    s_initialized = false;
    
    ESP_LOGI(TAG, "Aperture controller deinitialized");
    
    return ESP_OK;
}

/*============================================================================
 * INTERNAL FUNCTIONS
 *===========================================================================*/

/**
 * @brief High-frequency control loop callback (1kHz)
 * 
 * This is the core real-time control function executed at 1kHz via timer ISR.
 * It updates the motion profile, computes the next step timing, and commands
 * the stepper driver. Must be efficient - no blocking operations.
 */
static void IRAM_ATTR control_loop_callback(void *arg)
{
    (void)arg;
    
    // Update current position from stepper driver
    s_controller.current_position = stepper_driver_get_position();
    
    // Check for position tracking errors
    if (check_position_fault() != ESP_OK) {
        // Fault detected - emergency handling
        s_controller.state = STATE_ERROR;
        s_controller.modulation_active = false;
        stepper_driver_emergency_disable();
        return;
    }
    
    // Execute modulation or point-to-point motion
    if (s_controller.modulation_active) {
        execute_modulation_step();
    } else if (s_controller.profile.is_active) {
        // Point-to-point motion in progress
        motion_profile_compute(&s_controller.profile, 
                               s_controller.current_position,
                               CONTROL_LOOP_PERIOD_US);
        int16_t next_pos = (int16_t)s_controller.profile.command_position;
        stepper_driver_move_to(next_pos);
    }
    
    // Update status variables for external monitoring
    update_status_variables();
}

/**
 * @brief Execute one step of the periodic modulation pattern
 * 
 * Computes the desired aperture position based on the current phase within
 * the modulation cycle, using the selected pattern function. Updates the
 * motion profile to smoothly track the computed trajectory.
 */
static esp_err_t execute_modulation_step(void)
{
    uint64_t now = esp_timer_get_time();
    uint32_t period_us = 1000000 / s_controller.config.modulation_freq_hz;
    
    // Compute phase within current cycle [0.0, 1.0]
    uint64_t elapsed = now - s_controller.timing.cycle_start_time_us;
    float phase = (float)(elapsed % period_us) / (float)period_us;
    
    // Detect cycle completion
    if (phase < s_controller.timing.phase) {
        // Phase wrapped - new cycle started
        s_controller.cycle_count++;
        s_controller.timing.is_first_half = !s_controller.timing.is_first_half;
        
        // Invoke callback if registered
        if (s_controller.cycle_callback != NULL) {
            // Note: Callback from ISR context - must be brief
            s_controller.cycle_callback(s_controller.timing.current_pattern,
                                        s_controller.cycle_callback_data);
        }
    }
    s_controller.timing.phase = phase;
    
    // Compute desired position from pattern
    float target_norm = compute_pattern_position(phase, s_controller.config.pattern);
    int16_t target_pos = (int16_t)(target_norm * s_controller.config.max_steps);
    
    // Update motion profile for smooth tracking
    if (!s_controller.profile.is_active || 
        abs(target_pos - s_controller.target_position) > 2) {
        // Target changed significantly - replan profile
        s_controller.target_position = target_pos;
        motion_profile_plan_move(&s_controller.profile,
                                 s_controller.current_position,
                                 target_pos);
    }
    
    // Compute profile update
    motion_profile_compute(&s_controller.profile,
                           s_controller.current_position,
                           CONTROL_LOOP_PERIOD_US);
    
    // Command stepper
    int16_t cmd_pos = (int16_t)s_controller.profile.command_position;
    stepper_driver_move_to(cmd_pos);
    
    return ESP_OK;
}

/**
 * @brief Compute normalized position from modulation pattern
 * 
 * Implements the aperture functions described in Section 4.3:
 * - Closing: A(t) = A_max - k*t (brightness decreases)
 * - Opening: A(t) = A_min + k*t (brightness increases)
 * - Triangular: Alternating closing/opening each half-cycle
 * 
 * @param phase Current phase within cycle [0.0, 1.0]
 * @param pattern Modulation pattern type
 * @return Normalized position [0.0, 1.0]
 */
static float compute_pattern_position(float phase, modulation_pattern_t pattern)
{
    float position = 0.0f;
    
    switch (pattern) {
        case PATTERN_CLOSING:
            // Linear closing: 1.0 -> 0.0
            // Corresponds to A(t) = A_max - k*t
            position = 1.0f - phase;
            break;
            
        case PATTERN_OPENING:
            // Linear opening: 0.0 -> 1.0
            // Corresponds to A(t) = A_min + k*t
            position = phase;
            break;
            
        case PATTERN_TRIANGULAR:
            // Triangular wave: closing then opening
            // First half (0.0-0.5): closing (1.0 -> 0.0)
            // Second half (0.5-1.0): opening (0.0 -> 1.0)
            if (phase < 0.5f) {
                position = 1.0f - (phase * 2.0f);
            } else {
                position = (phase - 0.5f) * 2.0f;
            }
            break;
            
        case PATTERN_SAWTOOTH:
            // Sawtooth: rapid close, slow open
            if (phase < 0.1f) {
                // Rapid closing (10% of cycle)
                position = 1.0f - (phase * 10.0f);
            } else {
                // Slow opening (90% of cycle)
                position = (phase - 0.1f) / 0.9f;
            }
            break;
            
        case PATTERN_CUSTOM:
            // Custom pattern - could load from lookup table
            // Default to sinusoidal for demonstration
            position = 0.5f * (1.0f + sinf(phase * 2.0f * M_PI - M_PI_2));
            break;
            
        default:
            position = 0.5f;  // Safe default - mid position
            break;
    }
    
    // Apply direction inversion if configured
    if (s_controller.config.invert_direction) {
        position = 1.0f - position;
    }
    
    // Ensure valid range
    if (position < 0.0f) position = 0.0f;
    if (position > 1.0f) position = 1.0f;
    
    return position;
}

/**
 * @brief Check for position tracking faults
 * 
 * Compares commanded vs actual position to detect step loss or mechanical
 * faults. If error exceeds threshold, sets error state.
 */
static esp_err_t check_position_fault(void)
{
    int16_t error = s_controller.commanded_position - s_controller.current_position;
    if (abs(error) > MAX_POSITION_ERROR_STEPS) {
        ESP_LOGE(TAG, "Position fault: error=%d steps", error);
        s_controller.error_flags |= 0x02;  // Tracking error flag
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Update status variables for external monitoring
 * 
 * Computes derived quantities like normalized aperture and brightness
 * for use by the decoder algorithm (Section 5.4).
 */
static void update_status_variables(void)
{
    // Update commanded position for fault detection
    s_controller.commanded_position = (int16_t)s_controller.profile.command_position;
}