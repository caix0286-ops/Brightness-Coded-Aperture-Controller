/**
 * @file stepper_driver.c
 * @brief Implementation of 4-wire 2-phase stepper motor driver
 * 
 * This driver implements precise position control for the aperture mechanism
 * using GPIO bit-banging with hardware timer support. The ESP32's MCPWM
 * or LEDC peripherals could alternatively be used for higher performance.
 * 
 * For the brightness-coded motion capture application, the driver must:
 * - Support step periods from 800us to 10ms for speed range
 * - Maintain open-loop position tracking for modulation sync
 * - Provide clean step pulses to prevent driver misinterpretation
 * - Handle enable/disable for power saving and safety
 * 
 * @see Section 5.1: Image Acquisition - Hardware considerations
 */

#include "stepper_driver.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

#define TAG "STEPPER"

/*============================================================================
 * INTERNAL STATE
 *===========================================================================*/

/**
 * @brief Stepper driver runtime state
 */
typedef struct {
    int16_t current_position;       /**< Tracked position (open-loop) */
    int16_t target_position;        /**< Commanded target position */
    uint32_t current_period_us;     /**< Current step period */
    uint32_t min_period_us;         /**< Minimum allowed period */
    bool is_enabled;                /**< Driver enable state */
    bool is_moving;                 /**< True if motion in progress */
    int8_t direction;               /**< Current direction: +1 or -1 */
    esp_timer_handle_t step_timer;  /**< High-res timer for step generation */
} stepper_state_t;

static stepper_state_t s_state = {0};

/*============================================================================
 * STEP SEQUENCE TABLE (4-wire 2-phase full step)
 *===========================================================================*/

/**
 * @brief Full-step sequence for 4-wire 2-phase motor
 * 
 * Step | Phase A | Phase B
 * -----|---------|--------
 *   0  |   ON    |   ON
 *   1  |   OFF   |   ON
 *   2  |   OFF   |   OFF
 *   3  |   ON    |   OFF
 * 
 * This sequence provides maximum torque. For smoother motion, half-step
 * or microstepping sequences could be implemented.
 */
static const uint8_t s_full_step_table[4][2] = {
    {1, 1},   // Step 0: A=ON, B=ON
    {0, 1},   // Step 1: A=OFF, B=ON
    {0, 0},   // Step 2: A=OFF, B=OFF
    {1, 0}    // Step 3: A=ON, B=OFF
};

/*============================================================================
 * FORWARD DECLARATIONS
 *===========================================================================*/

static void step_timer_callback(void *arg);
static void generate_step_pulse(void);
static void set_direction(bool clockwise);

/*============================================================================
 * PUBLIC API
 *===========================================================================*/

esp_err_t stepper_driver_init(void)
{
    ESP_LOGI(TAG, "Initializing stepper driver");
    
    // Configure GPIO outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STEPPER_STEP_GPIO) | 
                        (1ULL << STEPPER_DIR_GPIO) |
                        (1ULL << STEPPER_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) return err;
    
    // Configure home switch input with pull-up
    gpio_config_t home_conf = {
        .pin_bit_mask = (1ULL << STEPPER_HOME_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    err = gpio_config(&home_conf);
    if (err != ESP_OK) return err;
    
    // Set initial states
    gpio_set_level(STEPPER_STEP_GPIO, 0);
    gpio_set_level(STEPPER_DIR_GPIO, 0);
    gpio_set_level(STEPPER_ENABLE_GPIO, 1);  // Disable initially (active low)
    
    // Initialize state
    s_state.current_position = 0;
    s_state.target_position = 0;
    s_state.current_period_us = STEPPER_MAX_PERIOD_US;
    s_state.min_period_us = STEPPER_MIN_PERIOD_US;
    s_state.is_enabled = false;
    s_state.is_moving = false;
    s_state.direction = 1;
    
    // Create step generation timer
    const esp_timer_create_args_t timer_args = {
        .callback = &step_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "stepper_step",
        .skip_unhandled_events = true  // Skip if handler running
    };
    
    err = esp_timer_create(&timer_args, &s_state.step_timer);
    if (err != ESP_OK) return err;
    
    ESP_LOGI(TAG, "Stepper driver ready (STEP:%d, DIR:%d, EN:%d, HOME:%d)",
             STEPPER_STEP_GPIO, STEPPER_DIR_GPIO, 
             STEPPER_ENABLE_GPIO, STEPPER_HOME_GPIO);
    
    return ESP_OK;
}

esp_err_t stepper_driver_move_to(int16_t position)
{
    if (!s_state.is_enabled) {
        ESP_LOGW(TAG, "Move command while disabled - enabling driver");
        stepper_driver_set_enable(true);
    }
    
    // Update target
    s_state.target_position = position;
    
    // Determine direction
    int16_t delta = position - s_state.current_position;
    if (delta > 0) {
        s_state.direction = 1;
        set_direction(true);  // Clockwise
    } else if (delta < 0) {
        s_state.direction = -1;
        set_direction(false); // Counter-clockwise
    } else {
        s_state.is_moving = false;
        return ESP_OK;  // Already at target
    }
    
    // Start step generation if not already running
    if (!s_state.is_moving) {
        s_state.is_moving = true;
        
        // Start timer with current period
        esp_err_t err = esp_timer_start_periodic(s_state.step_timer, 
                                                  s_state.current_period_us);
        if (err != ESP_OK) {
            s_state.is_moving = false;
            return err;
        }
    }
    
    return ESP_OK;
}

int16_t stepper_driver_get_position(void)
{
    return s_state.current_position;
}

uint32_t stepper_driver_get_current_period(void)
{
    return s_state.current_period_us;
}

esp_err_t stepper_driver_home(uint32_t step_period_us, uint32_t timeout_ms)
{
    ESP_LOGI(TAG, "Homing at %d us/step, timeout %d ms", step_period_us, timeout_ms);
    
    stepper_driver_set_enable(true);
    
    // Set direction toward home (negative direction assumed)
    set_direction(false);
    s_state.direction = -1;
    
    // Configure step period
    s_state.current_period_us = step_period_us;
    
    // Start stepping toward home
    s_state.is_moving = true;
    esp_err_t err = esp_timer_start_periodic(s_state.step_timer, step_period_us);
    if (err != ESP_OK) return err;
    
    // Wait for home switch or timeout
    uint32_t elapsed_ms = 0;
    while (elapsed_ms < timeout_ms) {
        // Check home switch (active low with pull-up)
        if (gpio_get_level(STEPPER_HOME_GPIO) == 0) {
            // Home detected
            esp_timer_stop(s_state.step_timer);
            s_state.is_moving = false;
            s_state.current_position = 0;
            s_state.target_position = 0;
            
            ESP_LOGI(TAG, "Home position found after %d ms", elapsed_ms);
            return ESP_OK;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed_ms += 10;
    }
    
    // Timeout - stop motion
    esp_timer_stop(s_state.step_timer);
    s_state.is_moving = false;
    stepper_driver_set_enable(false);
    
    ESP_LOGE(TAG, "Homing timeout after %d ms", timeout_ms);
    return ESP_ERR_TIMEOUT;
}

void stepper_driver_emergency_disable(void)
{
    // Stop timer immediately
    esp_timer_stop(s_state.step_timer);
    s_state.is_moving = false;
    
    // Disable driver outputs
    gpio_set_level(STEPPER_ENABLE_GPIO, 1);  // Active high = disabled
    s_state.is_enabled = false;
    
    ESP_LOGW(TAG, "Emergency disable executed");
}

void stepper_driver_set_enable(bool enable)
{
    gpio_set_level(STEPPER_ENABLE_GPIO, enable ? 0 : 1);  // Active low
    s_state.is_enabled = enable;
    
    if (enable) {
        ESP_LOGD(TAG, "Driver enabled");
    } else {
        ESP_LOGD(TAG, "Driver disabled");
    }
}

esp_err_t stepper_driver_deinit(void)
{
    esp_timer_stop(s_state.step_timer);
    esp_timer_delete(s_state.step_timer);
    
    gpio_set_level(STEPPER_ENABLE_GPIO, 1);  // Disable
    gpio_reset_pin(STEPPER_STEP_GPIO);
    gpio_reset_pin(STEPPER_DIR_GPIO);
    gpio_reset_pin(STEPPER_ENABLE_GPIO);
    gpio_reset_pin(STEPPER_HOME_GPIO);
    
    memset(&s_state, 0, sizeof(stepper_state_t));
    
    return ESP_OK;
}

/*============================================================================
 * INTERNAL FUNCTIONS
 *===========================================================================*/

/**
 * @brief Timer callback for step generation
 * 
 * Called periodically at the current step period. Generates a step pulse
 * and updates position tracking. Stops when target reached.
 */
static void IRAM_ATTR step_timer_callback(void *arg)
{
    (void)arg;
    
    if (!s_state.is_moving) {
        return;
    }
    
    // Check if target reached
    int16_t delta = s_state.target_position - s_state.current_position;
    
    // Check direction consistency
    if ((delta > 0 && s_state.direction < 0) || 
        (delta < 0 && s_state.direction > 0)) {
        // Direction mismatch - should not happen with proper planning
        s_state.is_moving = false;
        return;
    }
    
    if (delta == 0) {
        // Target reached
        s_state.is_moving = false;
        esp_timer_stop(s_state.step_timer);
        return;
    }
    
    // Generate step pulse
    generate_step_pulse();
    
    // Update position tracking
    s_state.current_position += s_state.direction;
}

/**
 * @brief Generate a single step pulse on STEP pin
 * 
 * Creates a clean pulse with adequate duration for the stepper driver
 * to register. Pulse width ~10 microseconds.
 */
static void IRAM_ATTR generate_step_pulse(void)
{
    // Set step high
    gpio_set_level(STEPPER_STEP_GPIO, 1);
    
    // Short delay for pulse width (using ets_delay_us in IRAM context)
    ets_delay_us(10);
    
    // Set step low
    gpio_set_level(STEPPER_STEP_GPIO, 0);
}

/**
 * @brief Set direction output
 * 
 * @param clockwise true for clockwise, false for counter-clockwise
 */
static void set_direction(bool clockwise)
{
    gpio_set_level(STEPPER_DIR_GPIO, clockwise ? 1 : 0);
}