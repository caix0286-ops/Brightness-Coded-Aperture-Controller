/**
 * @file main.c
 * @brief Main application entry point for brightness-coded aperture controller
 * 
 * This is the main application for the ESP32-based brightness-coded aperture
 * control system, implementing the hardware component of the IERG5230 Final
 * Project on high-speed motion capture using continuously changing aperture.
 * 
 * The application demonstrates:
 * - Aperture controller initialization and configuration
 * - Smooth motion profile selection (runtime switchable)
 * - Periodic brightness-coded modulation at 30Hz
 * - Status monitoring and debugging output
 * - Integration with the theoretical encoding model
 * 
 * Hardware setup:
 * - ESP32 development board
 * - 4-wire 2-phase stepper motor (e.g., NEMA 17)
 * - Stepper driver module (DRV8825 or A4988)
 * - Variable aperture mechanism attached to motor shaft
 * - Optional: home limit switch
 * 
 * @see Full project report: "High-speed Motion Capture Using Continuously 
 *      Changing Aperture" - IERG5230 Final Project
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "aperture_controller.h"
#include "stepper_driver.h"
#include "motion_profile.h"
#include "transition_curves.h"
#include "brightness_encoder.h"

#define TAG "MAIN"

/*============================================================================
 * TASK DEFINITIONS
 *===========================================================================*/

/**
 * @brief Control task handle
 */
static TaskHandle_t s_control_task = NULL;

/**
 * @brief Monitor task handle
 */
static TaskHandle_t s_monitor_task = NULL;

/*============================================================================
 * FUNCTION PROTOTYPES
 *===========================================================================*/

static void control_task(void *pvParameters);
static void monitor_task(void *pvParameters);
static void cycle_callback(modulation_pattern_t pattern, void *user_data);
static void print_system_info(void);
static void demonstrate_profiles(void);

/*============================================================================
 * MAIN ENTRY POINT
 *===========================================================================*/

void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "Brightness-Coded Aperture Controller");
    ESP_LOGI(TAG, "IERG5230 Final Project - ESP32 Implementation");
    ESP_LOGI(TAG, "==============================================");
    
    // Initialize NVS (required for ESP32)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    // Print system information
    print_system_info();
    
    // Validate all transition curves mathematically
    ESP_LOGI(TAG, "Validating transition curves...");
    for (int i = 0; i < TRANSITION_TRANSITION_COUNT; i++) {
        transition_validate(i);
    }
    
    // Initialize aperture controller
    ESP_LOGI(TAG, "Initializing aperture controller...");
    err = aperture_controller_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Controller initialization failed: %s", esp_err_to_name(err));
        return;
    }
    
    // Register cycle callback for camera synchronization
    aperture_controller_register_cycle_callback(cycle_callback, NULL);
    
    // Configure default parameters
    aperture_config_t config = {
        .modulation_freq_hz = 30,           // 30Hz for 60fps camera
        .max_steps = APERTURE_MAX_STEPS,     // 500 steps full range
        .current_position = 0,
        .transition_type = TRANSITION_QUINTIC_SPLINE,  // Smoothest profile
        .pattern = PATTERN_TRIANGULAR,       // Alternating open/close
        .invert_direction = false,
        .home_speed_us = 5000,               // Slow homing for reliability
        .exposure_time_ms = 16.667f          // 1/60 second exposure
    };
    
    err = aperture_controller_configure(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Configuration failed: %s", esp_err_to_name(err));
        aperture_controller_deinit();
        return;
    }
    
    // Perform homing sequence
    ESP_LOGI(TAG, "Performing homing sequence...");
    err = aperture_controller_home();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Homing failed: %s", esp_err_to_name(err));
        aperture_controller_deinit();
        return;
    }
    
    // Demonstrate different motion profiles
    demonstrate_profiles();
    
    // Create monitor task for periodic status reporting
    xTaskCreate(monitor_task, "monitor", 4096, NULL, 5, &s_monitor_task);
    
    // Start modulation
    ESP_LOGI(TAG, "Starting brightness-coded modulation...");
    err = aperture_controller_start_modulation();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start modulation: %s", esp_err_to_name(err));
        aperture_controller_deinit();
        return;
    }
    
    // Main loop - handle commands, configuration changes, etc.
    ESP_LOGI(TAG, "System running. Commands:");
    ESP_LOGI(TAG, "  's' - Stop modulation");
    ESP_LOGI(TAG, "  'r' - Restart modulation");
    ESP_LOGI(TAG, "  '1' - Linear profile");
    ESP_LOGI(TAG, "  '2' - Sine profile");
    ESP_LOGI(TAG, "  '3' - Cubic spline profile");
    ESP_LOGI(TAG, "  '4' - Quintic spline profile (recommended)");
    ESP_LOGI(TAG, "  '5' - Trapezoidal profile");
    ESP_LOGI(TAG, "  'h' - Home aperture");
    ESP_LOGI(TAG, "  'e' - Emergency stop");
    
    while (1) {
        // Simple command interface via serial
        int cmd = getchar();
        if (cmd != EOF) {
            switch (cmd) {
                case 's':
                    ESP_LOGI(TAG, "Command: Stop modulation");
                    aperture_controller_stop();
                    break;
                    
                case 'r':
                    ESP_LOGI(TAG, "Command: Restart modulation");
                    aperture_controller_start_modulation();
                    break;
                    
                case '1':
                    ESP_LOGI(TAG, "Command: Switch to LINEAR profile");
                    aperture_controller_set_transition_type(TRANSITION_LINEAR);
                    break;
                    
                case '2':
                    ESP_LOGI(TAG, "Command: Switch to SINE profile");
                    aperture_controller_set_transition_type(TRANSITION_SINE);
                    break;
                    
                case '3':
                    ESP_LOGI(TAG, "Command: Switch to CUBIC SPLINE profile");
                    aperture_controller_set_transition_type(TRANSITION_CUBIC_SPLINE);
                    break;
                    
                case '4':
                    ESP_LOGI(TAG, "Command: Switch to QUINTIC SPLINE profile");
                    aperture_controller_set_transition_type(TRANSITION_QUINTIC_SPLINE);
                    break;
                    
                case '5':
                    ESP_LOGI(TAG, "Command: Switch to TRAPEZOIDAL profile");
                    aperture_controller_set_transition_type(TRANSITION_TRAPEZOIDAL);
                    break;
                    
                case 'h':
                    ESP_LOGI(TAG, "Command: Home aperture");
                    aperture_controller_stop();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    aperture_controller_home();
                    break;
                    
                case 'e':
                    ESP_LOGW(TAG, "Command: EMERGENCY STOP");
                    aperture_controller_emergency_stop();
                    break;
                    
                default:
                    break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz command polling
    }
}

/*============================================================================
 * TASK IMPLEMENTATIONS
 *===========================================================================*/

/**
 * @brief Monitor task - periodic status reporting
 * 
 * Prints current aperture status including position, velocity, brightness
 * encoding values, and modulation cycle count. Useful for debugging
 * and verifying correct operation.
 */
static void monitor_task(void *pvParameters)
{
    (void)pvParameters;
    
    aperture_status_t status;
    const int report_interval_ms = 1000;  // 1 second reporting interval
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(report_interval_ms));
        
        esp_err_t err = aperture_controller_get_status(&status);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get status");
            continue;
        }
        
        // Print formatted status
        ESP_LOGI(TAG, "=== APERTURE STATUS ===");
        ESP_LOGI(TAG, "State: %d, Position: %d/%d", 
                 status.state, status.position, APERTURE_MAX_STEPS);
        ESP_LOGI(TAG, "Normalized: %.3f, Brightness: %.3f",
                 status.normalized_aperture, status.normalized_brightness);
        ESP_LOGI(TAG, "Velocity: %.1f steps/s, Accel: %.1f steps/s^2",
                 status.instantaneous_velocity, status.instantaneous_acceleration);
        ESP_LOGI(TAG, "Step period: %d us, Cycles: %d",
                 status.step_period_us, status.modulation_cycle_count);
        ESP_LOGI(TAG, "Moving: %s", status.is_moving ? "YES" : "NO");
        ESP_LOGI(TAG, "=======================");
    }
}

/*============================================================================
 * CALLBACK AND UTILITY FUNCTIONS
 *===========================================================================*/

/**
 * @brief Modulation cycle completion callback
 * 
 * Called at the start of each modulation cycle. Can be used to trigger
 * camera shutter or synchronize external devices.
 * 
 * @param pattern Current modulation pattern phase
 * @param user_data User context (unused)
 */
static void cycle_callback(modulation_pattern_t pattern, void *user_data)
{
    (void)user_data;
    
    const char *pattern_name = "UNKNOWN";
    switch (pattern) {
        case PATTERN_CLOSING: pattern_name = "CLOSING"; break;
        case PATTERN_OPENING: pattern_name = "OPENING"; break;
        case PATTERN_TRIANGULAR: pattern_name = "TRIANGULAR"; break;
        case PATTERN_SAWTOOTH: pattern_name = "SAWTOOTH"; break;
        case PATTERN_CUSTOM: pattern_name = "CUSTOM"; break;
    }
    
    // This is where camera trigger would be synchronized
    // For now, just log the cycle transition
    ESP_LOGD(TAG, "Cycle callback: pattern=%s", pattern_name);
}

/**
 * @brief Print system information
 * 
 * Displays ESP32 hardware details and project configuration.
 */
static void print_system_info(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "ESP32 Hardware Info:");
    ESP_LOGI(TAG, "  Cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "  Revision: %d", chip_info.revision);
    ESP_LOGI(TAG, "  Features: %s%s%s",
             (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi " : "",
             (chip_info.features & CHIP_FEATURE_BT) ? "BT " : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "BLE " : "");
    
    ESP_LOGI(TAG, "Project Configuration:");
    ESP_LOGI(TAG, "  Max aperture steps: %d", APERTURE_MAX_STEPS);
    ESP_LOGI(TAG, "  Default modulation: %d Hz", DEFAULT_MODULATION_FREQ_HZ);
    ESP_LOGI(TAG, "  Control loop: %d Hz", 1000000 / 1000);  // 1kHz
    ESP_LOGI(TAG, "  Brightness levels: %d", BRIGHTNESS_LEVELS);
}

/**
 * @brief Demonstrate different motion profiles
 * 
 * Performs short test moves with each profile type to demonstrate
 * the smooth motion capabilities and allow audible/mechanical comparison.
 */
static void demonstrate_profiles(void)
{
    ESP_LOGI(TAG, "=== MOTION PROFILE DEMONSTRATION ===");
    
    const char *profile_names[] = {
        "LINEAR", "SINE", "CUBIC_SPLINE", "QUINTIC_SPLINE", "TRAPEZOIDAL"
    };
    
    // Test each profile with a small move
    for (int i = 0; i < TRANSITION_TRANSITION_COUNT; i++) {
        ESP_LOGI(TAG, "Testing profile: %s", profile_names[i]);
        
        // Set profile type
        aperture_controller_set_transition_type((transition_type_t)i);
        
        // Move to mid position
        aperture_controller_move_to(APERTURE_MAX_STEPS / 2);
        vTaskDelay(pdMS_TO_TICKS(500));  // Wait for motion
        
        // Return to start
        aperture_controller_move_to(0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Restore default (quintic spline)
    aperture_controller_set_transition_type(TRANSITION_QUINTIC_SPLINE);
    
    ESP_LOGI(TAG, "=== DEMONSTRATION COMPLETE ===");
    ESP_LOGI(TAG, "Recommended profile: QUINTIC_SPLINE (smoothest, minimal vibration)");
}