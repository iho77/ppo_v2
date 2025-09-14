/**
 * @file sensor_manager.c
 * @brief Sensor manager implementation using ADS1115 for O2 sensor readings
 */

#include "sdkconfig.h"
#include "sensor_manager.h"
// Using internal ADC only - no external sensor managers needed
#include "app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "SENSOR_MGR";
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

// Constants & definitions for robust filtering system
#define FILTER_SAMPLE_RATE_HZ           20.0f       // Nominal sample rate
#define FILTER_MEDIAN_WINDOW_SIZE       5           // Median-of-5 filter
#define FILTER_DEFAULT_TAU_SEC          5.0f        // EMA time constant (seconds)
#define FILTER_DEFAULT_S_UP_BAR_S       0.083f      // Default rising PPO2 rate limit (bar/s)
#define FILTER_DEFAULT_S_DOWN_BAR_S     0.05f       // Default falling PPO2 rate limit (bar/s)
// Alarm thresholds removed - handled by warning_manager component

// Battery measurement constants
#define BATTERY_VOLTAGE_DIVIDER_RATIO   3.0f        // Input voltage = measured * 3.0
#define BATTERY_FULL_VOLTAGE_V          3.3f        // Full charge voltage
#define BATTERY_LOW_VOLTAGE_V           2.8f        // Low voltage threshold  
#define BATTERY_HALF_VOLTAGE_V          3.1f        // Half charge voltage
#define BATTERY_TAU_SEC                 30.0f       // Slower filtering for battery (30 seconds)

// Robust filter parameters structure
typedef struct {
    float fs_hz;                    // Sample rate (Hz)
    float dt;                       // Sample period (1/fs_hz)
    float alpha;                    // EMA coefficient (dt/tau_sec)
    float tau_sec;                  // EMA time constant (seconds)
    
    float k_mV_per_bar;             // Sensor sensitivity (mV/bar)
    float clamp_up_mV_per_sample;   // Rising slew rate limit (mV/sample)
    float clamp_dn_mV_per_sample;   // Falling slew rate limit (mV/sample)
    
    float mv_min_plausible;         // Minimum plausible voltage (mV)
    float mv_max_plausible;         // Maximum plausible voltage (mV)
} robust_filter_params_t;

// Robust filter state structure
typedef struct {
    // Median-of-5 buffer (circular)
    float med_buf[FILTER_MEDIAN_WINDOW_SIZE];
    uint8_t med_count;              // Number of samples collected (0-5)
    uint8_t med_index;              // Current insertion index
    
    // EMA state
    float y_ema_mV;                 // Current EMA output
    
    // Slew-rate limiter state
    float y_lim_mV;                 // Current slew-limited output
    
    bool initialized;               // Filter initialization flag
} robust_filter_state_t;

// Output structure for robust smoothing step
typedef struct {
    float raw_mV;                   // Input raw voltage
    float med_mV;                   // Median-filtered voltage
    float y_ema_mV;                 // EMA output voltage
    float y_lim_mV;                 // Final slew-limited voltage
} smooth_step_output_t;

// Filter instances for dual sensors
static robust_filter_params_t s_sensor1_params = {0};
static robust_filter_params_t s_sensor2_params = {0};
static robust_filter_state_t s_sensor1_state = {0};
static robust_filter_state_t s_sensor2_state = {0};
static robust_filter_state_t s_battery_state = {0};  // Battery voltage filter state

// Robust filter function declarations
static void robust_filter_compute_clamps_for_two_sensors(float fs_hz, 
                                                        float k1_mV_per_bar, float k2_mV_per_bar,
                                                        float S_up_bar_s, float S_down_bar_s,
                                                        float *clamp_up_1_mV, float *clamp_dn_1_mV,
                                                        float *clamp_up_2_mV, float *clamp_dn_2_mV);
static void robust_filter_make_params(robust_filter_params_t *params, float fs_hz, float tau_sec,
                                     float k_mV_per_bar, float clamp_up_mV, float clamp_dn_mV);
static void robust_filter_reset_state(robust_filter_state_t *state);
static float robust_filter_median_of_5(float *buffer, uint8_t count);
static smooth_step_output_t robust_smooth_step(robust_filter_params_t *params, 
                                              robust_filter_state_t *state, 
                                              float raw_mV);
static void robust_filter_update_calibration_sensitivity(void);

// Test failure simulation (enable with #define SENSOR_FAILURE_TEST)
//#define SENSOR_FAILURE_TEST

// Test sensor reading stub (enable with #define SENSOR_READING_TEST)
//#define SENSOR_READING_TEST

// Test warning conditions (enable with #define WARNING_TEST)
//#define WARNING_TEST

#ifdef SENSOR_FAILURE_TEST
// Test failure modes - change these to test different scenarios
#define TEST_O2_COMM_FAILURE        0   // Set to 1 to simulate O2 communication failure
#define TEST_O2_OUT_OF_RANGE        0   // Set to 1 to simulate O2 out of range
// TEST_PRESSURE_COMM_FAILURE removed - no pressure sensor used
#define TEST_CALIBRATION_INVALID    0   // Set to 1 to simulate invalid calibration
#define TEST_STALE_DATA             0   // Set to 1 to simulate stale data (delays updates)

static uint32_t s_test_counter = 0;  // Counter for test scenarios
#endif

static bool s_initialized = false;
static sensor_data_t s_current_data = {0};
static uint32_t s_read_count = 0;
// No pressure sensor data needed - using fixed atmospheric pressure

// Safety constants
#define MAX_DATA_AGE_MS         5000    // Maximum age for sensor data (5 seconds)
#define MAX_CONSECUTIVE_FAILURES 10     // Maximum consecutive failures before critical error
#define FAIL_SAFE_FO2           0.16    // Conservative fail-safe FO2 (16% instead of 21%)
#define MIN_CALIBRATION_GAS_O2  0.18    // Minimum O2 fraction for calibration gas (18%)
#define MAX_CALIBRATION_GAS_O2  1.0     // Maximum O2 fraction for calibration gas (100%)

// Recovery thresholds
#define RECOVERY_ATTEMPT_THRESHOLD  5   // Start recovery attempts after 5 consecutive failures
#define SENSOR_REINIT_THRESHOLD    15   // Re-initialize sensors after 15 consecutive failures  
#define SYSTEM_RESET_THRESHOLD     30   // System reset after 30 consecutive failures

// Failure tracking
static uint32_t s_consecutive_o2_failures = 0;
// s_consecutive_pressure_failures removed - no pressure sensor used

// Recovery state tracking
static uint32_t s_last_recovery_attempt_time = 0;
static uint32_t s_sensor_reinit_count = 0;
static bool s_system_in_recovery = false;



#define EXAMPLE_ADC_ATTEN       ADC_ATTEN_DB_2_5
#define ADC_WIDTH               ADC_BITWIDTH_12   // 12-bit resolution

// ADC channels for sensors (ESP32-C3 specific)
// GPIO9 -> ADC1_CHANNEL_9 is not valid on ESP32-C3, use GPIO0-4
// Let's use GPIO0 and GPIO1 instead which map to ADC1_CHANNEL_0 and ADC1_CHANNEL_1
#define SENSOR1_ADC_CHANNEL     ADC_CHANNEL_0   // GPIO0 for ESP32-C3
#define SENSOR2_ADC_CHANNEL     ADC_CHANNEL_1   // GPIO1 for ESP32-C3
#define BATTERY_ADC_CHANNEL     ADC_CHANNEL_2   // GPIO2 for ESP32-C3 (battery voltage divider)

// Internal ADC handles
static adc_oneshot_unit_handle_t s_adc1_handle = NULL;
static adc_cali_handle_t s_adc1_cali_sensor1_handle = NULL;
static adc_cali_handle_t s_adc1_cali_sensor2_handle = NULL;
static adc_cali_handle_t s_adc1_cali_battery_handle = NULL;
static bool s_adc_calibrated_sensor1 = false;
static bool s_adc_calibrated_sensor2 = false;
static bool s_adc_calibrated_battery = false;

// Legacy calibration data - no longer used (kept for compatibility)
// All calibration now handled by multipoint calibration system

#ifdef SENSOR_READING_TEST
// Test stub variables for sensor reading simulation
static float s_test_sensor1_mv = 0.5f;  // Start at 0.5mV
static float s_test_sensor2_mv = 0.5f;  // Start at 0.5mV
static bool s_test_sensor1_ascending = true;  // Direction for sensor 1
static bool s_test_sensor2_ascending = true;  // Direction for sensor 2
#endif

// ADC calibration helper functions
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);

#ifdef SENSOR_FAILURE_TEST
// Test stub functions for simulating internal ADC sensor failures
static esp_err_t test_internal_adc_read(uint8_t channel, float *voltage_mv)
{
    s_test_counter++;
    
    if (TEST_O2_COMM_FAILURE && (s_test_counter % 3 == 0)) {
        ESP_LOGW(TAG, "TEST: Simulating ADC communication failure");
        return ESP_ERR_TIMEOUT;
    }
    
    if (TEST_O2_OUT_OF_RANGE) {
        // Return values outside expected range
        if (s_test_counter % 2 == 0) {
            *voltage_mv = -5.0f;  // Below range
            ESP_LOGW(TAG, "TEST: Simulating ADC out of range (low): %.1fmV", *voltage_mv);
        } else {
            *voltage_mv = 50.0f;  // Above range
            ESP_LOGW(TAG, "TEST: Simulating ADC out of range (high): %.1fmV", *voltage_mv);
        }
        return ESP_OK;
    }
    
    // Normal operation - return realistic sensor value
    *voltage_mv = 9.0f + (s_test_counter % 10) * 0.1f;  // 9.0-9.9mV range
    return ESP_OK;
}

// No pressure sensor used - using fixed atmospheric pressure (1.013 bar)

static const app_config_t* test_app_config_get_current(void)
{
    static app_config_t test_config = {
        .o2_cal = {
            .calibration_gas_o2_fraction = TEST_CALIBRATION_INVALID ? 0.15f : 0.21f,  // Invalid fraction if testing
            // .calibration_sensor_mv = TEST_CALIBRATION_INVALID ? 0.0f : 8.5f, // COMMENTED OUT: conflicts with modern calibration system
            .calibrated = TEST_CALIBRATION_INVALID ? false : true  // Mark as calibrated unless testing invalid
        },
        .o2_cal_sensor2 = {
            .calibration_gas_o2_fraction = TEST_CALIBRATION_INVALID ? 0.15f : 0.21f,
            // .calibration_sensor_mv = TEST_CALIBRATION_INVALID ? 0.0f : 8.7f, // COMMENTED OUT: conflicts with modern calibration system
            .calibrated = TEST_CALIBRATION_INVALID ? false : true  // Mark as calibrated unless testing invalid
        }
    };
    
    if (TEST_CALIBRATION_INVALID) {
        ESP_LOGW(TAG, "TEST: Simulating invalid calibration: O2=%.3f",
                 test_config.o2_cal.calibration_gas_o2_fraction);
    } else {
        ESP_LOGI(TAG, "TEST: Using valid calibration: S1(O2=%.3f) S2(O2=%.3f)",
                 test_config.o2_cal.calibration_gas_o2_fraction,
                 test_config.o2_cal_sensor2.calibration_gas_o2_fraction);
    }
    
    return &test_config;
}

// Test control functions
void sensor_manager_enable_test_mode(void)
{
    ESP_LOGW(TAG, "TEST MODE ENABLED - Sensor readings are simulated!");
    ESP_LOGW(TAG, "Test flags: O2_COMM=%d, O2_RANGE=%d, PRESS_COMM=%d, CAL_INV=%d, STALE=%d",
             TEST_O2_COMM_FAILURE, TEST_O2_OUT_OF_RANGE, TEST_PRESSURE_COMM_FAILURE, 
             TEST_CALIBRATION_INVALID, TEST_STALE_DATA);
}

#define internal_adc_read_voltage test_internal_adc_read
#define app_config_get_current test_app_config_get_current

#else
// Normal operation - add a function to indicate test mode is disabled
void sensor_manager_enable_test_mode(void)
{
    ESP_LOGI(TAG, "Test mode is not compiled in. Define SENSOR_FAILURE_TEST to enable.");
}
#endif

// Robust filter function implementations

/**
 * @brief Compute per-sample clamp magnitudes for two sensors from their measured k and physical slew limits
 */
static void robust_filter_compute_clamps_for_two_sensors(float fs_hz, 
                                                        float k1_mV_per_bar, float k2_mV_per_bar,
                                                        float S_up_bar_s, float S_down_bar_s,
                                                        float *clamp_up_1_mV, float *clamp_dn_1_mV,
                                                        float *clamp_up_2_mV, float *clamp_dn_2_mV)
{
    float dt = 1.0f / fs_hz;
    
    *clamp_up_1_mV = k1_mV_per_bar * S_up_bar_s * dt;
    *clamp_dn_1_mV = k1_mV_per_bar * S_down_bar_s * dt;
    
    *clamp_up_2_mV = k2_mV_per_bar * S_up_bar_s * dt;
    *clamp_dn_2_mV = k2_mV_per_bar * S_down_bar_s * dt;
    
    ESP_LOGD(TAG, "Clamps computed: S1(up=%.3f, dn=%.3f) S2(up=%.3f, dn=%.3f) mV/sample",
             *clamp_up_1_mV, *clamp_dn_1_mV, *clamp_up_2_mV, *clamp_dn_2_mV);
}

/**
 * @brief Initialize robust filter parameters
 */
static void robust_filter_make_params(robust_filter_params_t *params, float fs_hz, float tau_sec,
                                     float k_mV_per_bar, float clamp_up_mV, float clamp_dn_mV)
{
    if (!params) return;
    
    params->fs_hz = fs_hz;
    params->dt = 1.0f / fs_hz;
    params->tau_sec = tau_sec;
    params->alpha = params->dt / tau_sec;
    
    // Clamp alpha to (0,1]
    if (params->alpha <= 0.0f) params->alpha = 0.001f;
    if (params->alpha > 1.0f) params->alpha = 1.0f;
    
    params->k_mV_per_bar = k_mV_per_bar;
    params->clamp_up_mV_per_sample = clamp_up_mV;
    params->clamp_dn_mV_per_sample = clamp_dn_mV;
    
    params->mv_min_plausible = 0.0f;      // Minimum plausible voltage
    params->mv_max_plausible = 200.0f;    // Maximum plausible voltage
    
    ESP_LOGD(TAG, "Filter params: fs=%.1fHz, tau=%.1fs, alpha=%.4f, k=%.1f, clamp_up=%.3f, clamp_dn=%.3f",
             params->fs_hz, params->tau_sec, params->alpha, params->k_mV_per_bar,
             params->clamp_up_mV_per_sample, params->clamp_dn_mV_per_sample);
}

/**
 * @brief Reset robust filter state
 */
static void robust_filter_reset_state(robust_filter_state_t *state)
{
    if (!state) return;
    
    // Clear median buffer
    for (int i = 0; i < FILTER_MEDIAN_WINDOW_SIZE; i++) {
        state->med_buf[i] = 0.0f;
    }
    state->med_count = 0;
    state->med_index = 0;
    
    // Reset filter states
    state->y_ema_mV = 0.0f;
    state->y_lim_mV = 0.0f;
    
    state->initialized = false;
    
    ESP_LOGD(TAG, "Robust filter state reset");
}

/**
 * @brief Compute median of up to 5 values (deterministic, no heap)
 */
static float robust_filter_median_of_5(float *buffer, uint8_t count)
{
    if (count == 0) return 0.0f;
    if (count == 1) return buffer[0];
    
    // Copy to local array for sorting
    float vals[5];
    for (uint8_t i = 0; i < count && i < 5; i++) {
        vals[i] = buffer[i];
    }
    
    // Simple insertion sort for small array
    for (uint8_t i = 1; i < count; i++) {
        float key = vals[i];
        int j = i - 1;
        while (j >= 0 && vals[j] > key) {
            vals[j + 1] = vals[j];
            j--;
        }
        vals[j + 1] = key;
    }
    
    // Return median
    return vals[count / 2];
}

/**
 * @brief Robust smoothing step: median-of-5 → EMA → slew-rate limiter
 */
static smooth_step_output_t robust_smooth_step(robust_filter_params_t *params, 
                                              robust_filter_state_t *state, 
                                              float raw_mV)
{
    smooth_step_output_t output = {0};
    output.raw_mV = raw_mV;
    
    if (!params || !state) {
        output.med_mV = raw_mV;
        output.y_ema_mV = raw_mV;
        output.y_lim_mV = raw_mV;
        return output;
    }
    
    // 1. Plausibility clamp (optional)
    float clamped_mV = raw_mV;
    if (clamped_mV < params->mv_min_plausible) clamped_mV = params->mv_min_plausible;
    if (clamped_mV > params->mv_max_plausible) clamped_mV = params->mv_max_plausible;
    
    // 2. Median-of-5 outlier removal
    state->med_buf[state->med_index] = clamped_mV;
    state->med_index = (state->med_index + 1) % FILTER_MEDIAN_WINDOW_SIZE;
    if (state->med_count < FILTER_MEDIAN_WINDOW_SIZE) {
        state->med_count++;
    }
    
    float med_mV = robust_filter_median_of_5(state->med_buf, state->med_count);
    output.med_mV = med_mV;
    
    // 3. Initialize (first few samples)
    if (!state->initialized) {
        state->y_ema_mV = med_mV;
        state->y_lim_mV = med_mV;
        state->initialized = true;
        output.y_ema_mV = med_mV;
        output.y_lim_mV = med_mV;
        return output;
    }
    
    // 4. EMA low-pass filter
    float y_ema = state->y_ema_mV + params->alpha * (med_mV - state->y_ema_mV);
    state->y_ema_mV = y_ema;
    output.y_ema_mV = y_ema;
    
    // 5. Slew-rate limiter (apply per-sample mV clamps)
    float up_max = state->y_lim_mV + params->clamp_up_mV_per_sample;
    float down_min = state->y_lim_mV - params->clamp_dn_mV_per_sample;
    
    float y_lim = y_ema;
    if (y_lim > up_max) y_lim = up_max;
    if (y_lim < down_min) y_lim = down_min;
    
    state->y_lim_mV = y_lim;
    output.y_lim_mV = y_lim;
    
    return output;
}

/**
 * @brief Update filter calibration sensitivity when sensor calibration changes
 */
static void robust_filter_update_calibration_sensitivity(void)
{
    float k1_mV_per_bar = s_sensor1_params.k_mV_per_bar;  // Keep current if update fails
    float k2_mV_per_bar = s_sensor2_params.k_mV_per_bar;  // Keep current if update fails
    bool s1_updated = false;
    bool s2_updated = false;
    
    // Try to get updated sensitivity from current calibration system
    multi_point_calibration_t current_cal;
    
    // Update sensor 1 sensitivity
    esp_err_t s1_ret = sensor_calibration_get_current(0, &current_cal);
    if (s1_ret == ESP_OK && current_cal.valid) {
        // Validate sensitivity is reasonable (20-150 mV/bar range)
        if (current_cal.sensitivity_mv_per_bar >= 20.0f && current_cal.sensitivity_mv_per_bar <= 150.0f) {
            float new_k1 = (float)current_cal.sensitivity_mv_per_bar;
            if (fabsf(new_k1 - k1_mV_per_bar) > 0.1f) {  // Only update if changed significantly
                k1_mV_per_bar = new_k1;
                s1_updated = true;
            }
        } else {
            ESP_LOGW(TAG, "S1 calibration update: sensitivity out of range %.1f mV/bar, keeping current", 
                     (float)current_cal.sensitivity_mv_per_bar);
        }
    }
    
    // Update sensor 2 sensitivity
    esp_err_t s2_ret = sensor_calibration_get_current(1, &current_cal);
    if (s2_ret == ESP_OK && current_cal.valid) {
        // Validate sensitivity is reasonable (20-150 mV/bar range)
        if (current_cal.sensitivity_mv_per_bar >= 20.0f && current_cal.sensitivity_mv_per_bar <= 150.0f) {
            float new_k2 = (float)current_cal.sensitivity_mv_per_bar;
            if (fabsf(new_k2 - k2_mV_per_bar) > 0.1f) {  // Only update if changed significantly
                k2_mV_per_bar = new_k2;
                s2_updated = true;
            }
        } else {
            ESP_LOGW(TAG, "S2 calibration update: sensitivity out of range %.1f mV/bar, keeping current", 
                     (float)current_cal.sensitivity_mv_per_bar);
        }
    }
    
    // Only update if at least one sensor calibration changed
    if (s1_updated || s2_updated) {
        // Recompute clamps with updated sensitivity
        float clamp_up_1_mV, clamp_dn_1_mV, clamp_up_2_mV, clamp_dn_2_mV;
        robust_filter_compute_clamps_for_two_sensors(FILTER_SAMPLE_RATE_HZ,
                                                     k1_mV_per_bar, k2_mV_per_bar,
                                                     FILTER_DEFAULT_S_UP_BAR_S, FILTER_DEFAULT_S_DOWN_BAR_S,
                                                     &clamp_up_1_mV, &clamp_dn_1_mV,
                                                     &clamp_up_2_mV, &clamp_dn_2_mV);
        
        // Update existing filter parameters with new sensitivity and clamps
        s_sensor1_params.k_mV_per_bar = k1_mV_per_bar;
        s_sensor1_params.clamp_up_mV_per_sample = clamp_up_1_mV;
        s_sensor1_params.clamp_dn_mV_per_sample = clamp_dn_1_mV;
        
        s_sensor2_params.k_mV_per_bar = k2_mV_per_bar;
        s_sensor2_params.clamp_up_mV_per_sample = clamp_up_2_mV;
        s_sensor2_params.clamp_dn_mV_per_sample = clamp_dn_2_mV;
        
        ESP_LOGI(TAG, "Filter calibration updated: S1=%s%.1f, S2=%s%.1f mV/bar", 
                 s1_updated ? "NEW " : "", k1_mV_per_bar,
                 s2_updated ? "NEW " : "", k2_mV_per_bar);
    } else {
        ESP_LOGD(TAG, "Filter calibration unchanged: S1=%.1f, S2=%.1f mV/bar", k1_mV_per_bar, k2_mV_per_bar);
    }
}

// Helper function to check data staleness
static bool is_data_stale(uint32_t current_time)
{
    return (current_time - s_current_data.timestamp_ms) > MAX_DATA_AGE_MS;
}

// Recovery functions
static esp_err_t attempt_sensor_recovery(void);
static esp_err_t reinitialize_sensors(void);
static void trigger_system_reset(void);

// Helper function to set sensor failure state
static void set_sensor_failure(sensor_failure_t failure_type, const char* reason)
{
    s_current_data.valid = false;
    s_current_data.failure_type = failure_type;
    s_current_data.consecutive_failures++;
    s_current_data.o2_calculated_ppo2 = FAIL_SAFE_FO2 * s_current_data.pressure_bar;
    
    ESP_LOGE(TAG, "SENSOR FAILURE: %s (Type: %d, Consecutive: %lu)", 
             reason, failure_type, s_current_data.consecutive_failures);
    
    // Progressive recovery based on failure count
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Start recovery attempts after threshold
    if (s_current_data.consecutive_failures >= RECOVERY_ATTEMPT_THRESHOLD) {
        // Only attempt recovery every 10 seconds to avoid spam
        if (current_time - s_last_recovery_attempt_time > 10000) {
            ESP_LOGW(TAG, "RECOVERY: Attempting sensor recovery (failures: %lu)", s_current_data.consecutive_failures);
            s_system_in_recovery = true;
            s_last_recovery_attempt_time = current_time;
            
            if (attempt_sensor_recovery() == ESP_OK) {
                ESP_LOGI(TAG, "RECOVERY: Sensor recovery successful");
                s_system_in_recovery = false;
            }
        }
    }
    
    // Sensor re-initialization threshold
    if (s_current_data.consecutive_failures >= SENSOR_REINIT_THRESHOLD) {
        ESP_LOGE(TAG, "RECOVERY: Attempting sensor re-initialization (failures: %lu)", s_current_data.consecutive_failures);
        if (reinitialize_sensors() == ESP_OK) {
            ESP_LOGI(TAG, "RECOVERY: Sensor re-initialization successful");
            s_sensor_reinit_count++;
            s_current_data.consecutive_failures = 0; // Reset failure count after reinit
        }
    }
    
    // System reset threshold - last resort
    if (s_current_data.consecutive_failures >= SYSTEM_RESET_THRESHOLD) {
        ESP_LOGE(TAG, "CRITICAL: System reset required - too many consecutive failures!");
        trigger_system_reset(); // This will restart the system
    }
    
    // Critical failure threshold reached
    if (s_current_data.consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
        ESP_LOGE(TAG, "CRITICAL: Maximum consecutive failures reached! System may be unsafe!");
    }
}

esp_err_t sensor_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing sensor manager with internal ADC");

    if (s_initialized) {
        ESP_LOGW(TAG, "Sensor manager already initialized");
        return ESP_OK;
    }
    
    // Initialize advanced calibration system first
    esp_err_t cal_ret = sensor_calibration_init();
    if (cal_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize calibration system: %s", esp_err_to_name(cal_ret));
        return cal_ret;
    }

    // Initialize sensor data structure with fail-safe defaults for dual sensors
    s_current_data.o2_sensor1_reading_mv = 0.0f;
    s_current_data.o2_sensor2_reading_mv = 0.0f;
    s_current_data.o2_sensor1_ppo2 = FAIL_SAFE_FO2 * 1.013f;   // Conservative fail-safe PPO2
    s_current_data.o2_sensor2_ppo2 = FAIL_SAFE_FO2 * 1.013f;   // Conservative fail-safe PPO2
    s_current_data.o2_calculated_ppo2 = FAIL_SAFE_FO2 * 1.013f; // Conservative fail-safe PPO2
    s_current_data.pressure_bar = 1.013f;                      // Fixed atmospheric pressure
    s_current_data.sensor1_valid = false;                      // Invalid until first successful read
    s_current_data.sensor2_valid = false;                      // Invalid until first successful read
    s_current_data.battery_voltage_v = 3.3f;                   // Assume full battery initially
    s_current_data.battery_percentage = 100;                   // Assume full charge initially
    s_current_data.battery_low = false;                        // Not low initially
    s_current_data.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_current_data.valid = false;  // Invalid until first successful read
    s_current_data.failure_type = SENSOR_FAIL_NONE;
    s_current_data.consecutive_failures = 0;

    s_read_count = 0;
    
    // Initialize internal ADC1 for dual sensor reading
    ESP_LOGI(TAG, "Initializing internal ADC1 for dual O2 sensors");
    
    // Initialize ADC1 unit
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t adc_ret = adc_oneshot_new_unit(&init_config1, &s_adc1_handle);
    if (adc_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC1 unit: %s", esp_err_to_name(adc_ret));
        return adc_ret;
    }
    
    // Configure ADC1 channels
    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    
    // Configure sensor 1 channel
    adc_ret = adc_oneshot_config_channel(s_adc1_handle, SENSOR1_ADC_CHANNEL, &config);
    if (adc_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC1 sensor 1 channel: %s", esp_err_to_name(adc_ret));
        return adc_ret;
    }
    
    // Configure sensor 2 channel
    adc_ret = adc_oneshot_config_channel(s_adc1_handle, SENSOR2_ADC_CHANNEL, &config);
    if (adc_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC1 sensor 2 channel: %s", esp_err_to_name(adc_ret));
        return adc_ret;
    }
    
    // Configure battery channel
    adc_ret = adc_oneshot_config_channel(s_adc1_handle, BATTERY_ADC_CHANNEL, &config);
    if (adc_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC1 battery channel: %s", esp_err_to_name(adc_ret));
        return adc_ret;
    }
    
    // Initialize ADC calibration for both sensors and battery
    s_adc_calibrated_sensor1 = adc_calibration_init(ADC_UNIT_1, SENSOR1_ADC_CHANNEL, EXAMPLE_ADC_ATTEN, &s_adc1_cali_sensor1_handle);
    s_adc_calibrated_sensor2 = adc_calibration_init(ADC_UNIT_1, SENSOR2_ADC_CHANNEL, EXAMPLE_ADC_ATTEN, &s_adc1_cali_sensor2_handle);
    s_adc_calibrated_battery = adc_calibration_init(ADC_UNIT_1, BATTERY_ADC_CHANNEL, EXAMPLE_ADC_ATTEN, &s_adc1_cali_battery_handle);
    
    ESP_LOGI(TAG, "ADC calibration status: S1=%s, S2=%s, BAT=%s", 
             s_adc_calibrated_sensor1 ? "OK" : "FAILED", 
             s_adc_calibrated_sensor2 ? "OK" : "FAILED",
             s_adc_calibrated_battery ? "OK" : "FAILED");
    
    // Initialize robust filters with calibration data if available
    float k1_mV_per_bar = 60.0f;  // Default O2 sensor sensitivity
    float k2_mV_per_bar = 60.0f;  // Default O2 sensor sensitivity
    bool s1_cal_loaded = false;
    bool s2_cal_loaded = false;
    
    // Try to get actual sensitivity from current calibration system
    multi_point_calibration_t current_cal;
    
    // Load sensor 1 calibration
    esp_err_t s1_cal_ret = sensor_calibration_get_current(0, &current_cal);
    if (s1_cal_ret == ESP_OK && current_cal.valid) {
        // Validate sensitivity is reasonable (20-150 mV/bar range)
        if (current_cal.sensitivity_mv_per_bar >= 20.0f && current_cal.sensitivity_mv_per_bar <= 150.0f) {
            k1_mV_per_bar = (float)current_cal.sensitivity_mv_per_bar;
            s1_cal_loaded = true;
            ESP_LOGI(TAG, "S1 calibration loaded: %.1f mV/bar (R²=%.4f, points=%d)", 
                     k1_mV_per_bar, current_cal.correlation_r2, current_cal.num_points);
        } else {
            ESP_LOGW(TAG, "S1 calibration sensitivity out of range: %.1f mV/bar, using default", 
                     (float)current_cal.sensitivity_mv_per_bar);
        }
    } else {
        ESP_LOGI(TAG, "S1 calibration not available (error: %s), using default %.1f mV/bar", 
                 esp_err_to_name(s1_cal_ret), k1_mV_per_bar);
    }
    
    // Load sensor 2 calibration  
    esp_err_t s2_cal_ret = sensor_calibration_get_current(1, &current_cal);
    if (s2_cal_ret == ESP_OK && current_cal.valid) {
        // Validate sensitivity is reasonable (20-150 mV/bar range)
        if (current_cal.sensitivity_mv_per_bar >= 20.0f && current_cal.sensitivity_mv_per_bar <= 150.0f) {
            k2_mV_per_bar = (float)current_cal.sensitivity_mv_per_bar;
            s2_cal_loaded = true;
            ESP_LOGI(TAG, "S2 calibration loaded: %.1f mV/bar (R²=%.4f, points=%d)", 
                     k2_mV_per_bar, current_cal.correlation_r2, current_cal.num_points);
        } else {
            ESP_LOGW(TAG, "S2 calibration sensitivity out of range: %.1f mV/bar, using default", 
                     (float)current_cal.sensitivity_mv_per_bar);
        }
    } else {
        ESP_LOGI(TAG, "S2 calibration not available (error: %s), using default %.1f mV/bar", 
                 esp_err_to_name(s2_cal_ret), k2_mV_per_bar);
    }
    
    // Log calibration summary
    ESP_LOGI(TAG, "Filter calibration summary: S1=%s (%.1f mV/bar), S2=%s (%.1f mV/bar)",
             s1_cal_loaded ? "loaded" : "default", k1_mV_per_bar,
             s2_cal_loaded ? "loaded" : "default", k2_mV_per_bar);
    
    // Compute clamps for both sensors
    float clamp_up_1_mV, clamp_dn_1_mV, clamp_up_2_mV, clamp_dn_2_mV;
    robust_filter_compute_clamps_for_two_sensors(FILTER_SAMPLE_RATE_HZ,
                                                 k1_mV_per_bar, k2_mV_per_bar,
                                                 FILTER_DEFAULT_S_UP_BAR_S, FILTER_DEFAULT_S_DOWN_BAR_S,
                                                 &clamp_up_1_mV, &clamp_dn_1_mV,
                                                 &clamp_up_2_mV, &clamp_dn_2_mV);
    
    // Initialize filter parameters for both sensors
    robust_filter_make_params(&s_sensor1_params, FILTER_SAMPLE_RATE_HZ, FILTER_DEFAULT_TAU_SEC,
                             k1_mV_per_bar, clamp_up_1_mV, clamp_dn_1_mV);
    
    robust_filter_make_params(&s_sensor2_params, FILTER_SAMPLE_RATE_HZ, FILTER_DEFAULT_TAU_SEC,
                             k2_mV_per_bar, clamp_up_2_mV, clamp_dn_2_mV);
    
    // Reset filter states
    robust_filter_reset_state(&s_sensor1_state);
    robust_filter_reset_state(&s_sensor2_state);
    robust_filter_reset_state(&s_battery_state);  // Initialize battery filter
    
    ESP_LOGI(TAG, "Robust sensor filters initialized (fs=%.1fHz, tau=%.1fs, S1_k=%.1f, S2_k=%.1f mV/bar)", 
             FILTER_SAMPLE_RATE_HZ, FILTER_DEFAULT_TAU_SEC, k1_mV_per_bar, k2_mV_per_bar);
    
    s_initialized = true;
    ESP_LOGI(TAG, "Internal ADC1 initialized successfully");

    // Perform initial sensor read
    esp_err_t ret = sensor_manager_update();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Initial sensor read failed: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Sensor manager initialized");
    return ESP_OK;
}

esp_err_t sensor_manager_read(sensor_data_t *data)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Sensor manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Check for stale data
    if (s_current_data.valid && is_data_stale(current_time)) {
        ESP_LOGW(TAG, "Sensor data is stale (age: %lums)", current_time - s_current_data.timestamp_ms);
        // Create a copy and mark as stale
        *data = s_current_data;
        data->valid = false;
        data->failure_type = SENSOR_FAIL_DATA_STALE;
        return ESP_OK;  // Return data but marked as invalid
    }

    // Copy current sensor data
    *data = s_current_data;
    
    ESP_LOGD(TAG, "Sensor data read: O2#1=%.1fmV (PPO2=%.2f), O2#2=%.1fmV (PPO2=%.2f), Valid=%d/%d", 
             data->o2_sensor1_reading_mv, data->o2_sensor1_ppo2, data->o2_sensor2_reading_mv, data->o2_sensor2_ppo2, 
             data->sensor1_valid, data->sensor2_valid);

    return ESP_OK;
}

esp_err_t sensor_manager_update(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    s_read_count++;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

#ifdef SENSOR_FAILURE_TEST
    // Simulate stale data by skipping updates
    if (TEST_STALE_DATA && (s_test_counter % 20 < 10)) {
        ESP_LOGD(TAG, "TEST: Simulating stale data (skipping update)");
        return ESP_OK;  // Skip update to make data stale
    }
#endif

    // Read dual O2 sensor voltages from internal ADC1
    float raw_o2_sensor1_mv = 0.0f, raw_o2_sensor2_mv = 0.0f;
    bool sensor1_ok = false;
    bool sensor2_ok = false;
    
#ifdef SENSOR_READING_TEST
#ifdef WARNING_TEST
    // WARNING TEST MODE: Generate specific values to test warning system
    static uint32_t warning_test_cycle = 0;
    warning_test_cycle++;
    
    // Test cycle: 20 updates per scenario (1 second at 20Hz)
    uint32_t scenario = (warning_test_cycle / 20) % 6;
    
    switch (scenario) {
        case 0: // Normal operation
            raw_o2_sensor1_mv = 8.5f;   // Should give ~0.21 PPO2
            raw_o2_sensor2_mv = 8.7f;   // Should give ~0.21 PPO2
            break;
        case 1: // High PPO2 warning on sensor 1
            raw_o2_sensor1_mv = 58.0f;  // Should give ~1.44 PPO2 (warning)
            raw_o2_sensor2_mv = 8.7f;   // Normal
            break;
        case 2: // High PPO2 alarm on sensor 2
            raw_o2_sensor1_mv = 8.5f;   // Normal
            raw_o2_sensor2_mv = 66.0f;  // Should give ~1.67 PPO2 (alarm)
            break;
        case 3: // Low PPO2 warning on sensor 1
            raw_o2_sensor1_mv = 7.5f;   // Should give ~0.18 PPO2 (warning)
            raw_o2_sensor2_mv = 8.7f;   // Normal
            break;
        case 4: // Sensor disagreement (both normal individually, but differ > 0.05)
            raw_o2_sensor1_mv = 8.5f;   // Should give ~0.21 PPO2
            raw_o2_sensor2_mv = 10.5f;  // Should give ~0.26 PPO2 (diff = 0.05+)
            break;
        case 5: // Single sensor failure
            raw_o2_sensor1_mv = 8.5f;   // Normal
            raw_o2_sensor2_mv = 8.7f;   // Normal but we'll mark as failed
            sensor2_ok = false;         // Simulate sensor 2 failure
            break;
    }
    sensor1_ok = true;
    if (scenario != 5) sensor2_ok = true;
    
    ESP_LOGI(TAG, "WARNING TEST: Scenario %lu - S1: %.1fmV(%s), S2: %.1fmV(%s)", 
             scenario, raw_o2_sensor1_mv, sensor1_ok ? "OK" : "FAIL", 
             raw_o2_sensor2_mv, sensor2_ok ? "OK" : "FAIL");
             
#else
    // Normal test stub: Simulate sensor readings cycling from 0.5mV to 50mV and back
    // Sensor 1: increment by 5mV each cycle
    if (s_test_sensor1_ascending) {
        s_test_sensor1_mv += 5.0f;
        if (s_test_sensor1_mv >= 50.0f) {
            s_test_sensor1_mv = 50.0f;
            s_test_sensor1_ascending = false;
        }
    } else {
        s_test_sensor1_mv -= 5.0f;
        if (s_test_sensor1_mv <= 0.5f) {
            s_test_sensor1_mv = 0.5f;
            s_test_sensor1_ascending = true;
        }
    }
    
    // Sensor 2: increment by 5mV each cycle (offset by +2.5mV from sensor 1)
    if (s_test_sensor2_ascending) {
        s_test_sensor2_mv += 5.0f;
        if (s_test_sensor2_mv >= 50.0f) {
            s_test_sensor2_mv = 50.0f;
            s_test_sensor2_ascending = false;
        }
    } else {
        s_test_sensor2_mv -= 5.0f;
        if (s_test_sensor2_mv <= 0.5f) {
            s_test_sensor2_mv = 0.5f;
            s_test_sensor2_ascending = true;
        }
    }
    
    raw_o2_sensor1_mv = s_test_sensor1_mv;
    raw_o2_sensor2_mv = s_test_sensor2_mv + 2.5f;  // Slight offset for differentiation
    sensor1_ok = true;
    sensor2_ok = true;
    
    ESP_LOGI(TAG, "TEST STUB: Sensor readings - S1: %.1fmV, S2: %.1fmV", raw_o2_sensor1_mv, raw_o2_sensor2_mv);
#endif
    
#else
    // Real ADC reading code
    int raw_sensor1_adc, raw_sensor2_adc, raw_battery_adc;
    
    // Read ADC values using oneshot API
    adc_oneshot_read(s_adc1_handle, SENSOR2_ADC_CHANNEL, &raw_sensor2_adc);
    adc_oneshot_read(s_adc1_handle, SENSOR2_ADC_CHANNEL, &raw_sensor2_adc);
    adc_oneshot_read(s_adc1_handle, SENSOR2_ADC_CHANNEL, &raw_sensor2_adc);
    esp_err_t sensor2_ret = adc_oneshot_read(s_adc1_handle, SENSOR2_ADC_CHANNEL, &raw_sensor2_adc);
    adc_oneshot_read(s_adc1_handle, BATTERY_ADC_CHANNEL, &raw_battery_adc);
    adc_oneshot_read(s_adc1_handle, BATTERY_ADC_CHANNEL, &raw_battery_adc);
    adc_oneshot_read(s_adc1_handle, BATTERY_ADC_CHANNEL, &raw_battery_adc);
    esp_err_t battery_ret = adc_oneshot_read(s_adc1_handle, BATTERY_ADC_CHANNEL, &raw_battery_adc);
    adc_oneshot_read(s_adc1_handle, SENSOR1_ADC_CHANNEL, &raw_sensor1_adc);
    adc_oneshot_read(s_adc1_handle, SENSOR1_ADC_CHANNEL, &raw_sensor1_adc);
    adc_oneshot_read(s_adc1_handle, SENSOR1_ADC_CHANNEL, &raw_sensor1_adc);
    esp_err_t sensor1_ret = adc_oneshot_read(s_adc1_handle, SENSOR1_ADC_CHANNEL, &raw_sensor1_adc);

    ESP_LOGD(TAG, "ADC raw readings: S1=%d (ret=%s), S2=%d (ret=%s), BAT=%d (ret=%s)", 
             raw_sensor1_adc, esp_err_to_name(sensor1_ret),
             raw_sensor2_adc, esp_err_to_name(sensor2_ret),
             raw_battery_adc, esp_err_to_name(battery_ret));
    
    // Process sensor 1 reading
    if (sensor1_ret == ESP_OK) {
        if (s_adc_calibrated_sensor1) {
            int voltage_mv;
            esp_err_t cali_ret = adc_cali_raw_to_voltage(s_adc1_cali_sensor1_handle, raw_sensor1_adc, &voltage_mv);
            if (cali_ret == ESP_OK) {
                raw_o2_sensor1_mv = (float)voltage_mv;
                sensor1_ok = true;
                ESP_LOGD(TAG, "S1 calibrated: raw=%d -> %dmV", raw_sensor1_adc, voltage_mv);
            } else {
                ESP_LOGW(TAG, "S1 calibration failed: %s", esp_err_to_name(cali_ret));
            }
        } else {
            // Fallback: approximate conversion without calibration
            // For 12-bit ADC with 3.3V ref and DB_12 attenuation (0-3.1V range)
            raw_o2_sensor1_mv = (raw_sensor1_adc * ADC_REF_VOLTAGE_MV) / ADC_MAX_VALUE;  // 12-bit ADC: 0-4095 range, ~3.1V max with DB_12
            sensor1_ok = true;
            ESP_LOGD(TAG, "S1 uncalibrated: raw=%d -> %.1fmV", raw_sensor1_adc, raw_o2_sensor1_mv);
        }
    }
    
    // Process sensor 2 reading
    if (sensor2_ret == ESP_OK) {
        if (s_adc_calibrated_sensor2) {
            int voltage_mv;
            esp_err_t cali_ret = adc_cali_raw_to_voltage(s_adc1_cali_sensor2_handle, raw_sensor2_adc, &voltage_mv);
            if (cali_ret == ESP_OK) {
                raw_o2_sensor2_mv = (float)voltage_mv;
                sensor2_ok = true;
                ESP_LOGD(TAG, "S2 calibrated: raw=%d -> %dmV", raw_sensor2_adc, voltage_mv);
            } else {
                ESP_LOGW(TAG, "S2 calibration failed: %s", esp_err_to_name(cali_ret));
            }
        } else {
            // Fallback: approximate conversion without calibration
            // For 12-bit ADC with 3.3V ref and DB_12 attenuation (0-3.1V range)
            raw_o2_sensor2_mv = (raw_sensor2_adc * ADC_REF_VOLTAGE_MV) / ADC_MAX_VALUE;  // 12-bit ADC: 0-4095 range, ~3.1V max with DB_12
            sensor2_ok = true;
            ESP_LOGD(TAG, "S2 uncalibrated: raw=%d -> %.1fmV", raw_sensor2_adc, raw_o2_sensor2_mv);
        }
    }
#endif
    
    // Apply emulation mode voltage scaling if enabled
    if (sensor1_ok) {
        raw_o2_sensor1_mv /= SENSOR_VOLTAGE_SCALE_FACTOR;
    }
    if (sensor2_ok) {
        raw_o2_sensor2_mv /= SENSOR_VOLTAGE_SCALE_FACTOR;
    }
    
    // Process battery voltage reading
    float raw_battery_mv = 0.0f;
    bool battery_ok = false;
    
#ifndef SENSOR_FAILURE_TEST
    if (battery_ret == ESP_OK) {
        if (s_adc_calibrated_battery) {
            int voltage_mv;
            esp_err_t cali_ret = adc_cali_raw_to_voltage(s_adc1_cali_battery_handle, raw_battery_adc, &voltage_mv);
            if (cali_ret == ESP_OK) {
                raw_battery_mv = (float)voltage_mv;
                battery_ok = true;
                ESP_LOGD(TAG, "Battery calibrated: raw=%d -> %.dmV", raw_battery_adc, voltage_mv);
            } else {
                ESP_LOGW(TAG, "Battery calibration failed: %s", esp_err_to_name(cali_ret));
            }
        } else {
            // Fallback: approximate conversion without calibration
            raw_battery_mv = (raw_battery_adc * ADC_REF_VOLTAGE_MV) / ADC_MAX_VALUE;
            battery_ok = true;
            ESP_LOGD(TAG, "Battery uncalibrated: raw=%d -> %.1fmV", raw_battery_adc, raw_battery_mv);
        }
    }
#endif
    
#ifdef SENSOR_EMULATION_MODE
   /* if (sensor1_ok || sensor2_ok) {
        ESP_LOGI(TAG, "EMULATION MODE: Scaled readings - S1: %.1fmV, S2: %.1fmV (scale factor: %.1f)", 
                 sensor1_ok ? raw_o2_sensor1_mv : 0.0f, 
                 sensor2_ok ? raw_o2_sensor2_mv : 0.0f, 
                 SENSOR_VOLTAGE_SCALE_FACTOR);
    }  */
#endif
    
    // Apply robust filtering pipeline: median-of-5 → EMA → slew-rate limiter → alarm logic
    smooth_step_output_t sensor1_output = {0};
    smooth_step_output_t sensor2_output = {0};
    
    float filtered_o2_sensor1_mv = raw_o2_sensor1_mv;
    float filtered_o2_sensor2_mv = raw_o2_sensor2_mv;
    
    if (sensor1_ok) {
        sensor1_output = robust_smooth_step(&s_sensor1_params, &s_sensor1_state, raw_o2_sensor1_mv);
        filtered_o2_sensor1_mv = sensor1_output.y_lim_mV;
        ESP_LOGV(TAG, "S1 robust filter: raw=%.3f -> med=%.3f -> ema=%.3f -> lim=%.3f", 
                 sensor1_output.raw_mV, sensor1_output.med_mV, sensor1_output.y_ema_mV, 
                 sensor1_output.y_lim_mV);
    }
    
    if (sensor2_ok) {
        sensor2_output = robust_smooth_step(&s_sensor2_params, &s_sensor2_state, raw_o2_sensor2_mv);
        filtered_o2_sensor2_mv = sensor2_output.y_lim_mV;
        ESP_LOGV(TAG, "S2 robust filter: raw=%.3f -> med=%.3f -> ema=%.3f -> lim=%.3f", 
                 sensor2_output.raw_mV, sensor2_output.med_mV, sensor2_output.y_ema_mV, 
                 sensor2_output.y_lim_mV);
    }
    
    // Simple EMA filtering for battery voltage (slow changes expected)
    float filtered_battery_mv = raw_battery_mv;
    if (battery_ok) {
        float battery_alpha = 1.0f / (BATTERY_TAU_SEC * FILTER_SAMPLE_RATE_HZ + 1.0f);  // EMA coefficient for battery
        if (s_battery_state.initialized) {
            s_battery_state.y_ema_mV = (1.0f - battery_alpha) * s_battery_state.y_ema_mV + battery_alpha * raw_battery_mv;
        } else {
            s_battery_state.y_ema_mV = raw_battery_mv;
            s_battery_state.initialized = true;
        }
        filtered_battery_mv = s_battery_state.y_ema_mV;
        ESP_LOGV(TAG, "Battery filter: raw=%.1fmV -> ema=%.1fmV", raw_battery_mv, filtered_battery_mv);
    }
    
    // Check if both sensors failed
    if (!sensor1_ok && !sensor2_ok) {
        s_consecutive_o2_failures++;
        set_sensor_failure(SENSOR_FAIL_O2_COMMUNICATION, "Failed to read from both internal ADC channels");
        s_current_data.timestamp_ms = current_time;
        return ESP_OK;  // Return success but with failure state
    } else {
        s_consecutive_o2_failures = 0;  // Reset failure counter if at least one sensor works
    }

    // No pressure sensor used - using fixed atmospheric pressure for surface operations
    
    // Set fixed atmospheric pressure (no pressure sensor used for PPO2 calculation)
    s_current_data.pressure_bar = 1.013f;  // Fixed atmospheric pressure
    
    // Store filtered sensor readings (these become the "calculated" readings)
    s_current_data.o2_sensor1_reading_mv = sensor1_ok ? filtered_o2_sensor1_mv : 0.0f;
    s_current_data.o2_sensor2_reading_mv = sensor2_ok ? filtered_o2_sensor2_mv : 0.0f;
    
    // Process battery voltage with voltage divider compensation
    if (battery_ok) {
        // Convert ADC voltage to actual battery voltage (voltage divider: 1/3 ratio)
        float actual_battery_v = (filtered_battery_mv / 1000.0f) * BATTERY_VOLTAGE_DIVIDER_RATIO;
        s_current_data.battery_voltage_v = actual_battery_v;
        
        // Calculate battery percentage based on voltage levels
        if (actual_battery_v >= BATTERY_FULL_VOLTAGE_V) {
            s_current_data.battery_percentage = 100;
        } else if (actual_battery_v <= BATTERY_LOW_VOLTAGE_V) {
            s_current_data.battery_percentage = 0;
        } else {
            // Linear interpolation between low and full
            float voltage_range = BATTERY_FULL_VOLTAGE_V - BATTERY_LOW_VOLTAGE_V;
            float voltage_above_low = actual_battery_v - BATTERY_LOW_VOLTAGE_V;
            s_current_data.battery_percentage = (uint8_t)((voltage_above_low / voltage_range) * 100.0f);
        }
        
        // Set low battery flag
        s_current_data.battery_low = (actual_battery_v < BATTERY_LOW_VOLTAGE_V + 0.1f);  // 0.1V hysteresis
        
        ESP_LOGV(TAG, "Battery: %.1fmV ADC -> %.2fV actual -> %d%% (%s)", 
                 filtered_battery_mv, actual_battery_v, s_current_data.battery_percentage,
                 s_current_data.battery_low ? "LOW" : "OK");
    } else {
        // Battery reading failed - keep previous values or use safe defaults
        ESP_LOGV(TAG, "Battery reading failed, keeping previous values");
    }
    
    // Configuration no longer needed - using multipoint calibration system
    
    // Process dual O2 sensors independently
    float sensor_min = SENSOR_VOLTAGE_MIN_MV;   // Minimum reasonable sensor voltage (0V = 0 PPO2)
    float sensor_max = SENSOR_VOLTAGE_MAX_MV;  // Maximum reasonable sensor voltage
    
    // Process sensor #1 using multipoint calibration system only
    s_current_data.sensor1_valid = false;
    if (sensor1_ok) {
        if (filtered_o2_sensor1_mv >= sensor_min && filtered_o2_sensor1_mv <= sensor_max) {
            // Use multipoint calibration system with filtered reading
            esp_err_t cal_ret = sensor_calibration_voltage_to_ppo2(0, filtered_o2_sensor1_mv, &s_current_data.o2_sensor1_ppo2);
            if (cal_ret == ESP_OK) {
                s_current_data.sensor1_valid = true;
                ESP_LOGV(TAG, "Sensor #1: %.1fmV -> PPO2 %.3f (multipoint cal, filtered)", 
                         filtered_o2_sensor1_mv, s_current_data.o2_sensor1_ppo2);
            } else {
                ESP_LOGW(TAG, "Sensor #1: No valid calibration in multipoint system (error: %s)", esp_err_to_name(cal_ret));
            }
        } else {
            ESP_LOGW(TAG, "Sensor #1 filtered reading %.1fmV outside valid range [%.1f, %.1f]", 
                     filtered_o2_sensor1_mv, sensor_min, sensor_max);
        }
    }
    
    // Process sensor #2 using multipoint calibration system only
    s_current_data.sensor2_valid = false;
    if (sensor2_ok) {
        if (filtered_o2_sensor2_mv >= sensor_min && filtered_o2_sensor2_mv <= sensor_max) {
            // Use multipoint calibration system with filtered reading
            esp_err_t cal_ret = sensor_calibration_voltage_to_ppo2(1, filtered_o2_sensor2_mv, &s_current_data.o2_sensor2_ppo2);
            if (cal_ret == ESP_OK) {
                s_current_data.sensor2_valid = true;
                ESP_LOGV(TAG, "Sensor #2: %.1fmV -> PPO2 %.3f (multipoint cal, filtered)", 
                         filtered_o2_sensor2_mv, s_current_data.o2_sensor2_ppo2);
            } else {
                ESP_LOGW(TAG, "Sensor #2: No valid calibration in multipoint system (error: %s)", esp_err_to_name(cal_ret));
            }
        } else {
            ESP_LOGW(TAG, "Sensor #2 filtered reading %.1fmV outside valid range [%.1f, %.1f]", 
                     filtered_o2_sensor2_mv, sensor_min, sensor_max);
        }
    }
    
    // Check if we have at least one valid sensor
    if (!s_current_data.sensor1_valid && !s_current_data.sensor2_valid) {
        set_sensor_failure(SENSOR_FAIL_O2_COMMUNICATION, "Both O2 sensors failed or uncalibrated");
        s_current_data.timestamp_ms = current_time;
        return ESP_OK; // Return success but with failure state
    }
    
    // Calculate average PPO2 and FO2 from available sensors
    double total_ppo2 = 0.0;
    int valid_sensor_count = 0;
    
    if (s_current_data.sensor1_valid) {
        total_ppo2 += s_current_data.o2_sensor1_ppo2;
        valid_sensor_count++;
    }
    if (s_current_data.sensor2_valid) {
        total_ppo2 += s_current_data.o2_sensor2_ppo2;
        valid_sensor_count++;
    }
    
    // Store averaged values for warnings and display
    s_current_data.o2_calculated_ppo2 = total_ppo2 / valid_sensor_count;
    
    // Fixed atmospheric pressure (no pressure sensor dependency)
    s_current_data.pressure_bar = 1.013f;
    
    // Mark data as valid and reset failure counters
    s_current_data.valid = true;
    s_current_data.failure_type = SENSOR_FAIL_NONE;
    s_current_data.consecutive_failures = 0;
    
    // Update timestamp
    s_current_data.timestamp_ms = current_time;

    ESP_LOGV(TAG, "Sensor update #%lu: O2#1=%.1fmV(%.3f), O2#2=%.1fmV(%.3f), Avg PPO2=%.3f, Valid=%d/%d", 
             s_read_count, s_current_data.o2_sensor1_reading_mv, s_current_data.o2_sensor1_ppo2,
             s_current_data.o2_sensor2_reading_mv, s_current_data.o2_sensor2_ppo2,
             s_current_data.o2_calculated_ppo2, s_current_data.sensor1_valid, s_current_data.sensor2_valid);

    return ESP_OK;
}
bool sensor_manager_is_ready(void)
{
    return s_initialized && s_current_data.valid;
}

esp_err_t sensor_manager_set_o2_calibration(const o2_calibration_t *cal_data)
{
    ESP_LOGW(TAG, "DEPRECATED: sensor_manager_set_o2_calibration() is deprecated");
    ESP_LOGW(TAG, "  Use sensor_manager_calibrate_o2() for new multipoint calibration system");
    
    if (!s_initialized) {
        ESP_LOGE(TAG, "Sensor manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (cal_data == NULL || !cal_data->calibrated) {
        ESP_LOGE(TAG, "Invalid calibration data");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Legacy calibration no longer supported - calibration_sensor_mv field removed
    // Convert legacy calibration to multipoint calibration
    // float o2_percent = cal_data->calibration_gas_o2_fraction * 100.0f;
    // esp_err_t ret = sensor_manager_calibrate_o2(o2_percent, cal_data->calibration_sensor_mv, 1);
    //
    // if (ret == ESP_OK) {
    //     ESP_LOGI(TAG, "Legacy calibration converted to multipoint system: %.1f%% O2 at %.1fmV",
    //              o2_percent, cal_data->calibration_sensor_mv);
    // }
    ESP_LOGW(TAG, "Legacy calibration format no longer supported - use modern multipoint calibration system");
    esp_err_t ret = ESP_ERR_NOT_SUPPORTED;
    
    return ret;
}

esp_err_t sensor_manager_get_o2_calibration(o2_calibration_t *cal_data)
{
    ESP_LOGW(TAG, "DEPRECATED: sensor_manager_get_o2_calibration() is deprecated");
    ESP_LOGW(TAG, "  Use sensor_manager_get_health_status() for calibration info");
    
    if (!s_initialized) {
        ESP_LOGE(TAG, "Sensor manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (cal_data == NULL) {
        ESP_LOGE(TAG, "Calibration data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Try to get current calibration status from multipoint system
    sensor_health_info_t health_info;
    esp_err_t ret = sensor_calibration_assess_health(0, &health_info);
    
    if (ret == ESP_OK && health_info.health_status != SENSOR_HEALTH_UNKNOWN) {
        // Provide approximate legacy format data (calibration_sensor_mv field removed)
        cal_data->calibrated = true;
        cal_data->calibration_gas_o2_fraction = 0.21f;  // Assume air calibration
        // cal_data->calibration_sensor_mv = 21.0f / health_info.normalized_sensitivity; // Approximate - COMMENTED OUT: field removed
        ESP_LOGW(TAG, "Legacy format approximated from multipoint calibration (voltage data no longer available)");
    } else {
        // No calibration available
        cal_data->calibrated = false;
        cal_data->calibration_gas_o2_fraction = 0.0f;
        // cal_data->calibration_sensor_mv = 0.0f; // COMMENTED OUT: field removed
    }
    
    return ESP_OK;
}

esp_err_t sensor_manager_calibrate_o2(float known_o2_percent, float current_mv, int sensor_number)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Sensor manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Validate inputs
    if (known_o2_percent < 10.0f || known_o2_percent > 100.0f) {
        ESP_LOGE(TAG, "Invalid O2 percentage: %.1f%% (must be 10-100%%)", known_o2_percent);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (current_mv < CALIBRATION_VOLTAGE_MIN_MV || current_mv > CALIBRATION_VOLTAGE_MAX_MV) {  // Reasonable voltage range for O2 sensors
        ESP_LOGE(TAG, "Invalid sensor voltage: %.1fmV", current_mv);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (sensor_number != 1 && sensor_number != 2) {
        ESP_LOGE(TAG, "Invalid sensor number: %d (must be 1 or 2)", sensor_number);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Convert to 0-based sensor ID for multipoint calibration API
    uint8_t sensor_id = (sensor_number == 1) ? 0 : 1;
    float o2_fraction = known_o2_percent / 100.0f;
    
    ESP_LOGI(TAG, "Performing multipoint calibration for sensor #%d: %.1f%% O2 (%.3f fraction) at %.1fmV",
             sensor_number, known_o2_percent, o2_fraction, current_mv);
    
    // Use multipoint calibration system with two-point calibration
    // Point 1: Current calibration gas
    calibration_point_t point1 = {
        .ppo2_bar = o2_fraction * 1.013f,  // PPO2 at atmospheric pressure
        .sensor_mv = current_mv,
        .pressure_bar = 1.013f,
        .temperature_c = 25.0f,            // Assumed room temperature
        .uptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS
    };
    
    // Point 2: Zero oxygen (theoretical point for linear calibration)
    calibration_point_t point2 = {
        .ppo2_bar = 0.0,                   // Zero PPO2
        .sensor_mv = 0.0f,                 // Assume zero voltage for zero oxygen
        .pressure_bar = 1.013f,
        .temperature_c = 25.0f,
        .uptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS
    };
    
    // Perform two-point calibration
    two_point_calibration_t calibration_result;
    esp_err_t cal_ret = sensor_calibration_two_point(sensor_id, &point1, &point2, &calibration_result);
    
    if (cal_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to perform multipoint calibration: %s", esp_err_to_name(cal_ret));
        return cal_ret;
    }
    
    if (!calibration_result.valid) {
        ESP_LOGE(TAG, "Multipoint calibration failed validation");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    ESP_LOGI(TAG, "Multipoint calibration successful for sensor #%d:", sensor_number);
    ESP_LOGI(TAG, "  Sensitivity: %.3f mV/bar", calibration_result.sensitivity_mv_per_bar);
    ESP_LOGI(TAG, "  Offset: %.3f mV", calibration_result.offset_mv);
    ESP_LOGI(TAG, "  R²: %.6f", calibration_result.correlation_r2);
    ESP_LOGI(TAG, "  Max residual: %.3f mV", calibration_result.max_residual_mv);
    
    // Save calibration to persistent storage
    esp_err_t save_ret = sensor_calibration_save();
    if (save_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save calibration to storage: %s", esp_err_to_name(save_ret));
        return save_ret;
    }
    
    // Update robust filter calibration sensitivity
    robust_filter_update_calibration_sensitivity();
    
    ESP_LOGI(TAG, "O2 calibration saved successfully using multipoint system");
    return ESP_OK;
}

esp_err_t sensor_manager_get_raw_o2(float *raw_mv, int sensor_number)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Sensor manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (raw_mv == NULL) {
        ESP_LOGE(TAG, "Raw voltage pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (sensor_number == 1) {
        *raw_mv = s_current_data.o2_sensor1_reading_mv;
    } else if (sensor_number == 2) {
        *raw_mv = s_current_data.o2_sensor2_reading_mv;
    } else {
        ESP_LOGE(TAG, "Invalid sensor number: %d (must be 1 or 2)", sensor_number);
        return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

esp_err_t sensor_manager_get_calibration_status(float *current_raw_mv, bool *is_calibrated, float *calibration_offset, int sensor_number)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Sensor manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!current_raw_mv || !is_calibrated || !calibration_offset) {
        ESP_LOGE(TAG, "Invalid parameter pointers");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (sensor_number != 1 && sensor_number != 2) {
        ESP_LOGE(TAG, "Invalid sensor number: %d (must be 1 or 2)", sensor_number);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get current raw voltage
    if (sensor_number == 1) {
        *current_raw_mv = s_current_data.o2_sensor1_reading_mv;
    } else {
        *current_raw_mv = s_current_data.o2_sensor2_reading_mv;
    }
    
    // Check calibration status from multipoint system
    uint8_t sensor_id = (sensor_number == 1) ? 0 : 1;
    sensor_health_info_t health_info;
    esp_err_t ret = sensor_calibration_assess_health(sensor_id, &health_info);
    
    if (ret == ESP_OK && health_info.health_status != SENSOR_HEALTH_UNKNOWN) {
        *is_calibrated = true;
        *calibration_offset = health_info.offset_drift_mv;  // Actual offset from multipoint system
    } else {
        *is_calibrated = false;
        *calibration_offset = 0.0f;
    }
    
    return ESP_OK;
}

esp_err_t sensor_manager_perform_o2_calibration(float known_o2_percent, int sensor_number)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Sensor manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (sensor_number != 1 && sensor_number != 2) {
        ESP_LOGE(TAG, "Invalid sensor number: %d (must be 1 or 2)", sensor_number);
        return ESP_ERR_INVALID_ARG;
    }
    
    // First update sensor to get the latest reading
    esp_err_t ret = sensor_manager_update();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update sensor for calibration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Get the current sensor reading for specified sensor
    float current_mv;
    if (sensor_number == 1) {
        current_mv = s_current_data.o2_sensor1_reading_mv;
        if (!s_current_data.sensor1_valid) {
            ESP_LOGE(TAG, "Cannot calibrate sensor #1 - sensor not responding");
            return ESP_ERR_INVALID_STATE;
        }
    } else {
        current_mv = s_current_data.o2_sensor2_reading_mv;
        if (!s_current_data.sensor2_valid) {
            ESP_LOGE(TAG, "Cannot calibrate sensor #2 - sensor not responding");
            return ESP_ERR_INVALID_STATE;
        }
    }
    
    // Use the current sensor reading for calibration
    return sensor_manager_calibrate_o2(known_o2_percent, current_mv, sensor_number);
}

esp_err_t sensor_manager_perform_dual_o2_calibration(float known_o2_percent)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Sensor manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Validate input
    if (known_o2_percent < 10.0f || known_o2_percent > 100.0f) {
        ESP_LOGE(TAG, "Invalid O2 percentage: %.1f%% (must be 10-100%%)", known_o2_percent);
        return ESP_ERR_INVALID_ARG;
    }
    
    // First update sensors to get the latest readings
    esp_err_t ret = sensor_manager_update();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update sensors for calibration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Get current sensor readings
    float sensor1_mv = s_current_data.o2_sensor1_reading_mv;
    float sensor2_mv = s_current_data.o2_sensor2_reading_mv;
    
    ESP_LOGI(TAG, "Performing dual O2 calibration with %.1f%% O2 - S1: %.1fmV, S2: %.1fmV", 
             known_o2_percent, sensor1_mv, sensor2_mv);
    
    // Check if both sensors have valid RAW readings (not calibrated readings)
    // Allow calibration if we have reasonable raw voltage readings, even if sensors are not calibrated yet
    bool sensor1_has_raw_data = (sensor1_mv > SENSOR_RAW_MIN_MV && sensor1_mv < SENSOR_RAW_MAX_MV);
    bool sensor2_has_raw_data = (sensor2_mv > SENSOR_RAW_MIN_MV && sensor2_mv < SENSOR_RAW_MAX_MV);
    
    if (!sensor1_has_raw_data || !sensor2_has_raw_data) {
        ESP_LOGE(TAG, "Cannot calibrate - sensors not providing valid raw readings (S1:%.1fmV %s, S2:%.1fmV %s)", 
                 sensor1_mv, sensor1_has_raw_data ? "OK" : "FAIL",
                 sensor2_mv, sensor2_has_raw_data ? "OK" : "FAIL");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Sensors have valid raw readings for calibration (S1:%.1fmV, S2:%.1fmV)", 
             sensor1_mv, sensor2_mv);
    
    // Calibrate sensor 1
    esp_err_t ret1 = sensor_manager_calibrate_o2(known_o2_percent, sensor1_mv, 1);
    if (ret1 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate sensor #1: %s", esp_err_to_name(ret1));
        return ret1;
    }
    
    // Calibrate sensor 2
    esp_err_t ret2 = sensor_manager_calibrate_o2(known_o2_percent, sensor2_mv, 2);
    if (ret2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate sensor #2: %s", esp_err_to_name(ret2));
        return ret2;
    }
    
    ESP_LOGI(TAG, "Dual O2 calibration completed successfully for both sensors");
    return ESP_OK;
}

void sensor_manager_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing sensor manager");
    
    if (s_initialized) {
        // Cleanup ADC calibration handles
        if (s_adc_calibrated_sensor1 && s_adc1_cali_sensor1_handle) {
            adc_calibration_deinit(s_adc1_cali_sensor1_handle);
            s_adc1_cali_sensor1_handle = NULL;
            s_adc_calibrated_sensor1 = false;
        }
        
        if (s_adc_calibrated_sensor2 && s_adc1_cali_sensor2_handle) {
            adc_calibration_deinit(s_adc1_cali_sensor2_handle);
            s_adc1_cali_sensor2_handle = NULL;
            s_adc_calibrated_sensor2 = false;
        }
        
        if (s_adc_calibrated_battery && s_adc1_cali_battery_handle) {
            adc_calibration_deinit(s_adc1_cali_battery_handle);
            s_adc1_cali_battery_handle = NULL;
            s_adc_calibrated_battery = false;
        }
        
        // Cleanup ADC unit handle
        if (s_adc1_handle) {
            adc_oneshot_del_unit(s_adc1_handle);
            s_adc1_handle = NULL;
            ESP_LOGI(TAG, "ADC1 unit deinitialized");
        }
        
        // Reset robust sensor filters
        robust_filter_reset_state(&s_sensor1_state);
        robust_filter_reset_state(&s_sensor2_state);
        
        s_current_data.valid = false;
        s_initialized = false;
    }

    ESP_LOGI(TAG, "Sensor manager deinitialized");
}

// Recovery function implementations
static esp_err_t attempt_sensor_recovery(void)
{
    ESP_LOGI(TAG, "RECOVERY: Attempting basic sensor recovery");
    
    // Clear error counters for a fresh start
    s_consecutive_o2_failures = 0;
    
    // Force a sensor read to test if recovery worked
    esp_err_t ret = sensor_manager_update();
    if (ret == ESP_OK && s_current_data.valid) {
        ESP_LOGI(TAG, "RECOVERY: Basic recovery successful - sensors responding");
        return ESP_OK;
    }
    
    ESP_LOGW(TAG, "RECOVERY: Basic recovery failed");
    return ESP_FAIL;
}

static esp_err_t reinitialize_sensors(void)
{
    ESP_LOGI(TAG, "RECOVERY: Re-initializing sensors");
    
#ifndef SENSOR_FAILURE_TEST
    // In real hardware mode, reinitialize the actual sensor drivers
    // Deinitialize current sensors
    sensor_manager_deinit();
    
    // Wait a bit for hardware to settle
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Re-initialize sensors
    esp_err_t ret = sensor_manager_init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RECOVERY: Sensor re-initialization successful");
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "RECOVERY: Sensor re-initialization failed: %s", esp_err_to_name(ret));
    return ret;
#else
    // In test mode, just reset counters and state
    ESP_LOGI(TAG, "RECOVERY: Test mode - simulating sensor re-initialization");
    s_consecutive_o2_failures = 0;
    s_current_data.consecutive_failures = 0;
    s_current_data.valid = false;
    s_system_in_recovery = false;
    
    // Simulate successful reinit in test mode
    return ESP_OK;
#endif
}

static void trigger_system_reset(void)
{
    ESP_LOGE(TAG, "CRITICAL: Triggering system reset due to persistent sensor failures");
    ESP_LOGE(TAG, "CRITICAL: Total failures: %lu, Reinit attempts: %lu", 
             s_current_data.consecutive_failures, s_sensor_reinit_count);
    
    // Log critical system state before reset
    ESP_LOGE(TAG, "CRITICAL: System state - O2 failures: %lu", 
             s_consecutive_o2_failures);
             
    // Give some time for logs to be written
    vTaskDelay(pdMS_TO_TICKS(1000));
    
#ifndef SENSOR_FAILURE_TEST
    // Trigger hardware reset
    esp_restart();
#else
    // In test mode, don't actually reset - just log
    ESP_LOGE(TAG, "CRITICAL: Test mode - system reset would occur here");
    // Reset all counters to simulate system reset
    s_consecutive_o2_failures = 0;
    s_current_data.consecutive_failures = 0;
    s_sensor_reinit_count = 0;
    s_system_in_recovery = false;
#endif
}

// ADC calibration helper functions
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "ADC calibration scheme: Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_WIDTH,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "ADC calibration scheme: Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_WIDTH,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration successful");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory for ADC calibration");
    }

    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister Curve Fitting calibration scheme");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister Line Fitting calibration scheme");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

// Advanced calibration function implementations
esp_err_t sensor_manager_advanced_calibration(uint8_t sensor_id,
                                              float gas1_o2_fraction, float gas1_pressure_bar,
                                              float gas2_o2_fraction, float gas2_pressure_bar,
                                              two_point_calibration_t *result)
{
    if (!s_initialized || sensor_id >= 2 || !result) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Advanced calibration S%u: Gas1(%.3f O2, %.2f bar), Gas2(%.3f O2, %.2f bar)", 
             sensor_id, gas1_o2_fraction, gas1_pressure_bar, gas2_o2_fraction, gas2_pressure_bar);
    
    // Update sensor to get current readings
    esp_err_t ret = sensor_manager_update();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update sensor readings");
        return ret;
    }
    
    // Get current sensor voltage
    float current_mv;
    if (sensor_id == 0) {
        current_mv = s_current_data.o2_sensor1_reading_mv;
        if (!s_current_data.sensor1_valid) {
            ESP_LOGE(TAG, "Sensor 0 not responding for calibration");
            return ESP_ERR_INVALID_STATE;
        }
    } else {
        current_mv = s_current_data.o2_sensor2_reading_mv;
        if (!s_current_data.sensor2_valid) {
            ESP_LOGE(TAG, "Sensor 1 not responding for calibration");
            return ESP_ERR_INVALID_STATE;
        }
    }
    
    // For simplicity, assume we're calibrating at the first gas point
    // In a real implementation, you'd expose the sensor to each gas and take readings
    calibration_point_t point1 = {
        .ppo2_bar = gas1_o2_fraction * gas1_pressure_bar,
        .sensor_mv = current_mv,
        .pressure_bar = gas1_pressure_bar,
        .temperature_c = 25.0f, // Assumed room temperature
        .uptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS
    };
    
    // For the second point, we'll use a theoretical calculation
    // In practice, you'd expose to the second gas and take another reading
    calibration_point_t point2 = {
        .ppo2_bar = gas2_o2_fraction * gas2_pressure_bar,
        .sensor_mv = current_mv * (gas2_o2_fraction / gas1_o2_fraction), // Linear approximation
        .pressure_bar = gas2_pressure_bar,
        .temperature_c = 25.0f,
        .uptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS
    };
    
    return sensor_calibration_two_point(sensor_id, &point1, &point2, result);
}

esp_err_t sensor_manager_start_multipoint_calibration(uint8_t sensor_id)
{
    if (!s_initialized || sensor_id >= 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return sensor_calibration_start_session(sensor_id);
}

esp_err_t sensor_manager_add_calibration_point(uint8_t sensor_id, 
                                               float o2_fraction, 
                                               float pressure_bar)
{
    if (!s_initialized || sensor_id >= 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update sensor to get current reading
    esp_err_t ret = sensor_manager_update();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update sensor for calibration point");
        return ret;
    }
    
    // Get current sensor voltage
    float current_mv;
    bool sensor_valid;
    
    if (sensor_id == 0) {
        current_mv = s_current_data.o2_sensor1_reading_mv;
        sensor_valid = s_current_data.sensor1_valid;
    } else {
        current_mv = s_current_data.o2_sensor2_reading_mv;
        sensor_valid = s_current_data.sensor2_valid;
    }
    
    if (!sensor_valid) {
        ESP_LOGE(TAG, "Sensor %u not responding for calibration point", sensor_id);
        return ESP_ERR_INVALID_STATE;
    }
    
    calibration_point_t point = {
        .ppo2_bar = o2_fraction * pressure_bar,
        .sensor_mv = current_mv,
        .pressure_bar = pressure_bar,
        .temperature_c = 25.0f, // Assumed room temperature
        .uptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS
    };
    
    ESP_LOGI(TAG, "Adding calibration point S%u: PPO2=%.3f bar, V=%.1f mV", 
             sensor_id, point.ppo2_bar, point.sensor_mv);
    
    return sensor_calibration_add_point(sensor_id, &point);
}

esp_err_t sensor_manager_finalize_multipoint_calibration(uint8_t sensor_id,
                                                         multi_point_calibration_t *result)
{
    if (!s_initialized || sensor_id >= 2 || !result) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return sensor_calibration_finalize(sensor_id, result);
}

esp_err_t sensor_manager_get_health_status(uint8_t sensor_id, sensor_health_info_t *health_info)
{
    if (!s_initialized || sensor_id >= 2 || !health_info) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return sensor_calibration_assess_health(sensor_id, health_info);
}

esp_err_t sensor_manager_initialize_sensor_baseline(uint8_t sensor_id, const char *sensor_serial)
{
    if (!s_initialized || sensor_id >= 2 || !sensor_serial) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Use typical new sensor sensitivity as baseline (e.g., 60 mV/bar)
    double baseline_sensitivity = 60.0; // mV/bar for a typical new O2 sensor
    
    ESP_LOGI(TAG, "Initializing S%u baseline: serial='%s', sens=%.1f mV/bar", 
             sensor_id, sensor_serial, baseline_sensitivity);
    
    return sensor_calibration_set_baseline(sensor_id, sensor_serial, baseline_sensitivity);
}

esp_err_t sensor_manager_get_calibration_history(uint8_t sensor_id,
                                                 calibration_log_entry_t *entries,
                                                 uint8_t max_entries,
                                                 uint8_t *num_entries)
{
    if (!s_initialized || sensor_id >= 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return sensor_calibration_get_history(sensor_id, entries, max_entries, num_entries);
}

esp_err_t sensor_manager_print_sensor_summary(uint8_t sensor_id)
{
    if (!s_initialized || sensor_id >= 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return sensor_calibration_print_summary(sensor_id);
}

esp_err_t sensor_manager_print_csv_log(uint8_t sensor_id, uint8_t max_entries)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (sensor_id != 255 && sensor_id >= 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return sensor_calibration_print_csv_log(sensor_id, max_entries);
}

esp_err_t sensor_manager_reset_calibration_log(float sensor0_air_mv, float sensor1_air_mv,
                                               const char *sensor0_serial, const char *sensor1_serial)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Sensor manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (sensor0_air_mv <= 0.0f || sensor1_air_mv <= 0.0f) {
        ESP_LOGE(TAG, "Invalid air voltage readings");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!sensor0_serial || !sensor1_serial) {
        ESP_LOGE(TAG, "Serial numbers required");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Resetting calibration log via sensor manager");
    
    return sensor_calibration_reset_log_with_baseline(sensor0_air_mv, sensor1_air_mv,
                                                      sensor0_serial, sensor1_serial);
}

const char* sensor_manager_get_calibration_display_status(uint8_t sensor_id)
{
    if (!s_initialized || sensor_id >= 2) {
        return "err";
    }
    
    // Simple approach: check current calibration directly
    multi_point_calibration_t current_cal;
    esp_err_t cal_ret = sensor_calibration_get_current(sensor_id, &current_cal);
    
    // If no valid calibration exists, return "none"
    if (cal_ret != ESP_OK || !current_cal.valid) {
        return "na"; // No calibration - sensor is uncalibrated
    }
    
    // Analyze calibration type for valid calibrations
    if (current_cal.num_points == 2 && 
        current_cal.points[1].sensor_mv == 0.0f && 
        current_cal.points[1].ppo2_bar == 0.0f) {
        return "1pt"; // Single-point with theoretical zero
    } else if (current_cal.num_points >= 2) {
        return "2pt"; // True multi-point calibration
    } else if (current_cal.num_points == 1) {
        return "1pt"; // Single-point calibration
    }
    
    return "none"; // Fallback for edge cases
}

esp_err_t sensor_manager_reset_sensor(uint8_t sensor_id)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Sensor manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Resetting individual sensor %d via sensor manager", sensor_id);
    
    return sensor_calibration_reset_sensor(sensor_id);
}