/**
 * @file sensor_calibration.c
 * @brief Advanced calibration system implementation for dual O2 sensors
 * @version 1.0
 * @date 2025-01-27
 */

#include "sensor_calibration.h"
#include "app_types.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "CAL";

// NVS storage keys
#define NVS_NAMESPACE               "sensor_cal"
#define NVS_KEY_STORAGE             "cal_data"
#define NVS_KEY_POWER_CYCLES        "power_cycles"
#define NVS_KEY_NEXT_CAL_ID         "next_cal_id"

// Storage optimization: Use separate NVS keys for different data sections
#define NVS_KEY_BASELINES           "baselines"
#define NVS_KEY_THRESHOLDS          "thresholds"
#define NVS_KEY_HISTORY_META        "hist_meta"
#define NVS_KEY_HISTORY_S0          "hist_s0"  // Sensor 0 history
#define NVS_KEY_HISTORY_S1          "hist_s1"  // Sensor 1 history
#define NVS_KEY_CURRENT_CAL         "curr_cal"

// Memory management
static bool s_initialized = false;
calibration_storage_t *s_storage = NULL;  // Global - accessed by other files
static multi_point_calibration_t s_temp_sessions[NUM_O2_SENSORS]; // Temporary calibration sessions

// Default thresholds
static const calibration_thresholds_t DEFAULT_THRESHOLDS = {
    .min_sensitivity_mv_per_bar = 45.0,     // ~10mV in air
    .max_sensitivity_mv_per_bar = 75.0,     // ~17mV in air  
    .max_offset_magnitude_mv = 2.0,         // Small offset acceptable
    .min_correlation_r2 = 0.995,            // High linearity required
    .max_residual_mv = 2.0,                 // Maximum error 2mV
    .caution_sensitivity_threshold = 0.80,  // 80% of baseline = caution
    .fail_sensitivity_threshold = 0.70,     // 70% of baseline = fail
    .caution_offset_threshold_mv = 2.0,     // 2mV offset = caution
    .fail_offset_threshold_mv = 3.0,        // 3mV offset = fail  
    .repeatability_threshold_percent = 1.0   // 1% repeatability
};

// Forward declarations
// Implemented in this TU
static bool validate_storage_integrity(const calibration_storage_t *storage);
static uint32_t get_system_uptime_ms(void);

// Implemented in other files (provide external linkage)
esp_err_t load_from_nvs(void);
esp_err_t save_to_nvs(void);
uint32_t calculate_checksum(const calibration_storage_t *storage);
esp_err_t perform_linear_regression(const calibration_point_t *points, uint8_t num_points,
                                    double *sensitivity, double *offset,
                                    double *correlation_r2, double *max_residual);
esp_err_t run_quality_gates(uint8_t sensor_id, const multi_point_calibration_t *cal,
                            bool *passed_sensitivity, bool *passed_offset,
                            bool *passed_linearity, bool *passed_repeatability);
esp_err_t add_to_history(uint8_t sensor_id, const calibration_log_entry_t *entry);
esp_err_t compute_health_assessment(uint8_t sensor_id, sensor_health_info_t *health);

esp_err_t sensor_calibration_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Calibration system already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing sensor calibration system");
    ESP_LOGI(TAG, "Storage size: %u bytes (%u KB)", sizeof(calibration_storage_t), sizeof(calibration_storage_t) / 1024);
    
    // Allocate storage structure
    s_storage = calloc(1, sizeof(calibration_storage_t));
    if (!s_storage) {
        app_log_error_to_display(TAG, "Failed to allocate storage (%u bytes)", sizeof(calibration_storage_t));
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize default thresholds
    s_storage->thresholds = DEFAULT_THRESHOLDS;
    
    // Initialize temp sessions
    memset(s_temp_sessions, 0, sizeof(s_temp_sessions));
    
    // Try to load existing data from NVS
    esp_err_t ret = load_from_nvs();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load from NVS, using defaults: %s", esp_err_to_name(ret));
        
        // Initialize with defaults
        s_storage->next_calibration_id = 1;
        s_storage->power_cycle_count = 0;
        
        // Initialize sensor baselines and tracking data
        for (int i = 0; i < NUM_O2_SENSORS; i++) {
            s_storage->baselines[i].valid = false;
            s_storage->total_calibrations[i] = 0;
            s_storage->avg_sensitivity_trend[i] = 0.0;
            s_storage->avg_offset_drift[i] = 0.0;
            s_storage->current[i].valid = false;
            memset(&s_storage->previous[i], 0, sizeof(calibration_log_entry_t));
        }
    }
    
    // Check calibration status but DO NOT create default calibrations
    // For SAFETY, sensors must be properly calibrated by the user
    int uncalibrated_count = 0;
    for (int i = 0; i < NUM_O2_SENSORS; i++) {
        if (!s_storage->current[i].valid) {
            uncalibrated_count++;
            ESP_LOGW(TAG, "Sensor %d has NO valid calibration - UNCALIBRATED", i);
        } else {
            ESP_LOGI(TAG, "Sensor %d has valid calibration (ID: %lu)", i, s_storage->current[i].calibration_id);
        }
    }
    
    if (uncalibrated_count > 0) {
        ESP_LOGW(TAG, "%d sensor(s) are UNCALIBRATED - device requires calibration before safe use!", uncalibrated_count);
    } else {
        ESP_LOGI(TAG, "All sensors have valid calibrations");
    }
    
    // Increment power cycle counter
    s_storage->power_cycle_count++;
    ESP_LOGI(TAG, "Power cycle count: %lu", s_storage->power_cycle_count);
    
    // Update checksum and save
    s_storage->checksum = calculate_checksum(s_storage);
    save_to_nvs(); // Save power cycle increment
    
    s_initialized = true;
    ESP_LOGI(TAG, "Calibration system initialized (Cal ID: %lu)", s_storage->next_calibration_id);
    
    return ESP_OK;
}

esp_err_t sensor_calibration_two_point(uint8_t sensor_id, 
                                       const calibration_point_t *point1,
                                       const calibration_point_t *point2,
                                       two_point_calibration_t *result)
{
    if (!s_initialized || !s_storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (sensor_id >= NUM_O2_SENSORS || !point1 || !point2 || !result) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Avoid heavy float formatting to reduce stack/heap usage in logging
    int p1_bar_milli = (int)(point1->ppo2_bar * 1000.0); // millibar units
    int p2_bar_milli = (int)(point2->ppo2_bar * 1000.0);
    int p1_mv_tenths = (int)(point1->sensor_mv * 10.0f);
    int p2_mv_tenths = (int)(point2->sensor_mv * 10.0f);
    ESP_LOGI(TAG, "Two-point cal S%u: P1(%d mbar, %d/10 mV) P2(%d mbar, %d/10 mV)",
             sensor_id, p1_bar_milli, p1_mv_tenths, p2_bar_milli, p2_mv_tenths);
    
    // Clear result
    memset(result, 0, sizeof(two_point_calibration_t));
    
    // Copy points
    result->point1 = *point1;
    result->point2 = *point2;
    result->calibration_id = s_storage->next_calibration_id++;
    result->uptime_ms = get_system_uptime_ms();
    
    // Calculate linear parameters: V = m * PPO2 + b
    double delta_ppo2 = point2->ppo2_bar - point1->ppo2_bar;
    double delta_voltage = point2->sensor_mv - point1->sensor_mv;
    
    if (fabs(delta_ppo2) < 1e-6) {
        ESP_LOGE(TAG, "PPO2 points too close: %.6f bar difference", delta_ppo2);
        return ESP_ERR_INVALID_ARG;
    }
    
    result->sensitivity_mv_per_bar = delta_voltage / delta_ppo2;
    result->offset_mv = point1->sensor_mv - result->sensitivity_mv_per_bar * point1->ppo2_bar;
    
    // For two points, correlation is perfect
    result->correlation_r2 = 1.0;
    result->max_residual_mv = 0.0;
    
    // Validate against quality gates
    bool passed_sens, passed_offset, passed_lin, passed_rep;
    
    // Create temporary multi-point calibration for quality check
    multi_point_calibration_t temp_cal = {
        .num_points = 2,
        .sensitivity_mv_per_bar = result->sensitivity_mv_per_bar,
        .offset_mv = result->offset_mv,
        .correlation_r2 = result->correlation_r2,
        .max_residual_mv = result->max_residual_mv,
        .calibration_id = result->calibration_id,
        .uptime_ms = result->uptime_ms,
        .valid = true
    };
    temp_cal.points[0] = *point1;
    temp_cal.points[1] = *point2;
    
    esp_err_t ret = run_quality_gates(sensor_id, &temp_cal, &passed_sens, &passed_offset, &passed_lin, &passed_rep);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Quality gate evaluation failed");
        return ret;
    }
    
    // Check if this is a single-point calibration with theoretical zero point
    bool is_single_point_with_zero = (point2->sensor_mv == 0.0f && point2->ppo2_bar == 0.0);
    
    if (is_single_point_with_zero) {
        // Single-point + theoretical zero calibration: use relaxed validation
        // Allow calibration if sensitivity is reasonable and offset is within tolerance
        bool relaxed_offset_check = fabs(result->offset_mv) < CALIBRATION_OFFSET_TOLERANCE_MV;
        bool relaxed_sens_check = (result->sensitivity_mv_per_bar > CALIBRATION_SENS_MIN_MV_PER_BAR && 
                                  result->sensitivity_mv_per_bar < CALIBRATION_SENS_MAX_MV_PER_BAR);
        result->valid = (passed_sens || relaxed_sens_check) && (passed_offset || relaxed_offset_check);
        
        // Reduce verbosity to avoid stack pressure from float formatting
        ESP_LOGW(TAG, "Single-point cal S%u (relaxed): sens_pass=%d/%d, off_pass=%d/%d, lin=%d, rep=%d => %s",
                 sensor_id, passed_sens, relaxed_sens_check, passed_offset, relaxed_offset_check,
                 passed_lin, passed_rep, result->valid ? "PASS" : "FAIL");
    } else {
        // Standard two-point validation: all gates must pass
        result->valid = passed_sens && passed_offset && passed_lin && passed_rep;
        ESP_LOGD(TAG, "Standard two-point validation: all gates must pass");
    }
    
    // Downgrade detailed float logs to DEBUG to limit runtime stack usage
    ESP_LOGD(TAG, "Two-point result S%u: m=%.2f mV/bar, b=%.2f mV, valid=%d", 
             sensor_id, result->sensitivity_mv_per_bar, result->offset_mv, result->valid);
    
    if (result->valid) {
        // Update current calibration
        s_storage->current[sensor_id] = temp_cal;
        s_storage->current[sensor_id].valid = true;  // Explicit flag set to ensure it's marked as valid
        
        // Check if this is the first calibration after sensor reset (no baseline exists)
        sensor_baseline_t *baseline = &s_storage->baselines[sensor_id];
        ESP_LOGD(TAG, "S%u: Checking baseline - valid=%d, key='%s'", sensor_id, baseline->valid, baseline->sensor_key);
        
        if (!baseline->valid) {
            ESP_LOGD(TAG, "S%u: First calibration after reset - establishing baseline", sensor_id);
            
            // Use the calibration sensitivity as the baseline sensitivity for health assessment
            baseline->baseline_sensitivity = result->sensitivity_mv_per_bar;
            baseline->install_calibration_id = result->calibration_id;
            baseline->total_calibrations = 0; // Will be incremented below
            baseline->power_cycles = s_storage->power_cycle_count;
            baseline->valid = true;
            
            ESP_LOGD(TAG, "S%u: Baseline established - key='%s', sens=%.3f mV/bar, cal_id=%lu", 
                     sensor_id, baseline->sensor_key, baseline->baseline_sensitivity, 
                     baseline->install_calibration_id);
        } else {
            ESP_LOGD(TAG, "S%u: Baseline already exists - sens=%.3f mV/bar", sensor_id, baseline->baseline_sensitivity);
        }
        
        // Create log entry
        calibration_log_entry_t log_entry = {0};
        log_entry.calibration_id = result->calibration_id;
        log_entry.sensor_id = sensor_id;
        log_entry.power_cycle_count = s_storage->power_cycle_count;
        log_entry.uptime_ms = result->uptime_ms;
        log_entry.num_points = 2;
        log_entry.points[0] = *point1;
        log_entry.points[1] = *point2;
        log_entry.sensitivity_mv_per_bar = result->sensitivity_mv_per_bar;
        log_entry.offset_mv = result->offset_mv;
        log_entry.correlation_r2 = result->correlation_r2;
        log_entry.max_residual_mv = result->max_residual_mv;
        log_entry.passed_sensitivity_check = passed_sens;
        log_entry.passed_offset_check = passed_offset;
        log_entry.passed_linearity_check = passed_lin;
        log_entry.passed_repeatability_check = passed_rep;
        log_entry.calibration_accepted = result->valid;
        
        // Compute health assessment
        compute_health_assessment(sensor_id, &log_entry.health_info);
        log_entry.normalized_sensitivity = log_entry.health_info.normalized_sensitivity;
        
        // Add to history
        add_to_history(sensor_id, &log_entry);
        
        // Save to NVS with error checking to prevent calibration loss
        s_storage->checksum = calculate_checksum(s_storage);
        esp_err_t save_ret = save_to_nvs();
        if (save_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save calibration to NVS, reverting valid flag: %s", esp_err_to_name(save_ret));
            s_storage->current[sensor_id].valid = false;
            result->valid = false;
        } else {
        ESP_LOGD(TAG, "Calibration S%u saved successfully to persistent storage", sensor_id);
        }
        
        // Verify baseline was saved correctly
        ESP_LOGD(TAG, "S%u: Post-save baseline verification - valid=%d, sens=%.3f mV/bar", 
                 sensor_id, s_storage->baselines[sensor_id].valid, s_storage->baselines[sensor_id].baseline_sensitivity);
    }
    
    return ESP_OK;
}

esp_err_t sensor_calibration_start_session(uint8_t sensor_id)
{
    if (!s_initialized || !s_storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (sensor_id >= NUM_O2_SENSORS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Clear temporary session
    memset(&s_temp_sessions[sensor_id], 0, sizeof(multi_point_calibration_t));
    s_temp_sessions[sensor_id].calibration_id = s_storage->next_calibration_id;
    s_temp_sessions[sensor_id].uptime_ms = get_system_uptime_ms();
    
    ESP_LOGD(TAG, "Started calibration session S%u (ID: %lu)", sensor_id, s_temp_sessions[sensor_id].calibration_id);
    return ESP_OK;
}

esp_err_t sensor_calibration_add_point(uint8_t sensor_id, const calibration_point_t *point)
{
    if (!s_initialized || !s_storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (sensor_id >= NUM_O2_SENSORS || !point) {
        return ESP_ERR_INVALID_ARG;
    }
    
    multi_point_calibration_t *session = &s_temp_sessions[sensor_id];
    
    if (session->num_points >= MAX_CALIBRATION_POINTS) {
        ESP_LOGE(TAG, "Too many calibration points (%u max)", MAX_CALIBRATION_POINTS);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Add point to session
    session->points[session->num_points] = *point;
    session->num_points++;
    
    ESP_LOGD(TAG, "Added point %u to S%u: %.3f bar, %.1f mV", 
             session->num_points, sensor_id, point->ppo2_bar, point->sensor_mv);
    
    return ESP_OK;
}

esp_err_t sensor_calibration_finalize(uint8_t sensor_id, multi_point_calibration_t *result)
{
    if (!s_initialized || !s_storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (sensor_id >= NUM_O2_SENSORS || !result) {
        return ESP_ERR_INVALID_ARG;
    }
    
    multi_point_calibration_t *session = &s_temp_sessions[sensor_id];
    
    if (session->num_points < 2) {
        ESP_LOGE(TAG, "Need at least 2 points for calibration (have %u)", session->num_points);
        return ESP_ERR_INVALID_SIZE;
    }
    
    ESP_LOGI(TAG, "Finalizing calibration S%u with %u points", sensor_id, session->num_points);
    
    // Perform linear regression
    esp_err_t ret = perform_linear_regression(session->points, session->num_points,
                                            &session->sensitivity_mv_per_bar,
                                            &session->offset_mv,
                                            &session->correlation_r2,
                                            &session->max_residual_mv);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Linear regression failed");
        return ret;
    }
    
    // Assign calibration ID and increment counter
    session->calibration_id = s_storage->next_calibration_id++;
    
    // Run quality gates
    bool passed_sens, passed_offset, passed_lin, passed_rep;
    ret = run_quality_gates(sensor_id, session, &passed_sens, &passed_offset, &passed_lin, &passed_rep);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Quality gate evaluation failed");
        return ret;
    }
    
    session->valid = passed_sens && passed_offset && passed_lin && passed_rep;
    
    ESP_LOGD(TAG, "Calibration S%u result: m=%.2f mV/bar, b=%.2f mV, R²=%.4f, max_res=%.2f mV, valid=%d",
             sensor_id, session->sensitivity_mv_per_bar, session->offset_mv, 
             session->correlation_r2, session->max_residual_mv, session->valid);
    
    // Copy result
    *result = *session;
    
    if (session->valid) {
        // Update current calibration
        s_storage->current[sensor_id] = *session;
        
        // Check if this is the first calibration after sensor reset (no baseline exists)
        sensor_baseline_t *baseline = &s_storage->baselines[sensor_id];
        ESP_LOGD(TAG, "S%u: Checking baseline - valid=%d, key='%s'", sensor_id, baseline->valid, baseline->sensor_key);
        
        if (!baseline->valid) {
            ESP_LOGD(TAG, "S%u: First calibration after reset - establishing baseline", sensor_id);
            
            // Use the calibration sensitivity as the baseline sensitivity for health assessment
            baseline->baseline_sensitivity = session->sensitivity_mv_per_bar;
            baseline->install_calibration_id = session->calibration_id;
            baseline->total_calibrations = 0; // Will be incremented below
            baseline->power_cycles = s_storage->power_cycle_count;
            baseline->valid = true;
            
            ESP_LOGD(TAG, "S%u: Baseline established - key='%s', sens=%.3f mV/bar, cal_id=%lu", 
                     sensor_id, baseline->sensor_key, baseline->baseline_sensitivity, 
                     baseline->install_calibration_id);
        } else {
            ESP_LOGD(TAG, "S%u: Baseline already exists - sens=%.3f mV/bar", sensor_id, baseline->baseline_sensitivity);
        }
        
        // Create log entry
        calibration_log_entry_t log_entry = {0};
        log_entry.calibration_id = session->calibration_id;
        log_entry.sensor_id = sensor_id;
        log_entry.power_cycle_count = s_storage->power_cycle_count;
        log_entry.uptime_ms = session->uptime_ms;
        log_entry.num_points = session->num_points;
        memcpy(log_entry.points, session->points, sizeof(calibration_point_t) * session->num_points);
        log_entry.sensitivity_mv_per_bar = session->sensitivity_mv_per_bar;
        log_entry.offset_mv = session->offset_mv;
        log_entry.correlation_r2 = session->correlation_r2;
        log_entry.max_residual_mv = session->max_residual_mv;
        log_entry.passed_sensitivity_check = passed_sens;
        log_entry.passed_offset_check = passed_offset;
        log_entry.passed_linearity_check = passed_lin;
        log_entry.passed_repeatability_check = passed_rep;
        log_entry.calibration_accepted = session->valid;
        
        // Compute health assessment
        compute_health_assessment(sensor_id, &log_entry.health_info);
        log_entry.normalized_sensitivity = log_entry.health_info.normalized_sensitivity;
        
        // Add to history
        add_to_history(sensor_id, &log_entry);
        
        // Save to NVS
        s_storage->checksum = calculate_checksum(s_storage);
        esp_err_t save_ret = save_to_nvs();
        if (save_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save calibration data to NVS: %s", esp_err_to_name(save_ret));
        } else {
            ESP_LOGD(TAG, "Calibration data saved to NVS successfully");
        }
        
        // Verify baseline was saved correctly
        ESP_LOGD(TAG, "S%u: Post-save baseline verification - valid=%d, sens=%.3f mV/bar", 
                 sensor_id, s_storage->baselines[sensor_id].valid, s_storage->baselines[sensor_id].baseline_sensitivity);
    }
    
    // Clear temp session
    memset(session, 0, sizeof(multi_point_calibration_t));
    
    return ESP_OK;
}

esp_err_t sensor_calibration_voltage_to_ppo2(uint8_t sensor_id, float voltage_mv, double *ppo2_bar)
{
    if (!s_initialized || !s_storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (sensor_id >= NUM_O2_SENSORS || !ppo2_bar) {
        return ESP_ERR_INVALID_ARG;
    }
    
    const multi_point_calibration_t *cal = &s_storage->current[sensor_id];
    
    if (!cal->valid) {
        ESP_LOGW(TAG, "No valid calibration for sensor %u", sensor_id);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Convert using linear model: PPO2 = (V - b) / m
    *ppo2_bar = (voltage_mv - cal->offset_mv) / cal->sensitivity_mv_per_bar;
    
    // Clamp to reasonable range
    if (*ppo2_bar < 0.0) *ppo2_bar = 0.0;
    if (*ppo2_bar > 5.0) *ppo2_bar = 5.0;
    
    return ESP_OK;
}

esp_err_t sensor_calibration_assess_health(uint8_t sensor_id, sensor_health_info_t *health_info)
{
    if (!s_initialized || !s_storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (sensor_id >= NUM_O2_SENSORS || !health_info) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return compute_health_assessment(sensor_id, health_info);
}

// Additional helper functions would continue here...
// For brevity, I'll implement the key helper functions

static uint32_t get_system_uptime_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

uint32_t calculate_checksum(const calibration_storage_t *storage)
{
    // Simple checksum - XOR of all 32-bit words except checksum itself
    const uint32_t *data = (const uint32_t *)storage;
    size_t words = (sizeof(calibration_storage_t) - sizeof(uint32_t)) / sizeof(uint32_t);
    uint32_t checksum = 0;
    
    for (size_t i = 0; i < words; i++) {
        checksum ^= data[i];
    }
    
    return checksum;
}

static bool validate_storage_integrity(const calibration_storage_t *storage)
{
    uint32_t calculated = calculate_checksum(storage);
    return calculated == storage->checksum;
}

const char* sensor_calibration_health_string(sensor_health_t health)
{
    switch (health) {
        case SENSOR_HEALTH_GOOD: return "GOOD";
        case SENSOR_HEALTH_CAUTION: return "CAUTION";
        case SENSOR_HEALTH_FAIL: return "FAIL";
        case SENSOR_HEALTH_UNKNOWN: return "UNKNOWN";
        default: return "INVALID";
    }
}

void sensor_calibration_deinit(void)
{
    if (s_initialized) {
        if (s_storage) {
            // Save current state before freeing
            save_to_nvs();
            free(s_storage);
            s_storage = NULL;
        }
        s_initialized = false;
        ESP_LOGI(TAG, "Calibration system deinitialized");
    }
}

// External function declarations (implemented in separate files)
esp_err_t get_calibration_history(uint8_t sensor_id, calibration_log_entry_t *entries,
                                  uint8_t max_entries, uint8_t *num_entries);

// Implementation of remaining API functions
esp_err_t sensor_calibration_get_history(uint8_t sensor_id,
                                         calibration_log_entry_t *entries,
                                         uint8_t max_entries,
                                         uint8_t *num_entries)
{
    return get_calibration_history(sensor_id, entries, max_entries, num_entries);
}

esp_err_t sensor_calibration_save(void)
{
    return save_to_nvs();
}

esp_err_t sensor_calibration_load(void)
{
    return load_from_nvs();
}

esp_err_t sensor_calibration_get_current(uint8_t sensor_id, multi_point_calibration_t *calibration)
{
    if (!s_initialized || !s_storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (sensor_id >= NUM_O2_SENSORS || !calibration) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *calibration = s_storage->current[sensor_id];
    return ESP_OK;
}

esp_err_t sensor_calibration_get_thresholds(calibration_thresholds_t *thresholds)
{
    if (!s_initialized || !s_storage || !thresholds) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *thresholds = s_storage->thresholds;
    return ESP_OK;
}

esp_err_t sensor_calibration_set_thresholds(const calibration_thresholds_t *thresholds)
{
    if (!s_initialized || !s_storage || !thresholds) {
        return ESP_ERR_INVALID_ARG;
    }
    
    s_storage->thresholds = *thresholds;
    s_storage->checksum = calculate_checksum(s_storage);
    return save_to_nvs();
}

esp_err_t sensor_calibration_clear_history(uint8_t sensor_id)
{
    if (!s_initialized || !s_storage || sensor_id >= NUM_O2_SENSORS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Reset calibration tracking data
    s_storage->total_calibrations[sensor_id] = 0;
    s_storage->avg_sensitivity_trend[sensor_id] = 0.0;
    s_storage->avg_offset_drift[sensor_id] = 0.0;
    memset(&s_storage->previous[sensor_id], 0, sizeof(calibration_log_entry_t));
    
    // TODO: Also clear calibration records from NVS if needed
    
    s_storage->checksum = calculate_checksum(s_storage);
    return save_to_nvs();
}

esp_err_t sensor_calibration_generate_uuid(uint8_t *uuid)
{
    if (!uuid) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Since we don't have RTC, use a simple counter-based UUID
    static uint32_t uuid_counter = 0;
    uuid_counter++;
    
    memset(uuid, 0, CALIBRATION_UUID_SIZE);
    
    // Pack counter and some system info into UUID
    uint32_t uptime = get_system_uptime_ms();
    memcpy(&uuid[0], &uuid_counter, sizeof(uint32_t));
    memcpy(&uuid[4], &uptime, sizeof(uint32_t));
    
    if (s_storage) {
        memcpy(&uuid[8], &s_storage->power_cycle_count, sizeof(uint32_t));
        memcpy(&uuid[12], &s_storage->next_calibration_id, sizeof(uint32_t));
    }
    
    return ESP_OK;
}

uint32_t sensor_calibration_get_power_cycles(void)
{
    return s_storage ? s_storage->power_cycle_count : 0;
}

esp_err_t sensor_calibration_increment_power_cycle(void)
{
    if (!s_initialized || !s_storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    s_storage->power_cycle_count++;
    s_storage->checksum = calculate_checksum(s_storage);
    return save_to_nvs();
}

esp_err_t sensor_calibration_print_csv_log(uint8_t sensor_id, uint8_t max_entries)
{
    if (!s_initialized || !s_storage) {
        ESP_LOGE(TAG, "Calibration system not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (sensor_id != 255 && sensor_id >= NUM_O2_SENSORS) {
        ESP_LOGE(TAG, "Invalid sensor ID: %u", sensor_id);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "=== CALIBRATION LOG CSV FORMAT ===");
    
    // Print CSV header
    ESP_LOGI(TAG, "SensorID,CalID,PowerCycle,UptimeMs,NumPoints,Gas1_PPO2,Gas1_mV,Gas2_PPO2,Gas2_mV,"
                  "Sensitivity_mV_per_bar,Offset_mV,R2,MaxRes_mV,NormSens,HealthStatus,"
                  "PassSens,PassOffset,PassLin,PassRep,Accepted,Temperature");
    
    uint8_t start_sensor = (sensor_id == 255) ? 0 : sensor_id;
    uint8_t end_sensor = (sensor_id == 255) ? NUM_O2_SENSORS - 1 : sensor_id;
    
    for (uint8_t sid = start_sensor; sid <= end_sensor; sid++) {
        uint32_t total_cals = s_storage->total_calibrations[sid];
        
        if (total_cals == 0) {
            ESP_LOGI(TAG, "No calibration history for sensor %u", sid);
            continue;
        }
        
        ESP_LOGI(TAG, "Sensor %u has %lu total calibrations", sid, total_cals);
        
        // Read calibration records from NVS (starting from the most recent)
        // We'll iterate through possible calibration IDs and try to load them
        nvs_handle_t nvs_handle;
        esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to open NVS for reading calibration history");
            continue;
        }
        
        uint32_t entries_printed = 0;
        uint32_t max_to_print = (max_entries == 0) ? total_cals : max_entries;
        
        // Try to load recent calibration records by ID (starting from latest known ID and working backwards)
        uint32_t current_cal_id = (s_storage->current[sid].valid) ? s_storage->current[sid].calibration_id : s_storage->total_calibrations[sid];
        
        for (uint32_t cal_id = current_cal_id; cal_id >= 1 && entries_printed < max_to_print; cal_id--) {
            char cal_key[32];
            snprintf(cal_key, sizeof(cal_key), "cal_s%u_%lu", sid, cal_id);
            
            calibration_log_entry_t entry;
            size_t required_size = sizeof(calibration_log_entry_t);
            
            ret = nvs_get_blob(nvs_handle, cal_key, &entry, &required_size);
            if (ret == ESP_OK) {
                // Extract meaningful calibration points (first 2 points for simplicity)
                double gas1_ppo2 = (entry.num_points > 0) ? entry.points[0].ppo2_bar : 0.0;
                double gas1_mv = (entry.num_points > 0) ? entry.points[0].sensor_mv : 0.0;
                double gas2_ppo2 = (entry.num_points > 1) ? entry.points[1].ppo2_bar : 0.0;
                double gas2_mv = (entry.num_points > 1) ? entry.points[1].sensor_mv : 0.0;
                
                // Health status as string
                const char* health_str = sensor_calibration_health_string(entry.health_info.health_status);
                
                // Format CSV line (split into two ESP_LOGI calls due to line length limits)
                ESP_LOGI(TAG, "%u,%lu,%lu,%lu,%u,%.3f,%.1f,%.3f,%.1f,%.3f,%.3f,%.5f,%.2f,%.3f,%s",
                         entry.sensor_id,                          // SensorID
                         entry.calibration_id,                     // CalID  
                         entry.power_cycle_count,                  // PowerCycle
                         entry.uptime_ms,                          // UptimeMs
                         entry.num_points,                         // NumPoints
                         gas1_ppo2,                                 // Gas1_PPO2
                         gas1_mv,                                   // Gas1_mV
                         gas2_ppo2,                                 // Gas2_PPO2
                         gas2_mv,                                   // Gas2_mV
                         entry.sensitivity_mv_per_bar,             // Sensitivity
                         entry.offset_mv,                          // Offset
                         entry.correlation_r2,                     // R2
                         entry.max_residual_mv,                    // MaxRes
                         entry.normalized_sensitivity,             // NormSens
                         health_str);                              // HealthStatus
                
                entries_printed++;
            }
        }
        
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "--- End sensor %u history (%lu entries) ---", sid, entries_printed);
    }
    
    ESP_LOGI(TAG, "=== END CALIBRATION LOG ===");
    return ESP_OK;
}

esp_err_t sensor_calibration_reset_log_with_baseline(float sensor0_air_mv, float sensor1_air_mv,
                                                     const char *sensor0_serial, const char *sensor1_serial)
{
    if (!s_initialized || !s_storage) {
        ESP_LOGE(TAG, "Calibration system not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (sensor0_air_mv <= 0.0f || sensor1_air_mv <= 0.0f) {
        ESP_LOGE(TAG, "Invalid air voltage readings: S0=%.1fmV, S1=%.1fmV", sensor0_air_mv, sensor1_air_mv);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!sensor0_serial || !sensor1_serial) {
        ESP_LOGE(TAG, "Serial numbers cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Resetting calibration log with baseline calibration:");
    ESP_LOGI(TAG, "  Sensor 0: %.1fmV @ 21%% O2, serial='%s'", sensor0_air_mv, sensor0_serial);
    ESP_LOGI(TAG, "  Sensor 1: %.1fmV @ 21%% O2, serial='%s'", sensor1_air_mv, sensor1_serial);
    
    // Clear all existing calibration data
    s_storage->next_calibration_id = 1;  // Start from calibration ID 1
    s_storage->power_cycle_count++; // Increment power cycle for this reset operation
    
    // Clear tracking data for both sensors
    for (int sid = 0; sid < NUM_O2_SENSORS; sid++) {
        s_storage->total_calibrations[sid] = 0;
        s_storage->avg_sensitivity_trend[sid] = 0.0;
        s_storage->avg_offset_drift[sid] = 0.0;
        memset(&s_storage->previous[sid], 0, sizeof(calibration_log_entry_t));
        memset(&s_storage->current[sid], 0, sizeof(multi_point_calibration_t));
        s_storage->current[sid].valid = false;
    }
    
    // Current system uptime for consistent timestamps
    uint32_t current_uptime = get_system_uptime_ms();
    
    // Standard calibration conditions
    const float air_pressure = 1.013f;      // Standard atmospheric pressure (bar)
    const float air_o2_fraction = 0.2095f;  // Air O2 fraction (20.95%)
    const float nitrogen_pressure = 1.013f; // Nitrogen pressure (bar)
    const float nitrogen_o2_fraction = 0.0f; // Pure nitrogen (0% O2)
    const float nitrogen_mv = 0.0f;         // Expected reading in pure nitrogen
    const float temperature = 25.0f;        // Assumed room temperature
    
    // Process each sensor
    float air_mv_values[NUM_O2_SENSORS] = {sensor0_air_mv, sensor1_air_mv};
    const char* serial_numbers[NUM_O2_SENSORS] = {sensor0_serial, sensor1_serial};
    
    // Clear all calibration data for both sensors - DO NOT create any calibrations
    // This is for SAFETY - sensors must be properly calibrated before use
    for (int sid = 0; sid < NUM_O2_SENSORS; sid++) {
        ESP_LOGI(TAG, "Clearing ALL calibration data for sensor %d - sensor will be UNCALIBRATED", sid);
        
        // Clear current calibration completely
        memset(&s_storage->current[sid], 0, sizeof(multi_point_calibration_t));
        s_storage->current[sid].valid = false;  // Mark as invalid/uncalibrated
        s_storage->current[sid].calibration_id = 0;  // No calibration ID
        
        // Clear baseline data
        memset(&s_storage->baselines[sid], 0, sizeof(sensor_baseline_t));
        s_storage->baselines[sid].valid = false;
        
        // Reset calibration counters
        s_storage->total_calibrations[sid] = 0;
        s_storage->avg_sensitivity_trend[sid] = 0.0;
        s_storage->avg_offset_drift[sid] = 0.0;
        
        ESP_LOGW(TAG, "Sensor %d is now UNCALIBRATED - must be calibrated before use!", sid);
    }
    
    // Update checksum and save to NVS
    s_storage->checksum = calculate_checksum(s_storage);
    esp_err_t save_ret = save_to_nvs();
    if (save_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save reset calibration log to NVS: %s", esp_err_to_name(save_ret));
        return save_ret;
    }
    
    ESP_LOGI(TAG, "Calibration log reset completed successfully");
    ESP_LOGI(TAG, "  Next calibration ID: %lu", s_storage->next_calibration_id);
    ESP_LOGI(TAG, "  Power cycle count: %lu", s_storage->power_cycle_count);
    ESP_LOGI(TAG, "  Both sensors have 1 baseline calibration entry");
    
    return ESP_OK;
}

esp_err_t sensor_calibration_reset_sensor(uint8_t sensor_id)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Calibration system not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (sensor_id >= NUM_O2_SENSORS) {
        ESP_LOGE(TAG, "Invalid sensor ID: %d", sensor_id);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Resetting calibration data for sensor %d", sensor_id);
    
    // Increment sensor install counter
    s_storage->sensor_install_counters[sensor_id]++;
    uint32_t install_count = s_storage->sensor_install_counters[sensor_id];
    
    // Generate auto-incrementing serial number: s1XX for sensor 0, s2XX for sensor 1
    char auto_serial[SENSOR_BASELINE_KEY_SIZE];
    snprintf(auto_serial, sizeof(auto_serial), "s%d%02lu", 
             sensor_id + 1, install_count);
    
    ESP_LOGI(TAG, "Assigning auto-generated serial number: %s", auto_serial);
    
    // Clear all calibration data for this sensor - DO NOT create any calibrations
    // This is for SAFETY - sensor must be properly calibrated before use
    ESP_LOGI(TAG, "Clearing ALL calibration data for sensor %d - sensor will be UNCALIBRATED", sensor_id);
    
    // Clear current calibration completely
    memset(&s_storage->current[sensor_id], 0, sizeof(multi_point_calibration_t));
    s_storage->current[sensor_id].valid = false;  // Mark as invalid/uncalibrated
    s_storage->current[sensor_id].calibration_id = 0;  // No calibration ID
    
    // Clear baseline data and set new serial
    memset(&s_storage->baselines[sensor_id], 0, sizeof(sensor_baseline_t));
    strncpy(s_storage->baselines[sensor_id].sensor_key, auto_serial, SENSOR_BASELINE_KEY_SIZE - 1);
    s_storage->baselines[sensor_id].sensor_key[SENSOR_BASELINE_KEY_SIZE - 1] = '\0';
    s_storage->baselines[sensor_id].valid = false;  // No baseline established yet
    s_storage->baselines[sensor_id].install_calibration_id = 0;  // Will be set on first calibration
    s_storage->baselines[sensor_id].total_calibrations = 0;
    s_storage->baselines[sensor_id].power_cycles = s_storage->power_cycle_count;
    
    // Clear previous calibration data
    memset(&s_storage->previous[sensor_id], 0, sizeof(calibration_log_entry_t));
    
    // Reset calibration counters and trends
    s_storage->total_calibrations[sensor_id] = 0;
    s_storage->avg_sensitivity_trend[sensor_id] = 0.0;
    s_storage->avg_offset_drift[sensor_id] = 0.0;
    
    ESP_LOGW(TAG, "Sensor %d is now UNCALIBRATED with serial %s - must be calibrated before use!", 
             sensor_id, auto_serial);
    
    // Update checksum and save to NVS
    s_storage->checksum = calculate_checksum(s_storage);
    esp_err_t save_ret = save_to_nvs();
    if (save_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save sensor %d reset to NVS: %s", sensor_id, esp_err_to_name(save_ret));
        return save_ret;
    }
    
    ESP_LOGI(TAG, "Sensor %d reset completed successfully", sensor_id);
    ESP_LOGI(TAG, "  New serial number: %s", auto_serial);
    ESP_LOGI(TAG, "  Install count: %lu", install_count);
    ESP_LOGI(TAG, "  Sensor is UNCALIBRATED and requires calibration before use");
    
    return ESP_OK;
}
