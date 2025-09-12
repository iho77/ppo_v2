/**
 * @file sensor_health.c
 * @brief Sensor health monitoring and degradation tracking
 * @version 1.0  
 * @date 2025-01-27
 */

#include "sensor_calibration.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "SENSOR_HEALTH";

// External storage reference
extern calibration_storage_t *s_storage;

// External function declarations
extern uint32_t calculate_checksum(const calibration_storage_t *storage);
extern esp_err_t save_to_nvs(void);

/**
 * @brief Perform linear regression on sensitivity vs calibration sequence
 * Used for degradation rate analysis
 */
static esp_err_t analyze_sensitivity_trend(const calibration_log_entry_t *entries, uint8_t num_entries,
                                          double *degradation_rate, uint32_t *predicted_eol_cals, bool *trend_valid)
{
    if (!entries || num_entries < 5 || !degradation_rate || !predicted_eol_cals || !trend_valid) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *trend_valid = false;
    *degradation_rate = 0.0;
    *predicted_eol_cals = 0;
    
    // Need at least 5 points for meaningful trend analysis
    if (num_entries < 5) {
        ESP_LOGD(TAG, "Insufficient data for trend analysis (%u entries)", num_entries);
        return ESP_OK;
    }
    
    // Calculate linear regression: normalized_sensitivity vs calibration_sequence
    double sum_x = 0.0, sum_y = 0.0;
    double sum_xy = 0.0, sum_xx = 0.0;
    
    for (uint8_t i = 0; i < num_entries; i++) {
        double x = (double)i;  // Calibration sequence (0 = oldest, num_entries-1 = newest)
        double y = entries[i].normalized_sensitivity;
        
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
    }
    
    double n = (double)num_entries;
    double denominator = n * sum_xx - sum_x * sum_x;
    
    if (fabs(denominator) < 1e-12) {
        ESP_LOGD(TAG, "Cannot compute trend - degenerate regression");
        return ESP_OK;
    }
    
    // Calculate slope (degradation rate per calibration)
    double slope = (n * sum_xy - sum_x * sum_y) / denominator;
    double intercept = (sum_y - slope * sum_x) / n;
    
    // Calculate R² to assess trend quality
    double ss_tot = 0.0, ss_res = 0.0;
    double mean_y = sum_y / n;
    
    for (uint8_t i = 0; i < num_entries; i++) {
        double x = (double)i;
        double y_actual = entries[i].normalized_sensitivity;
        double y_predicted = slope * x + intercept;
        
        ss_tot += (y_actual - mean_y) * (y_actual - mean_y);
        ss_res += (y_actual - y_predicted) * (y_actual - y_predicted);
    }
    
    double r_squared = (ss_tot > 1e-12) ? (1.0 - ss_res / ss_tot) : 0.0;
    
    // Trend is valid if:
    // 1. R² > 0.5 (reasonable correlation)
    // 2. Degradation rate is negative (sensitivity decreasing)
    // 3. Degradation rate is reasonable (not too fast/slow)
    *trend_valid = (r_squared > 0.5) && (slope < 0.0) && (slope > -0.1);  // Max 10% loss per cal
    
    if (*trend_valid) {
        *degradation_rate = -slope;  // Positive rate for loss
        
        // Predict EOL: when will normalized sensitivity reach 0.70?
        double current_sensitivity = entries[num_entries - 1].normalized_sensitivity;
        double eol_threshold = 0.70;
        
        if (current_sensitivity > eol_threshold && slope < 0.0) {
            double cals_to_eol = (current_sensitivity - eol_threshold) / (-slope);
            *predicted_eol_cals = (uint32_t)cals_to_eol;
        } else {
            *predicted_eol_cals = 0;  // Already at EOL or increasing
        }
        
        ESP_LOGI(TAG, "Sensitivity trend: slope=%.6f/cal, R²=%.3f, EOL_cals=%lu", 
                 slope, r_squared, *predicted_eol_cals);
    } else {
        ESP_LOGD(TAG, "Invalid trend: slope=%.6f, R²=%.3f", slope, r_squared);
    }
    
    return ESP_OK;
}

/**
 * @brief Compute health assessment for a sensor
 * @param sensor_id Sensor ID (0 or 1)
 * @param health_info Output health assessment
 * @return ESP_OK on success
 */
esp_err_t compute_health_assessment(uint8_t sensor_id, sensor_health_info_t *health_info)
{
    if (!s_storage || sensor_id >= NUM_O2_SENSORS || !health_info) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Initialize health info
    memset(health_info, 0, sizeof(sensor_health_info_t));
    health_info->health_status = SENSOR_HEALTH_UNKNOWN;
    health_info->normalized_sensitivity = 1.0;
    
    // Check if we have a current calibration
    const multi_point_calibration_t *current_cal = &s_storage->current[sensor_id];
    if (!current_cal->valid) {
        ESP_LOGW(TAG, "S%u: No valid calibration for health assessment", sensor_id);
        return ESP_OK;
    }
    
    // Check if we have a baseline
    const sensor_baseline_t *baseline = &s_storage->baselines[sensor_id];
    if (!baseline->valid) {
        ESP_LOGW(TAG, "S%u: No baseline for health assessment", sensor_id);
        health_info->health_status = SENSOR_HEALTH_UNKNOWN;
        return ESP_OK;
    }
    
    // Calculate current health metrics
    health_info->normalized_sensitivity = current_cal->sensitivity_mv_per_bar / baseline->baseline_sensitivity;
    health_info->offset_drift_mv = current_cal->offset_mv;
    health_info->linearity_r2 = current_cal->correlation_r2;
    health_info->max_residual_mv = current_cal->max_residual_mv;
    health_info->last_assessment_cal_id = current_cal->calibration_id;
    
    // Use running averages for trend analysis (no full history needed)
    health_info->degradation_rate_per_calibration = s_storage->avg_sensitivity_trend[sensor_id];
    health_info->trend_valid = s_storage->total_calibrations[sensor_id] >= 5; // Need at least 5 calibrations
    
    // Simple EOL prediction based on current degradation rate
    if (health_info->trend_valid && health_info->degradation_rate_per_calibration > 0) {
        double remaining_sensitivity = health_info->normalized_sensitivity - 0.70; // EOL threshold
        health_info->predicted_eol_calibrations = (uint32_t)(remaining_sensitivity / health_info->degradation_rate_per_calibration);
    } else {
        health_info->predicted_eol_calibrations = 0;
    }
    
    // Assess health status based on thresholds
    const calibration_thresholds_t *thresh = &s_storage->thresholds;
    
    bool sens_caution = health_info->normalized_sensitivity < thresh->caution_sensitivity_threshold;
    bool sens_fail = health_info->normalized_sensitivity < thresh->fail_sensitivity_threshold;
    bool offset_caution = fabs(health_info->offset_drift_mv) > thresh->caution_offset_threshold_mv;
    bool offset_fail = fabs(health_info->offset_drift_mv) > thresh->fail_offset_threshold_mv;
    bool linearity_fail = (health_info->linearity_r2 < thresh->min_correlation_r2) || 
                         (health_info->max_residual_mv > thresh->max_residual_mv);
    
    // Determine overall health status
    if (sens_fail || offset_fail || linearity_fail) {
        health_info->health_status = SENSOR_HEALTH_FAIL;
    } else if (sens_caution || offset_caution) {
        health_info->health_status = SENSOR_HEALTH_CAUTION;
    } else {
        health_info->health_status = SENSOR_HEALTH_GOOD;
    }
    
    ESP_LOGI(TAG, "S%u health assessment:", sensor_id);
    ESP_LOGI(TAG, "  Normalized sensitivity: %.3f", health_info->normalized_sensitivity);
    ESP_LOGI(TAG, "  Offset drift: %.2f mV", health_info->offset_drift_mv);  
    ESP_LOGI(TAG, "  Linearity R²: %.5f", health_info->linearity_r2);
    ESP_LOGI(TAG, "  Max residual: %.2f mV", health_info->max_residual_mv);
    ESP_LOGI(TAG, "  Status: %s", sensor_calibration_health_string(health_info->health_status));
    
    if (health_info->trend_valid) {
        ESP_LOGI(TAG, "  Degradation rate: %.4f per calibration", health_info->degradation_rate_per_calibration);
        ESP_LOGI(TAG, "  Predicted EOL: %lu calibrations", health_info->predicted_eol_calibrations);
    }
    
    return ESP_OK;
}

/**
 * @brief Set sensor baseline parameters (for new sensors)
 * @param sensor_id Sensor ID (0 or 1)
 * @param sensor_key Unique sensor identifier
 * @param baseline_sensitivity Initial sensitivity when new
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_set_baseline(uint8_t sensor_id, 
                                          const char *sensor_key,
                                          double baseline_sensitivity)
{
    if (!s_storage || sensor_id >= NUM_O2_SENSORS || !sensor_key) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (baseline_sensitivity <= 0.0) {
        ESP_LOGE(TAG, "Invalid baseline sensitivity: %.3f", baseline_sensitivity);
        return ESP_ERR_INVALID_ARG;
    }
    
    sensor_baseline_t *baseline = &s_storage->baselines[sensor_id];
    
    // Set baseline parameters
    strncpy(baseline->sensor_key, sensor_key, SENSOR_BASELINE_KEY_SIZE - 1);
    baseline->sensor_key[SENSOR_BASELINE_KEY_SIZE - 1] = '\0';
    baseline->baseline_sensitivity = baseline_sensitivity;
    baseline->install_calibration_id = s_storage->next_calibration_id;
    baseline->total_calibrations = 0;
    baseline->power_cycles = s_storage->power_cycle_count;
    baseline->valid = true;
    
    ESP_LOGI(TAG, "S%u baseline set: key='%s', sens=%.3f mV/bar, cal_id=%lu", 
             sensor_id, sensor_key, baseline_sensitivity, baseline->install_calibration_id);
    
    // Save to NVS
    s_storage->checksum = calculate_checksum(s_storage);
    save_to_nvs();
    
    return ESP_OK;
}

/**
 * @brief Detect if a sensor has been replaced
 * @param sensor_id Sensor ID (0 or 1)
 * @param current_key Current sensor key to check
 * @param is_new_sensor Output: true if sensor appears to be new/replaced
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_detect_replacement(uint8_t sensor_id,
                                               const char *current_key,
                                               bool *is_new_sensor)
{
    if (!s_storage || sensor_id >= NUM_O2_SENSORS || !current_key || !is_new_sensor) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *is_new_sensor = false;
    
    const sensor_baseline_t *baseline = &s_storage->baselines[sensor_id];
    
    if (!baseline->valid) {
        // No baseline exists - treat as new sensor
        *is_new_sensor = true;
        ESP_LOGI(TAG, "S%u: No existing baseline, treating as new sensor", sensor_id);
        return ESP_OK;
    }
    
    // Compare sensor keys
    if (strcmp(baseline->sensor_key, current_key) != 0) {
        *is_new_sensor = true;
        ESP_LOGI(TAG, "S%u: Sensor key changed from '%s' to '%s' - sensor replaced", 
                 sensor_id, baseline->sensor_key, current_key);
        return ESP_OK;
    }
    
    ESP_LOGD(TAG, "S%u: Sensor key unchanged ('%s') - same sensor", sensor_id, current_key);
    return ESP_OK;
}

/**
 * @brief Reset sensor baseline (for sensor replacement)
 * @param sensor_id Sensor ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_reset_baseline(uint8_t sensor_id)
{
    if (!s_storage || sensor_id >= NUM_O2_SENSORS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "S%u: Resetting baseline for sensor replacement", sensor_id);
    
    // Clear baseline
    memset(&s_storage->baselines[sensor_id], 0, sizeof(sensor_baseline_t));
    s_storage->baselines[sensor_id].valid = false;
    
    // Clear calibration tracking data
    s_storage->total_calibrations[sensor_id] = 0;
    s_storage->avg_sensitivity_trend[sensor_id] = 0.0;
    s_storage->avg_offset_drift[sensor_id] = 0.0;
    memset(&s_storage->previous[sensor_id], 0, sizeof(calibration_log_entry_t));
    
    // Clear current calibration
    memset(&s_storage->current[sensor_id], 0, sizeof(multi_point_calibration_t));
    s_storage->current[sensor_id].valid = false;
    
    // Save to NVS
    s_storage->checksum = calculate_checksum(s_storage);
    save_to_nvs();
    
    ESP_LOGI(TAG, "S%u: Baseline and history cleared for new sensor", sensor_id);
    
    return ESP_OK;
}

/**
 * @brief Print sensor health summary for debugging
 * @param sensor_id Sensor ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_print_summary(uint8_t sensor_id)
{
    if (!s_storage || sensor_id >= NUM_O2_SENSORS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "=== SENSOR %u SUMMARY ===", sensor_id);
    
    // Baseline info
    const sensor_baseline_t *baseline = &s_storage->baselines[sensor_id];
    if (baseline->valid) {
        ESP_LOGI(TAG, "Baseline: key='%s', sens=%.3f mV/bar, total_cals=%lu", 
                 baseline->sensor_key, baseline->baseline_sensitivity, baseline->total_calibrations);
    } else {
        ESP_LOGI(TAG, "Baseline: NOT SET");
    }
    
    // Current calibration
    const multi_point_calibration_t *current = &s_storage->current[sensor_id];
    if (current->valid) {
        ESP_LOGI(TAG, "Current cal: ID=%lu, sens=%.3f mV/bar, offset=%.3f mV, R²=%.5f", 
                 current->calibration_id, current->sensitivity_mv_per_bar, 
                 current->offset_mv, current->correlation_r2);
    } else {
        ESP_LOGI(TAG, "Current cal: NOT SET");
    }
    
    // Health assessment
    sensor_health_info_t health;
    esp_err_t ret = compute_health_assessment(sensor_id, &health);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Health: %s (norm_sens=%.3f, offset=%.2fmV)", 
                 sensor_calibration_health_string(health.health_status),
                 health.normalized_sensitivity, health.offset_drift_mv);
        
        if (health.trend_valid) {
            ESP_LOGI(TAG, "Trend: %.4f loss/cal, EOL in %lu cals", 
                     health.degradation_rate_per_calibration, health.predicted_eol_calibrations);
        }
    }
    
    // Calibration tracking summary
    ESP_LOGI(TAG, "Total calibrations: %lu, avg_sens_trend: %.4f, avg_offset_drift: %.2f", 
             s_storage->total_calibrations[sensor_id], 
             s_storage->avg_sensitivity_trend[sensor_id],
             s_storage->avg_offset_drift[sensor_id]);
    
    ESP_LOGI(TAG, "=== END SUMMARY ===");
    
    return ESP_OK;
}