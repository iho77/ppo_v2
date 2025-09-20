/**
 * @file sensor_calibration_math.c
 * @brief Mathematical functions for sensor calibration system
 * @version 1.0
 * @date 2025-01-27
 */

#include "sensor_calibration.h"
#include "esp_log.h"
#include <math.h>
#include <float.h>

static const char *TAG = "CAL_MATH";

/**
 * @brief Perform ordinary least squares linear regression
 * @param points Array of calibration points
 * @param num_points Number of points (2-6)
 * @param sensitivity Output: slope (mV/bar)
 * @param offset Output: intercept (mV)
 * @param correlation_r2 Output: correlation coefficient R²
 * @param max_residual Output: maximum absolute residual (mV)
 * @return ESP_OK on success
 */
esp_err_t perform_linear_regression(const calibration_point_t *points, uint8_t num_points,
                                   double *sensitivity, double *offset, 
                                   double *correlation_r2, double *max_residual)
{
    if (!points || num_points < 2 || num_points > MAX_CALIBRATION_POINTS) {
        ESP_LOGE(TAG, "Invalid parameters for regression");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!sensitivity || !offset || !correlation_r2 || !max_residual) {
        ESP_LOGE(TAG, "Output pointers cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate means
    double sum_x = 0.0, sum_y = 0.0;
    for (uint8_t i = 0; i < num_points; i++) {
        sum_x += points[i].ppo2_bar;
        sum_y += points[i].sensor_mv;
    }
    double mean_x = sum_x / num_points;
    double mean_y = sum_y / num_points;
    
    ESP_LOGD(TAG, "Regression with %u points", num_points);
    
    // Calculate slope and intercept using least squares
    double sum_xy = 0.0, sum_xx = 0.0, sum_yy = 0.0;
    
    for (uint8_t i = 0; i < num_points; i++) {
        double dx = points[i].ppo2_bar - mean_x;
        double dy = points[i].sensor_mv - mean_y;
        
        sum_xy += dx * dy;
        sum_xx += dx * dx;
        sum_yy += dy * dy;
    }
    
    // Check for degenerate case (all X values the same)
    if (fabs(sum_xx) < 1e-12) {
        ESP_LOGE(TAG, "Degenerate regression: all PPO2 values are identical");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate regression coefficients
    // Linear model: V = m * PPO2 + b
    double m = sum_xy / sum_xx;  // sensitivity (mV/bar)
    double b = mean_y - m * mean_x;  // offset (mV)
    
    *sensitivity = m;
    *offset = b;
    
    ESP_LOGD(TAG, "Regression coefficients: m=%.4f mV/bar, b=%.4f mV", m, b);
    
    // Calculate correlation coefficient R²
    double ss_res = 0.0;  // Sum of squares of residuals
    double ss_tot = 0.0;  // Total sum of squares
    double max_abs_residual = 0.0;
    
    for (uint8_t i = 0; i < num_points; i++) {
        double predicted = m * points[i].ppo2_bar + b;
        double residual = points[i].sensor_mv - predicted;
        double abs_residual = fabs(residual);
        
        ss_res += residual * residual;
        ss_tot += (points[i].sensor_mv - mean_y) * (points[i].sensor_mv - mean_y);
        
        if (abs_residual > max_abs_residual) {
            max_abs_residual = abs_residual;
        }
        
        // Very verbose details disabled to save stack
    }
    
    // Calculate R² = 1 - (SS_res / SS_tot)
    double r_squared;
    if (fabs(ss_tot) < 1e-12) {
        // All Y values are identical - perfect fit if residuals are zero
        r_squared = (ss_res < 1e-12) ? 1.0 : 0.0;
    } else {
        r_squared = 1.0 - (ss_res / ss_tot);
    }
    
    // Clamp R² to [0, 1] range (numerical precision issues)
    if (r_squared < 0.0) r_squared = 0.0;
    if (r_squared > 1.0) r_squared = 1.0;
    
    *correlation_r2 = r_squared;
    *max_residual = max_abs_residual;
    
    // Use DEBUG level for detailed float results to reduce stack usage in main task
    ESP_LOGD(TAG, "Regression complete: m=%.3f mV/bar, b=%.3f mV, R2=%.5f, max_res=%.3f mV", 
             m, b, r_squared, max_abs_residual);
    
    return ESP_OK;
}

/**
 * @brief Run calibration quality gates
 * @param sensor_id Sensor ID (0 or 1)
 * @param cal Calibration to evaluate
 * @param passed_sensitivity Output: sensitivity check result
 * @param passed_offset Output: offset check result
 * @param passed_linearity Output: linearity check result
 * @param passed_repeatability Output: repeatability check result
 * @return ESP_OK on success
 */
esp_err_t run_quality_gates(uint8_t sensor_id, const multi_point_calibration_t *cal,
                           bool *passed_sensitivity, bool *passed_offset, 
                           bool *passed_linearity, bool *passed_repeatability)
{
    if (!cal || !passed_sensitivity || !passed_offset || !passed_linearity || !passed_repeatability) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get thresholds (would normally be from storage, using defaults for now)
    calibration_thresholds_t thresh = {
        .min_sensitivity_mv_per_bar = 45.0,
        .max_sensitivity_mv_per_bar = 80.0,
        .max_offset_magnitude_mv = 2.0,
        .min_correlation_r2 = 0.995,
        .max_residual_mv = 2.0,
        .repeatability_threshold_percent = 1.0
    };
    
    ESP_LOGD(TAG, "Quality gates S%u: sens=%.3f, offset=%.3f, R²=%.5f, res=%.3f", 
             sensor_id, cal->sensitivity_mv_per_bar, cal->offset_mv, cal->correlation_r2, cal->max_residual_mv);
    
    // 1. Sensitivity check
    *passed_sensitivity = (cal->sensitivity_mv_per_bar >= thresh.min_sensitivity_mv_per_bar) &&
                         (cal->sensitivity_mv_per_bar <= thresh.max_sensitivity_mv_per_bar);
    
    if (!*passed_sensitivity) {
        ESP_LOGW(TAG, "S%u SENSITIVITY FAIL: %.3f mV/bar (range: %.1f-%.1f)", 
                 sensor_id, cal->sensitivity_mv_per_bar, thresh.min_sensitivity_mv_per_bar, thresh.max_sensitivity_mv_per_bar);
    }
    
    // 2. Offset check
    *passed_offset = fabs(cal->offset_mv) <= thresh.max_offset_magnitude_mv;
    
    if (!*passed_offset) {
        ESP_LOGW(TAG, "S%u OFFSET FAIL: %.3f mV (max: ±%.1f mV)", 
                 sensor_id, cal->offset_mv, thresh.max_offset_magnitude_mv);
    }
    
    // 3. Linearity check
    *passed_linearity = (cal->correlation_r2 >= thresh.min_correlation_r2) &&
                       (cal->max_residual_mv <= thresh.max_residual_mv);
    
    if (!*passed_linearity) {
        ESP_LOGW(TAG, "S%u LINEARITY FAIL: R²=%.5f (min:%.3f), max_res=%.3f mV (max:%.1f)", 
                 sensor_id, cal->correlation_r2, thresh.min_correlation_r2, 
                 cal->max_residual_mv, thresh.max_residual_mv);
    }
    
    // 4. Repeatability check (for multi-point calibrations)
    *passed_repeatability = true;
    if (cal->num_points > 2) {
        // Check for repeated PPO2 values and their consistency
        for (uint8_t i = 0; i < cal->num_points; i++) {
            for (uint8_t j = i + 1; j < cal->num_points; j++) {
                double ppo2_diff = fabs(cal->points[i].ppo2_bar - cal->points[j].ppo2_bar);
                
                // If PPO2 values are very close (repeated measurement)
                if (ppo2_diff < 0.01) {  // Within 0.01 bar
                    double voltage_diff = fabs(cal->points[i].sensor_mv - cal->points[j].sensor_mv);
                    double avg_voltage = (cal->points[i].sensor_mv + cal->points[j].sensor_mv) / 2.0;
                    
                    // Check if voltage difference is within repeatability threshold
                    double percent_diff = (voltage_diff / avg_voltage) * 100.0;
                    
                    if (percent_diff > thresh.repeatability_threshold_percent) {
                        *passed_repeatability = false;
                        ESP_LOGW(TAG, "S%u REPEATABILITY FAIL: %.1f%% difference at PPO2=%.3f bar", 
                                 sensor_id, percent_diff, cal->points[i].ppo2_bar);
                        break;
                    }
                }
            }
            if (!*passed_repeatability) break;
        }
    }
    
    bool overall_pass = *passed_sensitivity && *passed_offset && *passed_linearity && *passed_repeatability;
    
    ESP_LOGI(TAG, "Quality gates S%u result: SENS=%s, OFFSET=%s, LINEAR=%s, REPEAT=%s => %s",
             sensor_id,
             *passed_sensitivity ? "PASS" : "FAIL",
             *passed_offset ? "PASS" : "FAIL", 
             *passed_linearity ? "PASS" : "FAIL",
             *passed_repeatability ? "PASS" : "FAIL",
             overall_pass ? "ACCEPTED" : "REJECTED");
    
    return ESP_OK;
}

/**
 * @brief Calculate normalized sensitivity and assess sensor health
 * @param sensor_id Sensor ID (0 or 1)
 * @param current_sensitivity Current calibration sensitivity (mV/bar)
 * @param baseline_sensitivity Baseline sensitivity when new (mV/bar)
 * @param current_offset Current offset (mV)
 * @param correlation_r2 Current linearity R²
 * @param max_residual Current maximum residual (mV)
 * @param health_info Output health assessment
 * @return ESP_OK on success
 */
esp_err_t assess_sensor_health(uint8_t sensor_id, double current_sensitivity, double baseline_sensitivity,
                              double current_offset, double correlation_r2, double max_residual,
                              sensor_health_info_t *health_info)
{
    if (!health_info) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Initialize health info
    memset(health_info, 0, sizeof(sensor_health_info_t));
    
    // Calculate normalized sensitivity
    if (baseline_sensitivity > 0.0) {
        health_info->normalized_sensitivity = current_sensitivity / baseline_sensitivity;
    } else {
        health_info->normalized_sensitivity = 1.0;  // Unknown baseline
        ESP_LOGW(TAG, "S%u: No baseline sensitivity available", sensor_id);
    }
    
    health_info->offset_drift_mv = current_offset;
    health_info->linearity_r2 = correlation_r2;
    health_info->max_residual_mv = max_residual;
    
    // Health thresholds (would normally be from storage)
    double caution_sens_thresh = 0.80;
    double fail_sens_thresh = 0.70;
    double caution_offset_thresh = 2.0;
    double fail_offset_thresh = 3.0;
    double min_r2 = 0.995;
    double max_res = 2.0;
    
    // Assess health status
    bool sens_caution = health_info->normalized_sensitivity < caution_sens_thresh;
    bool sens_fail = health_info->normalized_sensitivity < fail_sens_thresh;
    bool offset_caution = fabs(current_offset) > caution_offset_thresh;
    bool offset_fail = fabs(current_offset) > fail_offset_thresh;
    bool linearity_fail = (correlation_r2 < min_r2) || (max_residual > max_res);
    
    if (sens_fail || offset_fail || linearity_fail) {
        health_info->health_status = SENSOR_HEALTH_FAIL;
    } else if (sens_caution || offset_caution) {
        health_info->health_status = SENSOR_HEALTH_CAUTION;
    } else {
        health_info->health_status = SENSOR_HEALTH_GOOD;
    }
    
    ESP_LOGD(TAG, "S%u health: norm_sens=%.3f, offset=%.2fmV, R²=%.5f, status=%s", 
             sensor_id, health_info->normalized_sensitivity, current_offset, correlation_r2,
             sensor_calibration_health_string(health_info->health_status));
    
    return ESP_OK;
}
