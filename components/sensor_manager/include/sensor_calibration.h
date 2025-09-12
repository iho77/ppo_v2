/**
 * @file sensor_calibration.h
 * @brief Advanced calibration system for dual O2 sensors with health monitoring
 * @version 1.0
 * @date 2025-01-27
 * 
 * Note: Uses persistent counters instead of timestamps for unique IDs
 * since device has no RTC and timestamps reset on power cycles.
 */

#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Constants
#define MAX_CALIBRATION_POINTS      6       // Maximum points for multi-point calibration
#define MAX_CALIBRATION_HISTORY     200     // Keep last 200 calibrations per sensor
#define SENSOR_BASELINE_KEY_SIZE    32      // Size of sensor baseline key
#define NUM_O2_SENSORS              2       // Number of O2 sensors
#define CALIBRATION_UUID_SIZE       16      // UUID size in bytes

/**
 * @brief Calibration point for multi-point calibration
 */
typedef struct {
    double ppo2_bar;                        // Known PPO2 value (bar)
    float sensor_mv;                        // Sensor reading (mV)
    float pressure_bar;                     // Ambient pressure during calibration
    float temperature_c;                    // Temperature during calibration (optional)
    uint32_t uptime_ms;                     // System uptime when calibrated (for ordering)
} calibration_point_t;

/**
 * @brief Two-point calibration parameters
 */
typedef struct {
    // Calibration points
    calibration_point_t point1;            // First calibration point (typically air)
    calibration_point_t point2;            // Second calibration point (typically O2 or N2)
    
    // Linear model parameters: V = m * PPO2 + b
    double sensitivity_mv_per_bar;          // Sensitivity m (mV/bar)
    double offset_mv;                       // Zero offset b (mV)
    double correlation_r2;                  // Correlation coefficient R²
    double max_residual_mv;                 // Maximum absolute residual (mV)
    
    // Calibration metadata
    uint32_t calibration_id;                // Persistent calibration counter
    uint32_t uptime_ms;                     // System uptime when calibrated
    bool valid;                             // Calibration validity
} two_point_calibration_t;

/**
 * @brief Multi-point calibration data
 */
typedef struct {
    uint8_t num_points;                     // Number of calibration points (2-6)
    calibration_point_t points[MAX_CALIBRATION_POINTS];  // Calibration points
    
    // Linear regression results: V = m * PPO2 + b
    double sensitivity_mv_per_bar;          // Sensitivity m (mV/bar)
    double offset_mv;                       // Zero offset b (mV)
    double correlation_r2;                  // Correlation coefficient R²
    double max_residual_mv;                 // Maximum absolute residual (mV)
    
    // Calibration metadata
    uint32_t calibration_id;                // Persistent calibration counter
    uint32_t uptime_ms;                     // System uptime when calibrated
    bool valid;                             // Calibration validity
} multi_point_calibration_t;

/**
 * @brief Sensor health classification
 */
typedef enum {
    SENSOR_HEALTH_GOOD = 0,                 // Normal operation
    SENSOR_HEALTH_CAUTION,                  // Performance degraded but usable
    SENSOR_HEALTH_FAIL,                     // End of life, replace immediately
    SENSOR_HEALTH_UNKNOWN                   // Insufficient data
} sensor_health_t;

/**
 * @brief Sensor baseline parameters (stored for each unique sensor)
 */
typedef struct {
    char sensor_key[SENSOR_BASELINE_KEY_SIZE]; // Unique sensor identifier (serial/batch)
    double baseline_sensitivity;            // Original sensitivity when new (mV/bar)
    uint32_t install_calibration_id;        // Calibration ID when sensor was first installed
    uint32_t total_calibrations;            // Total number of calibrations performed
    uint32_t power_cycles;                  // Number of power cycles since install
    bool valid;                             // Baseline validity
} sensor_baseline_t;

/**
 * @brief Sensor health parameters with trend analysis
 */
typedef struct {
    // Current health metrics
    double normalized_sensitivity;          // S = k/k0 (current/baseline)
    double offset_drift_mv;                 // Current offset (mV)
    double linearity_r2;                    // Linearity quality
    double max_residual_mv;                 // Maximum calibration error
    
    // Trend analysis (based on calibration sequence, not time)
    double degradation_rate_per_calibration; // Sensitivity loss per calibration cycle
    uint32_t predicted_eol_calibrations;    // Calibrations until EOL (S < 0.70)
    bool trend_valid;                       // Sufficient data for trend analysis
    
    // Health classification
    sensor_health_t health_status;          // Current health status
    uint32_t last_assessment_cal_id;        // Calibration ID of last assessment
} sensor_health_info_t;

/**
 * @brief Single calibration log entry
 */
typedef struct {
    uint32_t calibration_id;                // Unique calibration ID (persistent counter)
    uint8_t sensor_id;                      // Sensor ID (0 or 1)
    uint32_t power_cycle_count;             // Power cycle count when calibrated
    uint32_t uptime_ms;                     // System uptime when calibrated
    
    // Calibration data
    uint8_t num_points;                     // Number of calibration points (1-6)
    calibration_point_t points[MAX_CALIBRATION_POINTS];  // Calibration points
    
    // Calculated parameters
    double sensitivity_mv_per_bar;          // Calculated sensitivity
    double offset_mv;                       // Calculated offset
    double correlation_r2;                  // Correlation coefficient
    double max_residual_mv;                 // Maximum residual error
    
    // Health metrics
    double normalized_sensitivity;          // S = k/k0
    sensor_health_info_t health_info;       // Health assessment
    
    // Quality gates results
    bool passed_sensitivity_check;          // Sensitivity within range
    bool passed_offset_check;               // Offset within range
    bool passed_linearity_check;            // Linearity acceptable
    bool passed_repeatability_check;        // Repeatability acceptable
    bool calibration_accepted;              // Overall calibration result
    
    // Metadata
    float temperature_c;                    // Temperature during calibration (if available)
    char notes[64];                         // Optional calibration notes
} calibration_log_entry_t;

/**
 * @brief Calibration thresholds (tunable)
 */
typedef struct {
    // Sensitivity thresholds (mV/bar at 25°C for new sensor)
    double min_sensitivity_mv_per_bar;      // Minimum acceptable (default 45)
    double max_sensitivity_mv_per_bar;      // Maximum acceptable (default 75)
    
    // Offset thresholds
    double max_offset_magnitude_mv;         // Maximum |offset| magnitude (default 2.0)
    
    // Linearity thresholds
    double min_correlation_r2;              // Minimum R² for linearity (default 0.995)
    double max_residual_mv;                 // Maximum residual error (default 2.0)
    
    // Health thresholds
    double caution_sensitivity_threshold;   // S < this = caution (default 0.80)
    double fail_sensitivity_threshold;      // S < this = fail (default 0.70)
    double caution_offset_threshold_mv;     // |b| > this = caution (default 2.0)
    double fail_offset_threshold_mv;        // |b| > this = fail (default 3.0)
    
    // Repeatability threshold
    double repeatability_threshold_percent; // Maximum difference between repeated readings (default 1.0)
} calibration_thresholds_t;

/**
 * @brief Persistent calibration storage structure
 * 
 * Note: This structure will be large (~300KB for 600 entries * 2 sensors)
 * Consider using circular buffer or NVS blob storage for efficiency
 */
typedef struct {
    // Global counters
    uint32_t next_calibration_id;           // Next calibration ID to assign
    uint32_t power_cycle_count;             // Total power cycles
    uint32_t sensor_install_counters[NUM_O2_SENSORS]; // Per-position installation counter
    
    // Per-sensor data
    sensor_baseline_t baselines[NUM_O2_SENSORS];       // Sensor baselines (when new)
    calibration_log_entry_t previous[NUM_O2_SENSORS];  // Previous calibration (for comparison)
    uint32_t total_calibrations[NUM_O2_SENSORS];       // Total calibration count per sensor
    
    // Simple degradation tracking (no full history needed)
    double avg_sensitivity_trend[NUM_O2_SENSORS];      // Running average of sensitivity degradation
    double avg_offset_drift[NUM_O2_SENSORS];           // Running average of offset drift
    
    // Current active calibration
    multi_point_calibration_t current[NUM_O2_SENSORS]; // Current calibration parameters
    
    // Configuration
    calibration_thresholds_t thresholds;    // Calibration quality thresholds
    
    // Integrity
    uint32_t checksum;                      // Data integrity checksum
} calibration_storage_t;

/**
 * @brief Initialize calibration system
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_init(void);

/**
 * @brief Perform two-point calibration
 * @param sensor_id Sensor ID (0 or 1)
 * @param point1 First calibration point
 * @param point2 Second calibration point
 * @param result Output calibration result
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_two_point(uint8_t sensor_id, 
                                       const calibration_point_t *point1,
                                       const calibration_point_t *point2,
                                       two_point_calibration_t *result);

/**
 * @brief Start new multi-point calibration session
 * @param sensor_id Sensor ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_start_session(uint8_t sensor_id);

/**
 * @brief Add point to multi-point calibration session
 * @param sensor_id Sensor ID (0 or 1)
 * @param point Calibration point to add
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_add_point(uint8_t sensor_id, const calibration_point_t *point);

/**
 * @brief Complete multi-point calibration and compute parameters
 * @param sensor_id Sensor ID (0 or 1)
 * @param result Output calibration result
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_finalize(uint8_t sensor_id, multi_point_calibration_t *result);

/**
 * @brief Convert sensor voltage to PPO2 using current calibration
 * @param sensor_id Sensor ID (0 or 1)
 * @param voltage_mv Sensor voltage reading (mV)
 * @param ppo2_bar Output PPO2 value (bar)
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_voltage_to_ppo2(uint8_t sensor_id, float voltage_mv, double *ppo2_bar);

/**
 * @brief Assess sensor health based on calibration history
 * @param sensor_id Sensor ID (0 or 1)
 * @param health_info Output health assessment
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_assess_health(uint8_t sensor_id, sensor_health_info_t *health_info);

/**
 * @brief Set sensor baseline parameters (for new sensors)
 * @param sensor_id Sensor ID (0 or 1)
 * @param sensor_key Unique sensor identifier (serial number, batch code, etc.)
 * @param baseline_sensitivity Initial sensitivity when new
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_set_baseline(uint8_t sensor_id, 
                                          const char *sensor_key,
                                          double baseline_sensitivity);

/**
 * @brief Detect if a sensor has been replaced (different from baseline key)
 * @param sensor_id Sensor ID (0 or 1)
 * @param current_key Current sensor key to check
 * @param is_new_sensor Output: true if sensor appears to be new/replaced
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_detect_replacement(uint8_t sensor_id,
                                               const char *current_key,
                                               bool *is_new_sensor);

/**
 * @brief Get calibration history for sensor
 * @param sensor_id Sensor ID (0 or 1)
 * @param entries Output array of log entries
 * @param max_entries Maximum entries to retrieve
 * @param num_entries Number of entries retrieved
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_get_history(uint8_t sensor_id,
                                         calibration_log_entry_t *entries,
                                         uint8_t max_entries,
                                         uint8_t *num_entries);

/**
 * @brief Save calibration data to persistent storage (NVS)
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_save(void);

/**
 * @brief Load calibration data from persistent storage (NVS)
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_load(void);

/**
 * @brief Get current calibration info for a sensor
 * @param sensor_id Sensor ID (0 or 1)
 * @param calibration Output current calibration info
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_get_current(uint8_t sensor_id, multi_point_calibration_t *calibration);

/**
 * @brief Get current calibration thresholds
 * @param thresholds Output thresholds structure
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_get_thresholds(calibration_thresholds_t *thresholds);

/**
 * @brief Set calibration thresholds
 * @param thresholds Input thresholds structure
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_set_thresholds(const calibration_thresholds_t *thresholds);

/**
 * @brief Clear calibration history for sensor (keep baseline)
 * @param sensor_id Sensor ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_clear_history(uint8_t sensor_id);

/**
 * @brief Reset sensor baseline (for sensor replacement)
 * @param sensor_id Sensor ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_reset_baseline(uint8_t sensor_id);

/**
 * @brief Get human-readable health status string
 * @param health Health status enum
 * @return Status string
 */
const char* sensor_calibration_health_string(sensor_health_t health);

/**
 * @brief Print calibration summary for debugging
 * @param sensor_id Sensor ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_print_summary(uint8_t sensor_id);

/**
 * @brief Print calibration log in CSV format via ESP_LOGI
 * @param sensor_id Sensor ID (0 or 1), or 255 for both sensors
 * @param max_entries Maximum entries to print (0 = all available)
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_print_csv_log(uint8_t sensor_id, uint8_t max_entries);

/**
 * @brief Reset calibration log with predefined baseline calibration
 * Creates a single 2-point calibration entry for each sensor:
 * - Point 1: 21% O2 at 1.013 bar (air) with specified mV reading
 * - Point 2: 0% O2 at 1.013 bar (nitrogen) with 0 mV reading
 * @param sensor0_air_mv Sensor 0 voltage reading for air (mV, typically ~10mV)
 * @param sensor1_air_mv Sensor 1 voltage reading for air (mV, typically ~10mV)
 * @param sensor0_serial Sensor 0 serial number/identifier
 * @param sensor1_serial Sensor 1 serial number/identifier
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_reset_log_with_baseline(float sensor0_air_mv, float sensor1_air_mv,
                                                     const char *sensor0_serial, const char *sensor1_serial);

/**
 * @brief Reset calibration data for a single sensor
 * Clears all calibration and health monitoring data for the specified sensor.
 * Automatically assigns new sensor serial number (s1XX for sensor 0, s2XX for sensor 1).
 * Used when replacing individual sensors.
 * @param sensor_id Sensor ID (0 or 1) 
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_reset_sensor(uint8_t sensor_id);

/**
 * @brief Get current power cycle count
 * @return Power cycle count
 */
uint32_t sensor_calibration_get_power_cycles(void);

/**
 * @brief Increment power cycle counter (call at startup)
 * @return ESP_OK on success
 */
esp_err_t sensor_calibration_increment_power_cycle(void);

/**
 * @brief Deinitialize calibration system
 */
void sensor_calibration_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_CALIBRATION_H