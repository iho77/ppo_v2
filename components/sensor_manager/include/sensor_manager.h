/**
 * @file sensor_manager.h
 * @brief Simple sensor manager stub
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "app_types.h"
#include "sensor_calibration.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize sensor manager
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_manager_init(void);

/**
 * @brief Read current sensor data
 * @param data Pointer to sensor data structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_manager_read(sensor_data_t *data);

/**
 * @brief Update sensors (call from main loop)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_manager_update(void);

/**
 * @brief Check if sensors are ready
 * @return true if ready, false otherwise
 */
bool sensor_manager_is_ready(void);

/**
 * @brief Set O2 sensor calibration parameters
 * @param cal_data Calibration data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_manager_set_o2_calibration(const o2_calibration_t *cal_data);

/**
 * @brief Get current O2 sensor calibration parameters
 * @param cal_data Pointer to store calibration data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_manager_get_o2_calibration(o2_calibration_t *cal_data);

/**
 * @brief Perform O2 sensor calibration with known O2 percentage
 * @param known_o2_percent Known O2 percentage in the calibration gas (0-100%)
 * @param current_mv Current sensor reading in mV
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_manager_calibrate_o2(float known_o2_percent, float current_mv, int sensor_number);

/**
 * @brief Get raw (uncalibrated) O2 sensor reading
 * @param raw_mv Pointer to store raw voltage reading
 * @param sensor_number Sensor number to read (1 or 2)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_manager_get_raw_o2(float *raw_mv, int sensor_number);

/**
 * @brief Perform O2 sensor calibration with known O2 percentage
 * This will read the current sensor value and calibrate against the known O2%
 * @param known_o2_percent Known O2 percentage in calibration gas (0.1-100%)
 * @param sensor_number Sensor number to calibrate (1 or 2)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_manager_perform_o2_calibration(float known_o2_percent, int sensor_number);

/**
 * @brief Perform O2 calibration for both sensors simultaneously
 * Uses current readings from both sensors to perform calibration
 * @param known_o2_percent Known O2 percentage (10.0 to 100.0)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_manager_perform_dual_o2_calibration(float known_o2_percent);

/**
 * @brief Get calibration status including current sensor reading
 * @param current_raw_mv Pointer to store current raw reading
 * @param is_calibrated Pointer to store calibration status
 * @param calibration_offset Pointer to store current calibration offset
 * @param sensor_number Sensor number to check (1 or 2)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_manager_get_calibration_status(float *current_raw_mv, bool *is_calibrated, float *calibration_offset, int sensor_number);

/**
 * @brief Deinitialize sensor manager
 */
void sensor_manager_deinit(void);

/**
 * @brief Enable test mode for sensor failure simulation
 * Only active if SENSOR_FAILURE_TEST is defined at compile time
 * Use this to test different failure modes during development
 */
void sensor_manager_enable_test_mode(void);

/**
 * @brief Perform advanced two-point calibration with health assessment
 * @param sensor_id Sensor ID (0 or 1)
 * @param gas1_o2_fraction O2 fraction of first gas (0.0-1.0)
 * @param gas1_pressure_bar Pressure during first gas measurement (bar)
 * @param gas2_o2_fraction O2 fraction of second gas (0.0-1.0)
 * @param gas2_pressure_bar Pressure during second gas measurement (bar)
 * @param result Output two-point calibration result
 * @return ESP_OK on success
 */
esp_err_t sensor_manager_advanced_calibration(uint8_t sensor_id,
                                              float gas1_o2_fraction, float gas1_pressure_bar,
                                              float gas2_o2_fraction, float gas2_pressure_bar,
                                              two_point_calibration_t *result);

/**
 * @brief Start multi-point calibration session
 * @param sensor_id Sensor ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t sensor_manager_start_multipoint_calibration(uint8_t sensor_id);

/**
 * @brief Add calibration point to current session
 * @param sensor_id Sensor ID (0 or 1)
 * @param o2_fraction Known O2 fraction (0.0-1.0)
 * @param pressure_bar Ambient pressure (bar)
 * @return ESP_OK on success
 */
esp_err_t sensor_manager_add_calibration_point(uint8_t sensor_id, 
                                               float o2_fraction, 
                                               float pressure_bar);

/**
 * @brief Complete multi-point calibration session
 * @param sensor_id Sensor ID (0 or 1)
 * @param result Output calibration result
 * @return ESP_OK on success
 */
esp_err_t sensor_manager_finalize_multipoint_calibration(uint8_t sensor_id,
                                                         multi_point_calibration_t *result);

/**
 * @brief Get sensor health assessment
 * @param sensor_id Sensor ID (0 or 1)
 * @param health_info Output health information
 * @return ESP_OK on success
 */
esp_err_t sensor_manager_get_health_status(uint8_t sensor_id, sensor_health_info_t *health_info);

/**
 * @brief Initialize new sensor baseline
 * @param sensor_id Sensor ID (0 or 1)
 * @param sensor_serial Sensor serial number or identifier
 * @return ESP_OK on success
 */
esp_err_t sensor_manager_initialize_sensor_baseline(uint8_t sensor_id, const char *sensor_serial);

/**
 * @brief Get calibration history
 * @param sensor_id Sensor ID (0 or 1)
 * @param entries Output array for log entries
 * @param max_entries Maximum entries to retrieve
 * @param num_entries Number of entries retrieved
 * @return ESP_OK on success
 */
esp_err_t sensor_manager_get_calibration_history(uint8_t sensor_id,
                                                 calibration_log_entry_t *entries,
                                                 uint8_t max_entries,
                                                 uint8_t *num_entries);

/**
 * @brief Print calibration and health summary
 * @param sensor_id Sensor ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t sensor_manager_print_sensor_summary(uint8_t sensor_id);

/**
 * @brief Print calibration log in CSV format via ESP_LOGI
 * @param sensor_id Sensor ID (0 or 1), or 255 for both sensors
 * @param max_entries Maximum entries to print (0 = all available)
 * @return ESP_OK on success
 */
esp_err_t sensor_manager_print_csv_log(uint8_t sensor_id, uint8_t max_entries);

/**
 * @brief Reset calibration log with predefined baseline calibration
 * Creates a single 2-point calibration entry for each sensor:
 * - Point 1: 21% O2 at 1.013 bar (air) with specified mV reading
 * - Point 2: 0% O2 at 1.013 bar (nitrogen) with 0 mV reading
 * This is useful for factory reset or testing scenarios.
 * @param sensor0_air_mv Sensor 0 voltage reading for air (mV, typically ~10mV)
 * @param sensor1_air_mv Sensor 1 voltage reading for air (mV, typically ~10mV)
 * @param sensor0_serial Sensor 0 serial number/identifier
 * @param sensor1_serial Sensor 1 serial number/identifier
 * @return ESP_OK on success
 */
esp_err_t sensor_manager_reset_calibration_log(float sensor0_air_mv, float sensor1_air_mv,
                                               const char *sensor0_serial, const char *sensor1_serial);

/**
 * @brief Get calibration status string for display
 * @param sensor_id Sensor ID (0 or 1)
 * @return Calibration status string: "none", "1 pt", "2 pt"
 */
const char* sensor_manager_get_calibration_display_status(uint8_t sensor_id);

/**
 * @brief Reset calibration data for a single sensor
 * Clears all calibration and health monitoring data for the specified sensor.
 * Automatically assigns new sensor serial number (s1XX for sensor 0, s2XX for sensor 1).
 * Used when replacing individual sensors.
 * @param sensor_id Sensor ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t sensor_manager_reset_sensor(uint8_t sensor_id);

/**
 * @brief Check if system is in single sensor configuration
 * Returns true if one sensor channel is disabled (ADC reading < 6mV at startup)
 * @return true if single sensor mode, false if dual sensor mode
 */
bool sensor_manager_is_single_sensor_mode(void);

/**
 * @brief Get the active sensor ID in single sensor mode
 * @return 0 or 1 for active sensor, -1 if in dual sensor mode
 */
int sensor_manager_get_active_sensor_id(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_MANAGER_H