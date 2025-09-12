/**
 * @file ppo2_logger.h
 * @brief PPO2 measurement logging system for dive analysis and troubleshooting
 * @version 1.0
 * @date 2025-01-27
 * 
 * Provides automatic logging of PPO2 measurements from both sensors every minute
 * with persistent storage in NVS flash. Designed for 3-hour logging capacity
 * with minimal impact on system performance and flash health.
 */

#ifndef PPO2_LOGGER_H
#define PPO2_LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Configuration constants
#define PPO2_LOG_MAX_ENTRIES        180     // 3 hours at 1/minute
#define PPO2_LOG_INTERVAL_MS        60000   // 1 minute interval
#define PPO2_LOG_NVS_NAMESPACE      "ppo2_log"
#define PPO2_LOG_NVS_KEY            "log_data"
#define PPO2_LOG_WRITE_INTERVAL_MS  300000  // Write to NVS every 5 minutes

/**
 * @brief Single PPO2 log entry (16 bytes)
 * Optimized for memory efficiency while maintaining data integrity
 */
typedef struct {
    uint8_t sensor_id;              // Sensor ID (0 or 1)
    uint8_t reserved;               // Padding for alignment
    uint16_t timestamp_minutes;     // Minutes since boot (max 45.5 days)
    uint32_t calibration_id;        // Current calibration ID
    double ppo2_bar;                // PPO2 reading in bar
} __attribute__((packed)) ppo2_log_entry_t;

/**
 * @brief Log statistics for display and analysis
 */
typedef struct {
    uint16_t total_entries;         // Number of logged entries
    uint16_t minutes_logged;        // Total minutes of data available
    uint32_t oldest_timestamp;      // Oldest entry timestamp (minutes)
    uint32_t newest_timestamp;      // Newest entry timestamp (minutes)
    uint8_t sensors_active;         // Bitmask of active sensors (bit 0=S1, bit 1=S2)
    uint32_t boot_counter;          // Current boot cycle number
    uint32_t total_entries_ever;    // Total entries since first installation
} ppo2_log_stats_t;

// Public API functions

/**
 * @brief Initialize PPO2 logger system
 * 
 * Allocates memory, opens NVS storage, loads existing log data,
 * and prepares timers. Must be called before any other logger functions.
 * 
 * @return ESP_OK on success
 *         ESP_ERR_NO_MEM if memory allocation fails
 *         ESP_ERR_NVS_* for NVS errors
 */
esp_err_t ppo2_logger_init(void);

/**
 * @brief Start automatic PPO2 logging
 * 
 * Begins periodic logging every minute. Must be called after both
 * ppo2_logger_init() and sensor_manager_init() have completed.
 * 
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t ppo2_logger_start(void);

/**
 * @brief Stop automatic PPO2 logging
 * 
 * Stops periodic logging and saves any pending data to NVS.
 * Logger can be restarted with ppo2_logger_start().
 * 
 * @return ESP_OK on success
 */
esp_err_t ppo2_logger_stop(void);

/**
 * @brief Manually add a PPO2 log entry
 * 
 * For testing, calibration logging, or special measurement events.
 * Entry is added immediately to the circular buffer.
 * 
 * @param sensor_id Sensor ID (0 or 1)
 * @param ppo2_bar PPO2 reading in bar
 * @param calibration_id Current calibration ID
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_ARG for invalid sensor_id
 *         ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t ppo2_logger_add_entry(uint8_t sensor_id, double ppo2_bar, uint32_t calibration_id);

/**
 * @brief Get comprehensive log statistics
 * 
 * Provides summary information about the current log state,
 * useful for display in menus and system diagnostics.
 * 
 * @param stats Output statistics structure
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_ARG if stats is NULL
 *         ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t ppo2_logger_get_stats(ppo2_log_stats_t *stats);

/**
 * @brief Get log entries for display/export
 * 
 * Retrieves entries from the circular buffer in chronological order.
 * Supports pagination for large datasets.
 * 
 * @param entries Output buffer for entries
 * @param max_entries Maximum entries to retrieve
 * @param num_entries Number of entries actually retrieved
 * @param start_index Starting index (0 = oldest, use for pagination)
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_ARG for invalid parameters
 *         ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t ppo2_logger_get_entries(ppo2_log_entry_t *entries, 
                                 uint16_t max_entries, 
                                 uint16_t *num_entries,
                                 uint16_t start_index);

/**
 * @brief Print all log entries in CSV format
 * 
 * Outputs complete log to ESP_LOGI for easy capture via serial monitor.
 * Format: SensorID,TimestampMin,CalibrationID,PPO2_bar
 * Useful for dive analysis and data export.
 * 
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t ppo2_logger_print_csv(void);

/**
 * @brief Print log entries in human-readable format
 * 
 * Outputs formatted log suitable for debugging and quick analysis.
 * Shows timestamp, sensor, PPO2 value, and calibration info.
 * 
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t ppo2_logger_print_formatted(void);

/**
 * @brief Clear all log entries
 * 
 * Resets the circular buffer and clears NVS storage.
 * Boot counter and total entries counter are preserved.
 * 
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t ppo2_logger_reset(void);

/**
 * @brief Force immediate save to NVS
 * 
 * Normally saves occur automatically every 5 minutes.
 * Use this for immediate persistence before shutdown or critical events.
 * 
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_STATE if not initialized
 *         ESP_ERR_NVS_* for NVS errors
 */
esp_err_t ppo2_logger_save_now(void);

/**
 * @brief Get logger running status
 * 
 * @return true if automatic logging is active
 *         false if stopped or not initialized
 */
bool ppo2_logger_is_running(void);

/**
 * @brief Get initialization status
 * 
 * @return true if logger has been successfully initialized
 *         false if not initialized or initialization failed
 */
bool ppo2_logger_is_initialized(void);

/**
 * @brief Deinitialize PPO2 logger
 * 
 * Stops logging, saves pending data, releases memory, and closes NVS.
 * Logger must be reinitialized before use after calling this function.
 */
void ppo2_logger_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // PPO2_LOGGER_H