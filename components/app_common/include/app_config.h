/**
 * @file app_config.h
 * @brief Configuration constants for ESP32-C3 sensor monitoring system
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "app_types.h"
#include "esp_err.h"

// Hardware pin definitions (moved to component-specific files)
// GPIO assignments are now defined where they are actually used

// Only keep constants that are actually used across multiple components
// Component-specific constants have been moved to their respective files

// Communication timeouts (used by multiple I2C components)
#define I2C_TIMEOUT_MS              500

// PPO2 alarm and warning default limits (bar)
#define PPO2_DEFAULT_LOW_WARNING    0.19f  // Low PPO2 warning at 0.16 bar
#define PPO2_DEFAULT_LOW_ALARM      0.18f  // Low PPO2 alarm at 0.10 bar  
#define PPO2_DEFAULT_HIGH_WARNING   1.4f  // High PPO2 warning at 1.40 bar
#define PPO2_DEFAULT_HIGH_ALARM     1.6f  // High PPO2 alarm at 1.60 bar

// Display default settings
#define DISPLAY_DEFAULT_BRIGHTNESS  80     // 80% brightness
#define DISPLAY_DEFAULT_CONTRAST    50     // 50% contrast
#define DISPLAY_DEFAULT_AUTO_BRIGHT true   // Auto brightness enabled

// System default settings
#define SYSTEM_DEFAULT_SLEEP_TIMEOUT_S  300  // 5 minutes sleep timeout

// Legacy O2 calibration constants removed - using simplified calibration system

// NVS namespace and keys
#define NVS_NAMESPACE_CONFIG        "config"

// PPO2 limit keys
#define NVS_KEY_PPO2_LOW_WARN       "ppo2_low_warn"
#define NVS_KEY_PPO2_LOW_ALARM      "ppo2_low_alarm"
#define NVS_KEY_PPO2_HIGH_WARN      "ppo2_high_warn"
#define NVS_KEY_PPO2_HIGH_ALARM     "ppo2_hi_alarm"

// Display setting keys
#define NVS_KEY_BRIGHTNESS          "brightness"
#define NVS_KEY_CONTRAST            "contrast"
#define NVS_KEY_AUTO_BRIGHTNESS     "auto_bright"

// System setting keys
#define NVS_KEY_SLEEP_TIMEOUT       "sleep_timeout"

// O2 calibration validity keys
#define NVS_KEY_O2_CAL_VALID        "o2_cal_valid"   // Sensor 1 calibration validity
#define NVS_KEY_O2_CAL_VALID_S2     "o2_cal_val_s2" // Sensor 2 calibration validity

// Date/time keys
#define NVS_KEY_DATETIME            "datetime"

// Duplicate removed - already defined above as SYSTEM_DEFAULT_SLEEP_TIMEOUT_S

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize configuration system and NVS storage
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_config_init(void);

/**
 * @brief Load application configuration from NVS
 * @param config Pointer to configuration structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_config_load(app_config_t *config);

/**
 * @brief Save application configuration to NVS
 * @param config Pointer to configuration structure to save
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_config_save(const app_config_t *config);

/**
 * @brief Get default configuration values
 * @param config Pointer to configuration structure to fill with defaults
 */
void app_config_get_defaults(app_config_t *config);

/**
 * @brief Get pointer to current configuration (loads once, cached)
 * @return Pointer to current configuration structure
 */
const app_config_t* app_config_get_current(void);

#ifdef __cplusplus
}
#endif

#endif // APP_CONFIG_H