/**
 * @file warning_manager.h
 * @brief Warning system using ESP32-C3 built-in RGB LED
 */

#ifndef WARNING_MANAGER_H
#define WARNING_MANAGER_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Warning states for PPO2 monitoring
 */
typedef enum {
    WARNING_STATE_NORMAL = 0,     // Green LED - PPO2 in normal range
    WARNING_STATE_WARNING,        // Yellow blinking LED - PPO2 in warning range
    WARNING_STATE_ALARM           // Red blinking LED - PPO2 in alarm range
} warning_state_t;

/**
 * @brief Warning manager configuration
 */
typedef struct {
    gpio_num_t led_gpio;          // GPIO pin for LED strip
    bool use_led_strip;           // true for addressable LED, false for simple GPIO
} warning_config_t;

/**
 * @brief Initialize warning manager
 * @param config Warning system configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t warning_manager_init(const warning_config_t *config);

/**
 * @brief Update warning system based on sensor data
 * @param sensor_data Current sensor readings
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t warning_manager_update(const sensor_data_t *sensor_data);

/**
 * @brief Get current warning state
 * @return Current warning state
 */
warning_state_t warning_manager_get_state(void);

/**
 * @brief Get current warning display message
 * @return Warning message string, or NULL if no warning message active
 */
const char* warning_manager_get_display_message(void);

/**
 * @brief Test warning system (cycle through all states)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t warning_manager_test(void);

/**
 * @brief Deinitialize warning manager
 */
void warning_manager_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // WARNING_MANAGER_H