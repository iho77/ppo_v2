/**
 * @file display_manager.h
 * @brief SH1107 128x128 OLED display manager interface (I2C)
 */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include "app_types.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Display configuration structure
 */
typedef struct {
    i2c_master_bus_handle_t i2c_bus;  // I2C bus handle from new master driver
    int i2c_address;                   // I2C address (0x3C or 0x3D)
    uint32_t i2c_freq_hz;             // I2C frequency in Hz
} display_config_t;

/**
 * @brief Initialize display manager with I2C configuration
 * @param config Display configuration with I2C port and pins
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_manager_init(const display_config_t *config);


/**
 * @brief Update display with main diving mode using sensor data
 * @param sensor_data Sensor data structure with PPO2, pressure, etc.
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_manager_update_main(const sensor_data_t *sensor_data);

/**
 * @brief Update display with menu overlay showing 7 items vertically centered
 * @param selected_item Index of currently selected menu item (0-6)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_manager_update_menu(uint8_t selected_item);

/**
 * @brief Update display with system message (multi-line text with back button)
 * @param message Multi-line message text to display
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_manager_update_system_message(const char* message);

/**
 * @brief Update display with reset calibration screen
 * @param sensor1_mv Sensor 1 voltage reading in mV
 * @param sensor2_mv Sensor 2 voltage reading in mV
 * @param selected_item Index of selected reset option (0=reset all, 1=reset S1, 2=reset S2, 3=back)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_manager_update_reset_calibration(float sensor1_mv, float sensor2_mv, uint8_t selected_item);


/**
 * @brief Update display with dual sensor calibration screen
 * @param sensor1_mv Sensor 1 raw voltage reading in mV
 * @param sensor1_ppo2 Sensor 1 PPO2 value
 * @param sensor2_mv Sensor 2 raw voltage reading in mV  
 * @param sensor2_ppo2 Sensor 2 PPO2 value
 * @param selected_item Index of selected option (gas selection: 0-2, action mode: 0-1)
 * @param selected_gas Selected gas type (1=Air, 2=O2, 3=Custom)
 * @param custom_o2_percent Custom O2 percentage value (for Custom option)
 * @param custom_editing Whether custom editing is active
 * @param in_action_mode True if in action mode (calibrate/back), false if in gas selection mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_manager_update_dual_calibration(float sensor1_mv, float sensor1_ppo2, 
                                                 float sensor2_mv, float sensor2_ppo2,
                                                 uint8_t selected_item, uint8_t selected_gas,
                                                 float custom_o2_percent, bool custom_editing, bool in_action_mode,
                                                 bool calibration_session_active);

/**
 * @brief Clear display
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_manager_clear(void);

/**
 * @brief Set display brightness/contrast
 * @param brightness Brightness level (0-100)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_manager_set_brightness(uint8_t brightness);


/**
 * @brief Get display dimensions
 * @param width Pointer to store width
 * @param height Pointer to store height
 */
void display_manager_get_dimensions(uint8_t *width, uint8_t *height);

/**
 * @brief Put display in sleep mode
 * @param sleep true to sleep, false to wake
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_manager_sleep(bool sleep);

/**
 * @brief Deinitialize display manager
 */
void display_manager_deinit(void);


#ifdef __cplusplus
}
#endif

#endif // DISPLAY_MANAGER_H