/**
 * @file button_manager.h
 * @brief Simple polling-based button manager
 */

#ifndef BUTTON_MANAGER_H
#define BUTTON_MANAGER_H

#include "app_types.h"
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Button manager configuration
 */
typedef struct {
    gpio_num_t mode_gpio;     // GPIO pin for mode button
    gpio_num_t select_gpio;   // GPIO pin for select button
} button_config_t;

/**
 * @brief Initialize button manager
 * @param config Button configuration with GPIO pins
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t button_manager_init(const button_config_t *config);

/**
 * @brief Get button event (polling-based)
 * @param button_id Which button to check
 * @return Button event or BUTTON_EVENT_NONE
 */
button_event_t button_manager_get_event(button_id_t button_id);

/**
 * @brief Update button states (call from main loop)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t button_manager_update(void);

/**
 * @brief Get current button state
 * @param button_id Button to check
 * @return true if pressed, false if released
 */
bool button_manager_is_pressed(button_id_t button_id);

/**
 * @brief Deinitialize button manager
 */
void button_manager_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // BUTTON_MANAGER_H