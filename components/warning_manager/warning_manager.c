/**
 * @file warning_manager.c
 * @brief Warning system implementation using ESP32-C3 RGB LED
 */

#include "sdkconfig.h"
#include "warning_manager.h"
#include "sensor_manager.h"
#include "app_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include <math.h>

static const char *TAG = "WARNING_MGR";

// Warning manager state
static bool s_initialized = false;
static warning_config_t s_config = {0};
static warning_state_t s_current_state = WARNING_STATE_NORMAL;
static led_strip_handle_t s_led_strip = NULL;

// Timing control
static uint32_t s_last_blink_time = 0;
static bool s_led_on = false;

// Warning display message
static char s_warning_message[64] = {0};

// Warning thresholds (loaded from config)
static float s_ppo2_low_warning = 0.16f;
static float s_ppo2_low_alarm = 0.10f;
static float s_ppo2_high_warning = 1.40f;
static float s_ppo2_high_alarm = 1.60f;

// Sensor disagreement threshold is now defined in app_types.h as PPO2_DISAGREEMENT_THRESHOLD

// LED colors (RGB values 0-255)
#define LED_COLOR_GREEN_R    0
#define LED_COLOR_GREEN_G    100
#define LED_COLOR_GREEN_B    0

#define LED_COLOR_YELLOW_R   100
#define LED_COLOR_YELLOW_G   100
#define LED_COLOR_YELLOW_B   0

#define LED_COLOR_RED_R      100
#define LED_COLOR_RED_G      0
#define LED_COLOR_RED_B      0

#define LED_COLOR_OFF_R      0
#define LED_COLOR_OFF_G      0
#define LED_COLOR_OFF_B      0

// Blink timing (milliseconds)  
#define BLINK_PERIOD_WARNING_MS  500   // 2Hz (500ms period, 250ms on/off)
#define BLINK_PERIOD_ALARM_MS    250   // 4Hz (250ms period, 125ms on/off)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)


static esp_err_t set_led_color(uint8_t red, uint8_t green, uint8_t blue);
static warning_state_t calculate_warning_state(int32_t ppo2);
static warning_state_t calculate_dual_sensor_warning_state(const sensor_data_t *sensor_data);
static void update_led_based_on_state(void);

esp_err_t warning_manager_init(const warning_config_t *config)
{
    ESP_LOGI(TAG, "Initializing warning manager");

    if (s_initialized) {
        ESP_LOGW(TAG, "Warning manager already initialized");
        return ESP_OK;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Configuration is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_config = *config;

    if (s_config.use_led_strip) {
        // Initialize addressable LED strip
        ESP_LOGI(TAG, "Configuring addressable LED strip on GPIO %d", s_config.led_gpio);
        
        led_strip_config_t strip_config = {
            .strip_gpio_num = s_config.led_gpio,
            .max_leds = 1, // ESP32-C3 has one built-in LED
        };
        
        led_strip_rmt_config_t rmt_config = {
             .clk_src = RMT_CLK_SRC_DEFAULT,
        // Set the RMT counter clock
            .resolution_hz = LED_STRIP_RMT_RES_HZ,
            .flags.with_dma = false,
        };
        
        esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize LED strip: %s", esp_err_to_name(ret));
            return ret;
        }

        // Clear LED initially
        led_strip_clear(s_led_strip);
        
    } else {
        // Initialize simple GPIO LED
        ESP_LOGI(TAG, "Configuring GPIO LED on pin %d", s_config.led_gpio);
        gpio_reset_pin(s_config.led_gpio);
        gpio_set_direction(s_config.led_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(s_config.led_gpio, 0); // LED off initially
    }

    // Load current warning thresholds from config
    const app_config_t *app_config = app_config_get_current();
    s_ppo2_low_warning = app_config->ppo2_low_warning;
    s_ppo2_low_alarm = app_config->ppo2_low_alarm;
    s_ppo2_high_warning = app_config->ppo2_high_warning;
    s_ppo2_high_alarm = app_config->ppo2_high_alarm;

    ESP_LOGI(TAG, "Warning thresholds: Low [%.2f-%.2f], High [%.2f-%.2f]", 
             s_ppo2_low_alarm, s_ppo2_low_warning, s_ppo2_high_warning, s_ppo2_high_alarm);

    // Initialize with normal state (green LED)
    s_current_state = WARNING_STATE_NORMAL;
    s_last_blink_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_led_on = false;

    // Set initial green color
    set_led_color(LED_COLOR_GREEN_R, LED_COLOR_GREEN_G, LED_COLOR_GREEN_B);

    s_initialized = true;
    ESP_LOGI(TAG, "Warning manager initialized");
    return ESP_OK;
}

esp_err_t warning_manager_update(const sensor_data_t *sensor_data)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Warning manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (sensor_data == NULL) {
        ESP_LOGE(TAG, "Sensor data is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Update thresholds from current config (may have changed)
    const app_config_t *config = app_config_get_current();
    s_ppo2_low_warning = config->ppo2_low_warning;
    s_ppo2_low_alarm = config->ppo2_low_alarm;
    s_ppo2_high_warning = config->ppo2_high_warning;
    s_ppo2_high_alarm = config->ppo2_high_alarm;

    // Calculate new warning state based on dual sensor logic
    warning_state_t new_state;
    if (!sensor_data->valid) {
        // Invalid sensor data - show warning state
        new_state = WARNING_STATE_WARNING;
        snprintf(s_warning_message, sizeof(s_warning_message), "SENSOR DATA INVALID");
        ESP_LOGW(TAG, "Sensor data invalid - triggering warning state");
    } else {
        // Use enhanced dual sensor warning logic
        new_state = calculate_dual_sensor_warning_state(sensor_data);
    }
    
    if (new_state != s_current_state) {
        ESP_LOGI(TAG, "Warning state changed: %d -> %d (S1_PPO2: %.3f, S2_PPO2: %.3f, Valid: %d/%d)", 
                 s_current_state, new_state, 
                 sensor_data->o2_sensor1_ppo2, sensor_data->o2_sensor2_ppo2,
                 sensor_data->sensor1_valid, sensor_data->sensor2_valid);
        s_current_state = new_state;
        s_led_on = false; // Reset blink state when changing states
        
        // Clear warning message when returning to normal state
        if (new_state == WARNING_STATE_NORMAL) {
            s_warning_message[0] = '\0';
        }
    }

    // Update LED based on current state
    update_led_based_on_state();

    return ESP_OK;
}

warning_state_t warning_manager_get_state(void)
{
    return s_current_state;
}

const char* warning_manager_get_display_message(void)
{
    return s_warning_message[0] ? s_warning_message : NULL;
}

esp_err_t warning_manager_test(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Warning manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Testing warning system...");

    // Test green (normal)
    ESP_LOGI(TAG, "Testing GREEN (normal)");
    set_led_color(LED_COLOR_GREEN_R, LED_COLOR_GREEN_G, LED_COLOR_GREEN_B);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Test yellow blinking (warning)
    ESP_LOGI(TAG, "Testing YELLOW blinking (warning)");
    for (int i = 0; i < 8; i++) {
        if (i % 2 == 0) {
            set_led_color(LED_COLOR_YELLOW_R, LED_COLOR_YELLOW_G, LED_COLOR_YELLOW_B);
        } else {
            set_led_color(LED_COLOR_OFF_R, LED_COLOR_OFF_G, LED_COLOR_OFF_B);
        }
        vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD_WARNING_MS));
    }

    // Test red blinking (alarm)
    ESP_LOGI(TAG, "Testing RED blinking (alarm)");
    for (int i = 0; i < 16; i++) {
        if (i % 2 == 0) {
            set_led_color(LED_COLOR_RED_R, LED_COLOR_RED_G, LED_COLOR_RED_B);
        } else {
            set_led_color(LED_COLOR_OFF_R, LED_COLOR_OFF_G, LED_COLOR_OFF_B);
        }
        vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD_ALARM_MS));
    }

    // Return to normal state
    s_current_state = WARNING_STATE_NORMAL;
    set_led_color(LED_COLOR_GREEN_R, LED_COLOR_GREEN_G, LED_COLOR_GREEN_B);
    
    ESP_LOGI(TAG, "Warning system test completed");
    return ESP_OK;
}

void warning_manager_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing warning manager");

    if (s_initialized) {
        // Turn off LED
        if (s_config.use_led_strip && s_led_strip) {
            led_strip_clear(s_led_strip);
            led_strip_del(s_led_strip);
            s_led_strip = NULL;
        } else if (!s_config.use_led_strip) {
            gpio_set_level(s_config.led_gpio, 0);
        }

        s_initialized = false;
    }

    ESP_LOGI(TAG, "Warning manager deinitialized");
}

// Private helper functions

static esp_err_t set_led_color(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_config.use_led_strip && s_led_strip) {
        // Addressable LED strip
        esp_err_t ret = led_strip_set_pixel(s_led_strip, 0, red, green, blue);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set LED pixel: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = led_strip_refresh(s_led_strip);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to refresh LED strip: %s", esp_err_to_name(ret));
            return ret;
        }
    } else {
        // Simple GPIO LED (can only be on/off, no color)
        bool led_state = (red > 0 || green > 0 || blue > 0);
        gpio_set_level(s_config.led_gpio, led_state ? 1 : 0);
    }

    return ESP_OK;
}

static warning_state_t calculate_warning_state(int32_t ppo2)
{

    ESP_LOGD(TAG, "Calculating warning state for PPO2: %3ld mbar", ppo2);

    // Check alarm conditions first (most critical)
    if (ppo2 < s_ppo2_low_alarm*1000) {
        snprintf(s_warning_message, sizeof(s_warning_message), "PPO2 LOW ALARM %3ld mbar", ppo2);
        return WARNING_STATE_ALARM;
    } else if (ppo2 > s_ppo2_high_alarm*1000) {
        snprintf(s_warning_message, sizeof(s_warning_message), "PPO2 HIGH ALARM %3ld mbar", ppo2);
        return WARNING_STATE_ALARM;
    }
    
    // Check warning conditions
    if (ppo2 < s_ppo2_low_warning*1000) {
        snprintf(s_warning_message, sizeof(s_warning_message), "PPO2 LOW %3ld mbar", ppo2);
        return WARNING_STATE_WARNING;
    } else if (ppo2 > s_ppo2_high_warning*1000) {
        snprintf(s_warning_message, sizeof(s_warning_message), "PPO2 HIGH %3ld mbar", ppo2);
        return WARNING_STATE_WARNING;
    }
    
    // Normal range
    return WARNING_STATE_NORMAL;
}

static warning_state_t calculate_dual_sensor_warning_state(const sensor_data_t *sensor_data)
{
    warning_state_t worst_state = WARNING_STATE_NORMAL;
    
    ESP_LOGD(TAG, "Calculating dual sensor warning state: S1_PPO2=%3ld (valid=%d), S2_PPO2=%3ld (valid=%d)", 
             sensor_data->o2_sensor1_ppo2_mbar, sensor_data->sensor1_valid,
             sensor_data->o2_sensor2_ppo2_mbar, sensor_data->sensor2_valid);

    // Check each individual sensor (if valid)
    if (sensor_data->sensor1_valid) {
        warning_state_t sensor1_state = calculate_warning_state(sensor_data->o2_sensor1_ppo2_mbar);
        if (sensor1_state > worst_state) {
            worst_state = sensor1_state;
            ESP_LOGD(TAG, "Sensor #1 triggered warning state %d (PPO2: %.3f)", sensor1_state, sensor_data->o2_sensor1_ppo2);
        }
    }
    
    if (sensor_data->sensor2_valid) {
        warning_state_t sensor2_state = calculate_warning_state(sensor_data->o2_sensor2_ppo2_mbar);
        if (sensor2_state > worst_state) {
            worst_state = sensor2_state;
            ESP_LOGD(TAG, "Sensor #2 triggered warning state %d (PPO2: %.3f)", sensor2_state, sensor_data->o2_sensor2_ppo2);
        }
    }
    
    // Check for sensor disagreement and single sensor failures (only in dual sensor mode)
    bool is_single_sensor_mode = sensor_manager_is_single_sensor_mode();

    if (!is_single_sensor_mode) {
        // DUAL SENSOR MODE: Check for sensor disagreement (both sensors must be valid)
        if (sensor_data->sensor1_valid && sensor_data->sensor2_valid) {
            float ppo2_difference = fabs(sensor_data->o2_sensor1_ppo2_mbar - sensor_data->o2_sensor2_ppo2_mbar);
            if (ppo2_difference > PPO2_DISAGREEMENT_THRESHOLD * 1000) {
                // Sensor disagreement - trigger warning (but not alarm unless one sensor is already in alarm)
                if (worst_state < WARNING_STATE_WARNING) {
                    worst_state = WARNING_STATE_WARNING;
                }
                // Set display message
                snprintf(s_warning_message, sizeof(s_warning_message),
                         "SENSOR DISAGREEMENT %.2f", ppo2_difference);
                ESP_LOGW(TAG, "Sensor disagreement detected: S1=%.3f, S2=%.3f, diff=%.3f (threshold=%.3f)",
                         sensor_data->o2_sensor1_ppo2, sensor_data->o2_sensor2_ppo2,
                         ppo2_difference, PPO2_DISAGREEMENT_THRESHOLD);
            }
        }

        // DUAL SENSOR MODE: If only one sensor is valid, show warning (single sensor failure)
        if ((sensor_data->sensor1_valid && !sensor_data->sensor2_valid) ||
            (!sensor_data->sensor1_valid && sensor_data->sensor2_valid)) {
            if (worst_state < WARNING_STATE_WARNING) {
                worst_state = WARNING_STATE_WARNING;
            }
            // Set display message
            if (sensor_data->sensor1_valid) {
                snprintf(s_warning_message, sizeof(s_warning_message), "SENSOR 2 FAILURE");
            } else {
                snprintf(s_warning_message, sizeof(s_warning_message), "SENSOR 1 FAILURE");
            }
            ESP_LOGW(TAG, "Single sensor failure detected in dual mode: S1_valid=%d, S2_valid=%d",
                     sensor_data->sensor1_valid, sensor_data->sensor2_valid);
        }
    } else {
        // SINGLE SENSOR MODE: Only check if the active sensor failed
        int active_sensor_id = sensor_manager_get_active_sensor_id();
        bool active_sensor_valid = (active_sensor_id == 0) ? sensor_data->sensor1_valid : sensor_data->sensor2_valid;

        if (!active_sensor_valid) {
            if (worst_state < WARNING_STATE_WARNING) {
                worst_state = WARNING_STATE_WARNING;
            }
            snprintf(s_warning_message, sizeof(s_warning_message), "ACTIVE SENSOR %d FAILURE", active_sensor_id + 1);
            ESP_LOGW(TAG, "Active sensor %d failed in single sensor mode", active_sensor_id + 1);
        } else {
            ESP_LOGD(TAG, "Single sensor mode: Sensor %d operating normally", active_sensor_id + 1);
        }
    }
    
    return worst_state;
}

static void update_led_based_on_state(void)
{
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    switch (s_current_state) {
        case WARNING_STATE_NORMAL:
            // Solid green
            set_led_color(LED_COLOR_OFF_R, LED_COLOR_OFF_G, LED_COLOR_OFF_B);
            break;
            
        case WARNING_STATE_WARNING:
            // Yellow blinking at 2Hz (500ms period, 250ms on/off)
            if (current_time - s_last_blink_time >= BLINK_PERIOD_WARNING_MS) {
                s_led_on = !s_led_on;
                s_last_blink_time = current_time;
                
                if (s_led_on) {
                    set_led_color(LED_COLOR_YELLOW_R, LED_COLOR_YELLOW_G, LED_COLOR_YELLOW_B);
                } else {
                    set_led_color(LED_COLOR_OFF_R, LED_COLOR_OFF_G, LED_COLOR_OFF_B);
                }
            }
            break;
            
        case WARNING_STATE_ALARM:
            // Red blinking at 4Hz (250ms period, 125ms on/off) 
            if (current_time - s_last_blink_time >= BLINK_PERIOD_ALARM_MS) {
                s_led_on = !s_led_on;
                s_last_blink_time = current_time;
                
                if (s_led_on) {
                    set_led_color(LED_COLOR_RED_R, LED_COLOR_RED_G, LED_COLOR_RED_B);
                } else {
                    set_led_color(LED_COLOR_OFF_R, LED_COLOR_OFF_G, LED_COLOR_OFF_B);
                }
            }
            break;
    }
}