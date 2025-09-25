/**
 * @file app_config.c
 * @brief Application configuration management with NVS storage
 */

#include "app_config.h"
#include "app_types.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static const char *TAG = "APP_CONFIG";

// Global configuration cache
static app_config_t s_current_config = {0};
static bool s_config_loaded = false;

esp_err_t app_config_init(void)
{
    ESP_LOGI(TAG, "Initializing configuration system");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_LOGW(TAG, "NVS partition needs erasing, doing full erase");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
   // Load configuration into cache
    esp_err_t load_ret = app_config_load(&s_current_config);
    if (load_ret == ESP_OK) {
        s_config_loaded = true;
        ESP_LOGI(TAG, "Configuration system initialized and config loaded");
    } else {
        ESP_LOGW(TAG, "Configuration system initialized but config load failed: %s", esp_err_to_name(load_ret));
    }       
    return ESP_OK;
}

void app_config_get_defaults(app_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Config pointer is NULL");
        return;
    }
    
    // PPO2 alarm and warning limits
    config->ppo2_low_warning = PPO2_DEFAULT_LOW_WARNING;
    config->ppo2_low_alarm = PPO2_DEFAULT_LOW_ALARM;
    config->ppo2_high_warning = PPO2_DEFAULT_HIGH_WARNING;
    config->ppo2_high_alarm = PPO2_DEFAULT_HIGH_ALARM;

    // Sensor monitoring settings
    config->sensor_disagreement_threshold = SENSOR_DEFAULT_DISAGREEMENT_THRESHOLD;
    config->atmospheric_pressure = ATMOSPHERIC_DEFAULT_PRESSURE;
    
    // Display settings
    config->display_brightness = DISPLAY_DEFAULT_BRIGHTNESS;
    config->display_contrast = DISPLAY_DEFAULT_CONTRAST;
    config->auto_brightness = DISPLAY_DEFAULT_AUTO_BRIGHT;
    
    // System settings
    config->sleep_timeout_s = SYSTEM_DEFAULT_SLEEP_TIMEOUT_S;
    
    // Default date/time (2025-01-01 00:00:00)
    config->current_datetime.year = 2025;
    config->current_datetime.month = 1;
    config->current_datetime.day = 1;
    config->current_datetime.hour = 0;
    config->current_datetime.minute = 0;
    config->current_datetime.second = 0;
    
    // Default O2 calibration values for sensor #1 (calibrated for testing)
    config->o2_cal.calibration_gas_o2_fraction = 0.21f;  // Default to air (21% O2)
    // config->o2_cal.calibration_sensor_mv = 8.5f;        // Default sensor voltage for air - COMMENTED OUT: conflicts with modern calibration system
    config->o2_cal.calibrated = true;                   // Pre-calibrated for testing

    // Default O2 calibration values for sensor #2 (calibrated for testing)
    config->o2_cal_sensor2.calibration_gas_o2_fraction = 0.21f;  // Default to air (21% O2)
    // config->o2_cal_sensor2.calibration_sensor_mv = 8.7f;        // Slightly different default for sensor 2 - COMMENTED OUT: conflicts with modern calibration system
    config->o2_cal_sensor2.calibrated = true;                   // Pre-calibrated for testing
    
    ESP_LOGD(TAG, "Default configuration loaded");
}

esp_err_t app_config_load(app_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Config pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Loading configuration from NVS");
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE_CONFIG, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS namespace, using defaults: %s", esp_err_to_name(ret));
        app_config_get_defaults(config);
        return ESP_OK;  // Not an error, just use defaults
    }
    
    // Start with defaults
    app_config_get_defaults(config);
    
    // Load values from NVS (ignore errors, keep defaults)
    size_t required_size;
    
    // Load PPO2 limits
    required_size = sizeof(int32_t);
    nvs_get_blob(nvs_handle, NVS_KEY_PPO2_LOW_WARN, &config->ppo2_low_warning, &required_size);
    required_size = sizeof(int32_t);
    nvs_get_blob(nvs_handle, NVS_KEY_PPO2_LOW_ALARM, &config->ppo2_low_alarm, &required_size);
    required_size = sizeof(int32_t);
    nvs_get_blob(nvs_handle, NVS_KEY_PPO2_HIGH_WARN, &config->ppo2_high_warning, &required_size);
    required_size = sizeof(int32_t);
    nvs_get_blob(nvs_handle, NVS_KEY_PPO2_HIGH_ALARM, &config->ppo2_high_alarm, &required_size);

    // Load sensor monitoring settings
    required_size = sizeof(int32_t);
    nvs_get_blob(nvs_handle, NVS_KEY_SENSOR_DISAGREEMENT, &config->sensor_disagreement_threshold, &required_size);
    required_size = sizeof(int32_t);
    nvs_get_blob(nvs_handle, NVS_KEY_ATMOSPHERIC_PRESSURE, &config->atmospheric_pressure, &required_size);
    
    // Load display settings
    required_size = sizeof(uint8_t);
    nvs_get_blob(nvs_handle, NVS_KEY_BRIGHTNESS, &config->display_brightness, &required_size);
    required_size = sizeof(uint8_t);
    nvs_get_blob(nvs_handle, NVS_KEY_CONTRAST, &config->display_contrast, &required_size);
    
    uint8_t auto_bright = 0;
    required_size = sizeof(uint8_t);
    if (nvs_get_blob(nvs_handle, NVS_KEY_AUTO_BRIGHTNESS, &auto_bright, &required_size) == ESP_OK) {
        config->auto_brightness = (auto_bright != 0);
    }
    
    // Load system settings
    required_size = sizeof(uint32_t);
    nvs_get_blob(nvs_handle, NVS_KEY_SLEEP_TIMEOUT, &config->sleep_timeout_s, &required_size);
    
    // Load date/time
    required_size = sizeof(datetime_t);
    nvs_get_blob(nvs_handle, NVS_KEY_DATETIME, &config->current_datetime, &required_size);
    
    // Load O2 calibration data for sensor #1
    required_size = sizeof(float);
    nvs_get_blob(nvs_handle, "o2_cal_gas_o2", &config->o2_cal.calibration_gas_o2_fraction, &required_size);
    // required_size = sizeof(float);
    // nvs_get_blob(nvs_handle, "o2_cal_mv", &config->o2_cal.calibration_sensor_mv, &required_size); // COMMENTED OUT: conflicts with modern calibration system
    
    uint8_t cal_valid = 0;
    required_size = sizeof(uint8_t);
    if (nvs_get_blob(nvs_handle, NVS_KEY_O2_CAL_VALID, &cal_valid, &required_size) == ESP_OK) {
        config->o2_cal.calibrated = (cal_valid != 0);
    }
    
    // Load O2 calibration data for sensor #2
    required_size = sizeof(float);
    nvs_get_blob(nvs_handle, "o2_gas_s2", &config->o2_cal_sensor2.calibration_gas_o2_fraction, &required_size);
    // required_size = sizeof(float);
    // nvs_get_blob(nvs_handle, "o2_cal_mv_s2", &config->o2_cal_sensor2.calibration_sensor_mv, &required_size); // COMMENTED OUT: conflicts with modern calibration system
    
    uint8_t cal_valid_s2 = 0;
    required_size = sizeof(uint8_t);
    if (nvs_get_blob(nvs_handle, NVS_KEY_O2_CAL_VALID_S2, &cal_valid_s2, &required_size) == ESP_OK) {
        config->o2_cal_sensor2.calibrated = (cal_valid_s2 != 0);
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "Configuration loaded: PPO2 limits [%4ld-%4ld, %4ld-%4ld], brightness=%d%%, contrast=%d%%",
             config->ppo2_low_alarm, config->ppo2_low_warning,
             config->ppo2_high_warning, config->ppo2_high_alarm,
             config->display_brightness, config->display_contrast);
    ESP_LOGI(TAG, "Sensor monitoring: disagreement=%.3f bar, atmospheric=%.3f bar",
             config->sensor_disagreement_threshold, config->atmospheric_pressure);
    ESP_LOGI(TAG, "O2 sensor #1: cal_gas=%.3f O2, calibrated=%d",
             config->o2_cal.calibration_gas_o2_fraction, config->o2_cal.calibrated);
    ESP_LOGI(TAG, "O2 sensor #2: cal_gas=%.3f O2, calibrated=%d",
             config->o2_cal_sensor2.calibration_gas_o2_fraction, config->o2_cal_sensor2.calibrated);
    
    return ESP_OK;
}

esp_err_t app_config_save(const app_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Config pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Saving configuration to NVS");
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE_CONFIG, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Save PPO2 limits
    ret = nvs_set_blob(nvs_handle, NVS_KEY_PPO2_LOW_WARN, &config->ppo2_low_warning, sizeof(int32_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save PPO2 low warning: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ret = nvs_set_blob(nvs_handle, NVS_KEY_PPO2_LOW_ALARM, &config->ppo2_low_alarm, sizeof(int32_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save PPO2 low alarm: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ret = nvs_set_blob(nvs_handle, NVS_KEY_PPO2_HIGH_WARN, &config->ppo2_high_warning, sizeof(int32_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save PPO2 high warning: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ret = nvs_set_blob(nvs_handle, NVS_KEY_PPO2_HIGH_ALARM, &config->ppo2_high_alarm, sizeof(int32_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save PPO2 high alarm: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Save sensor monitoring settings
    ret = nvs_set_blob(nvs_handle, NVS_KEY_SENSOR_DISAGREEMENT, &config->sensor_disagreement_threshold, sizeof(int32_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save sensor disagreement threshold: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = nvs_set_blob(nvs_handle, NVS_KEY_ATMOSPHERIC_PRESSURE, &config->atmospheric_pressure, sizeof(int32_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save atmospheric pressure: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Save display settings
    ret = nvs_set_blob(nvs_handle, NVS_KEY_BRIGHTNESS, &config->display_brightness, sizeof(uint8_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save display brightness: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ret = nvs_set_blob(nvs_handle, NVS_KEY_CONTRAST, &config->display_contrast, sizeof(uint8_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save display contrast: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    uint8_t auto_bright = config->auto_brightness ? 1 : 0;
    ret = nvs_set_blob(nvs_handle, NVS_KEY_AUTO_BRIGHTNESS, &auto_bright, sizeof(uint8_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save auto brightness: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Save system settings
    ret = nvs_set_blob(nvs_handle, NVS_KEY_SLEEP_TIMEOUT, &config->sleep_timeout_s, sizeof(uint32_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save sleep timeout: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Save date/time
    ret = nvs_set_blob(nvs_handle, NVS_KEY_DATETIME, &config->current_datetime, sizeof(datetime_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save datetime: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Save O2 calibration data for sensor #1
    ret = nvs_set_blob(nvs_handle, "o2_cal_gas_o2", &config->o2_cal.calibration_gas_o2_fraction, sizeof(float));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save O2 #1 calibration gas O2: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // ret = nvs_set_blob(nvs_handle, "o2_cal_mv", &config->o2_cal.calibration_sensor_mv, sizeof(float)); // COMMENTED OUT: conflicts with modern calibration system
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to save O2 #1 calibration sensor mV: %s", esp_err_to_name(ret));
    //     goto cleanup;
    // }
    
    uint8_t cal_valid = config->o2_cal.calibrated ? 1 : 0;
    ret = nvs_set_blob(nvs_handle, NVS_KEY_O2_CAL_VALID, &cal_valid, sizeof(uint8_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save O2 calibration validity: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Save O2 calibration data for sensor #2
    ret = nvs_set_blob(nvs_handle, "o2_gas_s2", &config->o2_cal_sensor2.calibration_gas_o2_fraction, sizeof(float));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save O2 #2 calibration gas O2: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // ret = nvs_set_blob(nvs_handle, "o2_cal_mv_s2", &config->o2_cal_sensor2.calibration_sensor_mv, sizeof(float)); // COMMENTED OUT: conflicts with modern calibration system
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to save O2 #2 calibration sensor mV: %s", esp_err_to_name(ret));
    //     goto cleanup;
    // }
    
    uint8_t cal_valid_s2 = config->o2_cal_sensor2.calibrated ? 1 : 0;
    ret = nvs_set_blob(nvs_handle, NVS_KEY_O2_CAL_VALID_S2, &cal_valid_s2, sizeof(uint8_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save O2 sensor #2 calibration validity: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Commit changes
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ESP_LOGI(TAG, "Configuration saved successfully");
    
cleanup:
    nvs_close(nvs_handle);
    
    // If save was successful, update cached config
    if (ret == ESP_OK) {
        s_current_config = *config;
        s_config_loaded = true;
    }
    
    return ret;
}

const app_config_t* app_config_get_current(void)
{
    if (!s_config_loaded) {
        ESP_LOGW(TAG, "Config not loaded yet, returning defaults");
        app_config_get_defaults(&s_current_config);
        s_config_loaded = true;
    }
    
    return &s_current_config;
}

void app_log_error_to_display(const char* tag, const char* format, ...)
{
    static char error_buffer[256];
    
    va_list args;
    va_start(args, format);
    
    // Format the message
    vsnprintf(error_buffer, sizeof(error_buffer), format, args);
    
    // Log to ESP_LOGE only - display warnings handled separately by sensor validation
    ESP_LOGE(tag, "%s", error_buffer);
    
    va_end(args);
}