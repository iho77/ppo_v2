/**
 * @file sensor_calibration_storage.c
 * @brief NVS storage functions for sensor calibration system
 * @version 1.0
 * @date 2025-01-27
 */

#include "sensor_calibration.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>
#include <string.h>

static const char *TAG = "CAL_STORAGE";

// NVS storage keys
#define NVS_NAMESPACE               "sensor_cal"
#define NVS_KEY_POWER_CYCLES        "power_cycles"
#define NVS_KEY_NEXT_CAL_ID         "next_cal_id"
#define NVS_KEY_BASELINES           "baselines"
#define NVS_KEY_THRESHOLDS          "thresholds"
/* UNUSED 2025-09-20: Not used in this TU */
// #define NVS_KEY_HISTORY_META        "hist_meta"
#define NVS_KEY_CURRENT_CAL         "curr_cal"

// History is stored in chunks to work around NVS size limits
/* UNUSED 2025-09-20: Chunked history not implemented currently */
// #define HISTORY_CHUNK_SIZE          50
// #define MAX_HISTORY_CHUNKS          12  // 600 entries / 50 per chunk

// External reference to storage (defined in main calibration file)
extern calibration_storage_t *s_storage;

// External function declarations
extern uint32_t calculate_checksum(const calibration_storage_t *storage);

// Forward declaration for save function
esp_err_t save_to_nvs(void);

/**
 * @brief Load calibration data from NVS
 * @return ESP_OK on success
 */
esp_err_t load_from_nvs(void)
{
    if (!s_storage) {
        ESP_LOGE(TAG, "Storage not allocated");
        return ESP_ERR_INVALID_STATE;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Loading calibration data from NVS");
    
    // Load global counters
    size_t required_size = sizeof(uint32_t);
    ret = nvs_get_blob(nvs_handle, NVS_KEY_POWER_CYCLES, &s_storage->power_cycle_count, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Power cycles not found, starting from 0");
        s_storage->power_cycle_count = 0;
    }
    
    required_size = sizeof(uint32_t);
    ret = nvs_get_blob(nvs_handle, NVS_KEY_NEXT_CAL_ID, &s_storage->next_calibration_id, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Next calibration ID not found, starting from 1");
        s_storage->next_calibration_id = 1;
    }
    
    ESP_LOGI(TAG, "Loaded counters: power_cycles=%lu, next_cal_id=%lu", 
             s_storage->power_cycle_count, s_storage->next_calibration_id);
    
    // Load sensor baselines
    required_size = sizeof(s_storage->baselines);
    ret = nvs_get_blob(nvs_handle, NVS_KEY_BASELINES, s_storage->baselines, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Baselines not found, initializing empty");
        memset(s_storage->baselines, 0, sizeof(s_storage->baselines));
    } else {
        ESP_LOGI(TAG, "Loaded baselines for %d sensors", NUM_O2_SENSORS);
        for (int i = 0; i < NUM_O2_SENSORS; i++) {
            if (s_storage->baselines[i].valid) {
            ESP_LOGD(TAG, "S%d baseline: key='%s', sens=%.2f mV/bar, cals=%lu",
                     i, s_storage->baselines[i].sensor_key,
                     s_storage->baselines[i].baseline_sensitivity,
                     s_storage->baselines[i].total_calibrations);
            }
        }
    }
    
    // Load current calibrations
    required_size = sizeof(s_storage->current);
    ret = nvs_get_blob(nvs_handle, NVS_KEY_CURRENT_CAL, s_storage->current, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Current calibrations not found");
        memset(s_storage->current, 0, sizeof(s_storage->current));
    } else {
        ESP_LOGI(TAG, "Loaded current calibrations");
        for (int i = 0; i < NUM_O2_SENSORS; i++) {
            if (s_storage->current[i].valid) {
            ESP_LOGD(TAG, "S%d current cal: ID=%lu, sens=%.2f mV/bar, offset=%.2f mV",
                     i, s_storage->current[i].calibration_id,
                     s_storage->current[i].sensitivity_mv_per_bar,
                     s_storage->current[i].offset_mv);
            }
        }
    }
    
    // Load thresholds
    required_size = sizeof(calibration_thresholds_t);
    ret = nvs_get_blob(nvs_handle, NVS_KEY_THRESHOLDS, &s_storage->thresholds, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Thresholds not found, using defaults");
        // Defaults are already set in main init function
    } else {
        ESP_LOGI(TAG, "Loaded calibration thresholds");
    }
    
    // Load simple tracking data for each sensor
    for (int sensor = 0; sensor < NUM_O2_SENSORS; sensor++) {
        char key[32];
        
        // Load previous calibration for comparison
        snprintf(key, sizeof(key), "prev_cal_s%d", sensor);
        required_size = sizeof(calibration_log_entry_t);
        ret = nvs_get_blob(nvs_handle, key, &s_storage->previous[sensor], &required_size);
        if (ret != ESP_OK) {
            ESP_LOGD(TAG, "S%d previous calibration not found", sensor);
            memset(&s_storage->previous[sensor], 0, sizeof(calibration_log_entry_t));
        }
        
        // Load calibration count
        snprintf(key, sizeof(key), "total_cals_s%d", sensor);
        required_size = sizeof(uint32_t);
        ret = nvs_get_blob(nvs_handle, key, &s_storage->total_calibrations[sensor], &required_size);
        if (ret != ESP_OK) {
            ESP_LOGD(TAG, "S%d total calibration count not found", sensor);
            s_storage->total_calibrations[sensor] = 0;
        }
        
        // Load running averages for trend tracking
        snprintf(key, sizeof(key), "avg_sens_s%d", sensor);
        required_size = sizeof(double);
        ret = nvs_get_blob(nvs_handle, key, &s_storage->avg_sensitivity_trend[sensor], &required_size);
        if (ret != ESP_OK) {
            s_storage->avg_sensitivity_trend[sensor] = 0.0;
        }
        
        snprintf(key, sizeof(key), "avg_off_s%d", sensor);
        ret = nvs_get_blob(nvs_handle, key, &s_storage->avg_offset_drift[sensor], &required_size);
        if (ret != ESP_OK) {
            s_storage->avg_offset_drift[sensor] = 0.0;
        }
        
        if (s_storage->total_calibrations[sensor] > 0) {
            ESP_LOGI(TAG, "S%d tracking: %lu calibrations, avg_sens_trend=%.4f", 
                     sensor, s_storage->total_calibrations[sensor], s_storage->avg_sensitivity_trend[sensor]);
        }
    }
    
    nvs_close(nvs_handle);
    
    // If this is a fresh system (no checksum saved), initialize it properly and save
    if (s_storage->checksum == 0) {
        ESP_LOGI(TAG, "Fresh system detected - initializing checksum and saving to NVS");
        s_storage->checksum = calculate_checksum(s_storage);
        
        // Save the fresh structure to NVS immediately
        esp_err_t save_ret = save_to_nvs();
        if (save_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to save fresh calibration data: %s", esp_err_to_name(save_ret));
        } else {
            ESP_LOGI(TAG, "Fresh calibration data saved with checksum: 0x%08lx", s_storage->checksum);
        }
    } else {
        // Validate integrity for existing data
        uint32_t stored_checksum = s_storage->checksum;
        uint32_t calculated_checksum = calculate_checksum(s_storage);
        s_storage->checksum = stored_checksum; // Restore stored checksum
        
        if (stored_checksum != calculated_checksum) {
            ESP_LOGW(TAG, "Checksum mismatch: stored=0x%08lx, calculated=0x%08lx", 
                     stored_checksum, calculated_checksum);
            // Reinitialize checksum for corrupted data
            s_storage->checksum = calculate_checksum(s_storage);
            ESP_LOGI(TAG, "Checksum reinitialized to: 0x%08lx", s_storage->checksum);
            
            // Save the corrected structure
            esp_err_t save_ret = save_to_nvs();
            if (save_ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to save corrected calibration data: %s", esp_err_to_name(save_ret));
            }
        }
    }
    
    ESP_LOGI(TAG, "Calibration data loaded successfully");
    return ESP_OK;
}

/**
 * @brief Save calibration data to NVS
 * @return ESP_OK on success
 */
esp_err_t save_to_nvs(void)
{
    if (!s_storage) {
        ESP_LOGE(TAG, "Storage not allocated");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Update checksum before saving
    s_storage->checksum = calculate_checksum(s_storage);
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "Saving calibration data to NVS");
    
    // Save global counters
    ret = nvs_set_blob(nvs_handle, NVS_KEY_POWER_CYCLES, &s_storage->power_cycle_count, sizeof(uint32_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save power cycles: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ret = nvs_set_blob(nvs_handle, NVS_KEY_NEXT_CAL_ID, &s_storage->next_calibration_id, sizeof(uint32_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save next cal ID: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Save sensor baselines
    ret = nvs_set_blob(nvs_handle, NVS_KEY_BASELINES, s_storage->baselines, sizeof(s_storage->baselines));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save baselines: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Save current calibrations
    ret = nvs_set_blob(nvs_handle, NVS_KEY_CURRENT_CAL, s_storage->current, sizeof(s_storage->current));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save current calibrations: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Save thresholds
    ret = nvs_set_blob(nvs_handle, NVS_KEY_THRESHOLDS, &s_storage->thresholds, sizeof(calibration_thresholds_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save thresholds: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Save simple tracking data for each sensor
    for (int sensor = 0; sensor < NUM_O2_SENSORS; sensor++) {
        char key[32];
        
        // Save previous calibration
        snprintf(key, sizeof(key), "prev_cal_s%d", sensor);
        ret = nvs_set_blob(nvs_handle, key, &s_storage->previous[sensor], sizeof(calibration_log_entry_t));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save S%d previous calibration: %s", sensor, esp_err_to_name(ret));
            goto cleanup;
        }
        
        // Save calibration count
        snprintf(key, sizeof(key), "total_cals_s%d", sensor);
        ret = nvs_set_blob(nvs_handle, key, &s_storage->total_calibrations[sensor], sizeof(uint32_t));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save S%d total calibrations: %s", sensor, esp_err_to_name(ret));
            goto cleanup;
        }
        
        // Save running averages
        snprintf(key, sizeof(key), "avg_sens_s%d", sensor);
        ret = nvs_set_blob(nvs_handle, key, &s_storage->avg_sensitivity_trend[sensor], sizeof(double));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save S%d avg sensitivity trend: %s", sensor, esp_err_to_name(ret));
            goto cleanup;
        }
        
        snprintf(key, sizeof(key), "avg_off_s%d", sensor);
        ret = nvs_set_blob(nvs_handle, key, &s_storage->avg_offset_drift[sensor], sizeof(double));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save S%d avg offset drift: %s", sensor, esp_err_to_name(ret));
            goto cleanup;
        }
        
        ESP_LOGD(TAG, "Saved S%d tracking data: %lu calibrations", sensor, s_storage->total_calibrations[sensor]);
    }
    
    // Commit all changes
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ESP_LOGD(TAG, "Calibration data saved successfully");

cleanup:
    nvs_close(nvs_handle);
    return ret;
}

/**
 * @brief Add calibration entry to history (with circular buffer management)
 * @param sensor_id Sensor ID (0 or 1)
 * @param entry Calibration log entry to add
 * @return ESP_OK on success
 */
esp_err_t add_to_history(uint8_t sensor_id, const calibration_log_entry_t *entry)
{
    if (!s_storage || sensor_id >= NUM_O2_SENSORS || !entry) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Store calibration record directly to NVS for complete history
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for calibration storage");
        return ret;
    }
    
    // Create unique NVS key: cal_s<sensor>_<cal_id>
    char cal_key[32];
    snprintf(cal_key, sizeof(cal_key), "cal_s%u_%lu", sensor_id, entry->calibration_id);
    
    ret = nvs_set_blob(nvs_handle, cal_key, entry, sizeof(calibration_log_entry_t));
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to store calibration S%u ID=%lu to NVS: %s", 
                 sensor_id, entry->calibration_id, esp_err_to_name(ret));
        return ret;
    }
    
    // Update running averages for trend tracking
    if (s_storage->total_calibrations[sensor_id] > 0) {
        // Calculate degradation since last calibration
        double current_normalized_sens = entry->normalized_sensitivity;
        double prev_normalized_sens = s_storage->previous[sensor_id].normalized_sensitivity;
        
        if (prev_normalized_sens > 0) {
            double sensitivity_change = current_normalized_sens - prev_normalized_sens;
            
            // Update running average degradation rate (exponential moving average)
            double alpha = 0.3; // Weight for new data (0.0 = no change, 1.0 = use only new value)
            s_storage->avg_sensitivity_trend[sensor_id] = 
                (1 - alpha) * s_storage->avg_sensitivity_trend[sensor_id] + alpha * (-sensitivity_change);
        }
        
        // Update offset drift running average
        double current_offset = entry->offset_mv;
        double prev_offset = s_storage->previous[sensor_id].offset_mv;
        double offset_change = fabs(current_offset - prev_offset);
        
        double alpha_offset = 0.3;
        s_storage->avg_offset_drift[sensor_id] = 
            (1 - alpha_offset) * s_storage->avg_offset_drift[sensor_id] + alpha_offset * offset_change;
    }
    
    // Store this entry as the "previous" for next comparison
    s_storage->previous[sensor_id] = *entry;
    
    // Increment calibration counter
    s_storage->total_calibrations[sensor_id]++;
    
    // Update baseline total calibrations counter
    if (s_storage->baselines[sensor_id].valid) {
        s_storage->baselines[sensor_id].total_calibrations++;
    }
    
    ESP_LOGD(TAG, "Stored S%u cal ID=%lu to NVS, tracking: total=%lu, avg_sens_trend=%.4f", 
             sensor_id, entry->calibration_id, s_storage->total_calibrations[sensor_id], 
             s_storage->avg_sensitivity_trend[sensor_id]);
    
    return ESP_OK;
}

/**
 * @brief Get calibration history for sensor
 * @param sensor_id Sensor ID (0 or 1)  
 * @param entries Output array of log entries
 * @param max_entries Maximum entries to retrieve
 * @param num_entries Number of entries retrieved
 * @return ESP_OK on success
 */
esp_err_t get_calibration_history(uint8_t sensor_id, calibration_log_entry_t *entries,
                                 uint8_t max_entries, uint8_t *num_entries)
{
    if (!s_storage || sensor_id >= NUM_O2_SENSORS || !entries || !num_entries) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *num_entries = 0;
    uint32_t total_cals = s_storage->total_calibrations[sensor_id];
    
    if (total_cals == 0) {
        return ESP_OK; // No calibrations yet
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for reading calibration history");
        return ret;
    }
    
    uint8_t entries_loaded = 0;
    uint8_t max_to_load = (max_entries > total_cals) ? total_cals : max_entries;
    
    // Try to load recent calibration records by ID (starting from latest and working backwards)
    uint32_t current_cal_id = (s_storage->current[sensor_id].valid) ? 
                             s_storage->current[sensor_id].calibration_id : total_cals;
    
    for (uint32_t cal_id = current_cal_id; cal_id >= 1 && entries_loaded < max_to_load; cal_id--) {
        char cal_key[32];
        snprintf(cal_key, sizeof(cal_key), "cal_s%u_%lu", sensor_id, cal_id);
        
        size_t required_size = sizeof(calibration_log_entry_t);
        ret = nvs_get_blob(nvs_handle, cal_key, &entries[entries_loaded], &required_size);
        
        if (ret == ESP_OK) {
            entries_loaded++;
        }
    }
    
    nvs_close(nvs_handle);
    
    *num_entries = entries_loaded;
    ESP_LOGD(TAG, "Retrieved %u history entries for S%u from NVS", *num_entries, sensor_id);
    
    return ESP_OK;
}
