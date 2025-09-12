/**
 * @file ppo2_logger.c
 * @brief PPO2 measurement logging system implementation
 */

#include "ppo2_logger.h"
#include "sensor_manager.h"
#include "sensor_calibration.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "PPO2_LOG";

// Data format version for NVS compatibility
#define PPO2_LOG_VERSION            1
#define PPO2_LOG_MAGIC              0x50504F32  // "PPO2"

// Internal storage structure
typedef struct {
    uint32_t magic;                     // Magic number for data validation
    uint32_t version;                   // Data format version
    uint16_t next_index;                // Next write position (circular buffer)
    uint16_t entry_count;               // Number of valid entries (0-180)
    uint32_t boot_counter;              // Power cycle counter
    uint32_t total_entries_ever;        // Total entries since first installation
    ppo2_log_entry_t entries[PPO2_LOG_MAX_ENTRIES];
    uint32_t checksum;                  // Data integrity checksum
} ppo2_log_storage_t;

// Runtime state
static struct {
    bool initialized;
    bool running;
    ppo2_log_storage_t* storage;        // Allocated storage buffer
    esp_timer_handle_t log_timer;       // Timer for periodic logging
    esp_timer_handle_t save_timer;      // Timer for NVS saves
    nvs_handle_t nvs_handle;            // NVS handle
    uint32_t last_save_time;            // Last NVS save timestamp
    bool needs_save;                    // Flag indicating unsaved changes
    uint32_t startup_time_ms;           // Boot time for timestamp calculation
} s_logger_state = {0};

// Forward declarations
static uint32_t calculate_checksum(const ppo2_log_storage_t* storage);
static esp_err_t load_from_nvs(void);
static esp_err_t save_to_nvs(void);
static void log_timer_callback(void* arg);
static void save_timer_callback(void* arg);
static uint32_t get_uptime_minutes(void);
// ppo2_logger_add_entry is now public (declared in header)

/**
 * @brief Calculate CRC32-like checksum for data integrity
 */
static uint32_t calculate_checksum(const ppo2_log_storage_t* storage)
{
    if (!storage) return 0;
    
    uint32_t checksum = 0;
    const uint8_t* data = (const uint8_t*)storage;
    size_t len = sizeof(ppo2_log_storage_t) - sizeof(uint32_t); // Exclude checksum field
    
    for (size_t i = 0; i < len; i++) {
        checksum = checksum * 31 + data[i];
    }
    
    return checksum;
}

/**
 * @brief Get system uptime in minutes
 */
static uint32_t get_uptime_minutes(void)
{
    uint64_t uptime_us = esp_timer_get_time();
    uint64_t uptime_ms = uptime_us / 1000;
    return (uint32_t)(uptime_ms / 60000);  // Convert to minutes
}

/**
 * @brief Load log data from NVS
 */
static esp_err_t load_from_nvs(void)
{
    if (!s_logger_state.storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    size_t required_size = sizeof(ppo2_log_storage_t);
    esp_err_t ret = nvs_get_blob(s_logger_state.nvs_handle, PPO2_LOG_NVS_KEY, 
                                s_logger_state.storage, &required_size);
    
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        // First boot - initialize with defaults
        ESP_LOGI(TAG, "No existing log data, initializing fresh log");
        memset(s_logger_state.storage, 0, sizeof(ppo2_log_storage_t));
        s_logger_state.storage->magic = PPO2_LOG_MAGIC;
        s_logger_state.storage->version = PPO2_LOG_VERSION;
        s_logger_state.storage->boot_counter = 1;
        s_logger_state.storage->checksum = calculate_checksum(s_logger_state.storage);
        return ESP_OK;
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read log data from NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Validate loaded data
    if (s_logger_state.storage->magic != PPO2_LOG_MAGIC) {
        ESP_LOGW(TAG, "Invalid magic number in stored data, reinitializing");
        memset(s_logger_state.storage, 0, sizeof(ppo2_log_storage_t));
        s_logger_state.storage->magic = PPO2_LOG_MAGIC;
        s_logger_state.storage->version = PPO2_LOG_VERSION;
        s_logger_state.storage->boot_counter = 1;
        s_logger_state.storage->checksum = calculate_checksum(s_logger_state.storage);
        return ESP_OK;
    }
    
    // Verify checksum
    uint32_t calculated_checksum = calculate_checksum(s_logger_state.storage);
    if (calculated_checksum != s_logger_state.storage->checksum) {
        ESP_LOGW(TAG, "Checksum mismatch in stored data (calc=0x%08lx, stored=0x%08lx), reinitializing", 
                 calculated_checksum, s_logger_state.storage->checksum);
        // Keep boot counter and total entries if possible
        uint32_t saved_boot_counter = s_logger_state.storage->boot_counter;
        uint32_t saved_total_entries = s_logger_state.storage->total_entries_ever;
        
        memset(s_logger_state.storage, 0, sizeof(ppo2_log_storage_t));
        s_logger_state.storage->magic = PPO2_LOG_MAGIC;
        s_logger_state.storage->version = PPO2_LOG_VERSION;
        s_logger_state.storage->boot_counter = saved_boot_counter + 1;
        s_logger_state.storage->total_entries_ever = saved_total_entries;
        s_logger_state.storage->checksum = calculate_checksum(s_logger_state.storage);
        return ESP_OK;
    }
    
    // Increment boot counter on successful load
    s_logger_state.storage->boot_counter++;
    s_logger_state.storage->checksum = calculate_checksum(s_logger_state.storage);
    s_logger_state.needs_save = true;
    
    ESP_LOGI(TAG, "Loaded log data: %d entries, boot #%lu, total entries ever: %lu",
             s_logger_state.storage->entry_count, s_logger_state.storage->boot_counter,
             s_logger_state.storage->total_entries_ever);
    
    return ESP_OK;
}

/**
 * @brief Save log data to NVS
 */
static esp_err_t save_to_nvs(void)
{
    if (!s_logger_state.storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Update checksum before save
    s_logger_state.storage->checksum = calculate_checksum(s_logger_state.storage);
    
    // Atomic save using NVS blob
    esp_err_t ret = nvs_set_blob(s_logger_state.nvs_handle, PPO2_LOG_NVS_KEY, 
                                s_logger_state.storage, sizeof(ppo2_log_storage_t));
    if (ret == ESP_OK) {
        ret = nvs_commit(s_logger_state.nvs_handle);
        if (ret == ESP_OK) {
            s_logger_state.needs_save = false;
            s_logger_state.last_save_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            ESP_LOGD(TAG, "Log data saved to NVS successfully");
        }
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save log data to NVS: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Internal function to add entry to circular buffer
 */
esp_err_t ppo2_logger_add_entry(uint8_t sensor_id, double ppo2_bar, uint32_t calibration_id)
{
    if (!s_logger_state.storage || !s_logger_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (sensor_id >= 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Add entry to circular buffer
    ppo2_log_entry_t* entry = &s_logger_state.storage->entries[s_logger_state.storage->next_index];
    
    entry->sensor_id = sensor_id;
    entry->reserved = 0;
    entry->timestamp_minutes = (uint16_t)get_uptime_minutes();
    entry->calibration_id = calibration_id;
    entry->ppo2_bar = ppo2_bar;
    
    // Update circular buffer pointers
    s_logger_state.storage->next_index = (s_logger_state.storage->next_index + 1) % PPO2_LOG_MAX_ENTRIES;
    if (s_logger_state.storage->entry_count < PPO2_LOG_MAX_ENTRIES) {
        s_logger_state.storage->entry_count++;
    }
    s_logger_state.storage->total_entries_ever++;
    s_logger_state.needs_save = true;
    
    ESP_LOGV(TAG, "Added entry: S%d, %d min, PPO2=%.3f, cal_id=%lu", 
             sensor_id, entry->timestamp_minutes, ppo2_bar, calibration_id);
    
    return ESP_OK;
}

/**
 * @brief Timer callback for periodic logging
 */
static void log_timer_callback(void* arg)
{
    if (!s_logger_state.running || !s_logger_state.initialized) {
        return;
    }
    
    // Get current sensor data
    sensor_data_t sensor_data;
    esp_err_t ret = sensor_manager_read(&sensor_data);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read sensor data for logging: %s", esp_err_to_name(ret));
        return;
    }
    
    // Log entries for each valid sensor
    if (sensor_data.sensor1_valid && sensor_data.valid) {
        // Get current calibration ID for sensor 1
        multi_point_calibration_t current_cal;
        uint32_t cal_id = 0;
        if (sensor_calibration_get_current(0, &current_cal) == ESP_OK) {
            cal_id = current_cal.calibration_id;
        }
        
        esp_err_t add_ret = ppo2_logger_add_entry(0, sensor_data.o2_sensor1_ppo2, cal_id);
        if (add_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to log sensor 1 data: %s", esp_err_to_name(add_ret));
        }
    }
    
    if (sensor_data.sensor2_valid && sensor_data.valid) {
        // Get current calibration ID for sensor 2
        multi_point_calibration_t current_cal;
        uint32_t cal_id = 0;
        if (sensor_calibration_get_current(1, &current_cal) == ESP_OK) {
            cal_id = current_cal.calibration_id;
        }
        
        esp_err_t add_ret = ppo2_logger_add_entry(1, sensor_data.o2_sensor2_ppo2, cal_id);
        if (add_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to log sensor 2 data: %s", esp_err_to_name(add_ret));
        }
    }
    
    // If no sensors are valid, still log the fact that we're running
    if (!sensor_data.valid) {
        ESP_LOGV(TAG, "Skipping log entry - no valid sensor data");
    }
}

/**
 * @brief Timer callback for periodic NVS saves
 */
static void save_timer_callback(void* arg)
{
    if (s_logger_state.needs_save && s_logger_state.initialized) {
        esp_err_t ret = save_to_nvs();
        if (ret == ESP_OK) {
            ESP_LOGD(TAG, "Periodic save completed successfully");
        }
    }
}

// Public API Implementation

esp_err_t ppo2_logger_init(void)
{
    if (s_logger_state.initialized) {
        ESP_LOGW(TAG, "PPO2 logger already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing PPO2 logger");
    
    // Allocate storage buffer
    s_logger_state.storage = malloc(sizeof(ppo2_log_storage_t));
    if (!s_logger_state.storage) {
        ESP_LOGE(TAG, "Failed to allocate storage buffer (%d bytes)", sizeof(ppo2_log_storage_t));
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize NVS
    esp_err_t ret = nvs_open(PPO2_LOG_NVS_NAMESPACE, NVS_READWRITE, &s_logger_state.nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s", PPO2_LOG_NVS_NAMESPACE, esp_err_to_name(ret));
        free(s_logger_state.storage);
        s_logger_state.storage = NULL;
        return ret;
    }
    
    // Load existing data or initialize
    ret = load_from_nvs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load data from NVS: %s", esp_err_to_name(ret));
        nvs_close(s_logger_state.nvs_handle);
        free(s_logger_state.storage);
        s_logger_state.storage = NULL;
        return ret;
    }
    
    // Create timers (but don't start them yet)
    esp_timer_create_args_t log_timer_args = {
        .callback = &log_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ppo2_log_timer"
    };
    
    ret = esp_timer_create(&log_timer_args, &s_logger_state.log_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create log timer: %s", esp_err_to_name(ret));
        nvs_close(s_logger_state.nvs_handle);
        free(s_logger_state.storage);
        s_logger_state.storage = NULL;
        return ret;
    }
    
    esp_timer_create_args_t save_timer_args = {
        .callback = &save_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ppo2_save_timer"
    };
    
    ret = esp_timer_create(&save_timer_args, &s_logger_state.save_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create save timer: %s", esp_err_to_name(ret));
        esp_timer_delete(s_logger_state.log_timer);
        nvs_close(s_logger_state.nvs_handle);
        free(s_logger_state.storage);
        s_logger_state.storage = NULL;
        return ret;
    }
    
    // Record startup time
    s_logger_state.startup_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_logger_state.initialized = true;
    s_logger_state.running = false;
    
    ESP_LOGI(TAG, "PPO2 logger initialized successfully (storage: %d bytes)", sizeof(ppo2_log_storage_t));
    
    return ESP_OK;
}

esp_err_t ppo2_logger_start(void)
{
    if (!s_logger_state.initialized) {
        ESP_LOGE(TAG, "PPO2 logger not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_logger_state.running) {
        ESP_LOGW(TAG, "PPO2 logger already running");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Starting PPO2 logger (interval: %d ms)", PPO2_LOG_INTERVAL_MS);
    
    // Start periodic logging timer
    esp_err_t ret = esp_timer_start_periodic(s_logger_state.log_timer, PPO2_LOG_INTERVAL_MS * 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start log timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Start periodic save timer
    ret = esp_timer_start_periodic(s_logger_state.save_timer, PPO2_LOG_WRITE_INTERVAL_MS * 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start save timer: %s", esp_err_to_name(ret));
        esp_timer_stop(s_logger_state.log_timer);
        return ret;
    }
    
    s_logger_state.running = true;
    
    ESP_LOGI(TAG, "PPO2 logger started successfully");
    return ESP_OK;
}

esp_err_t ppo2_logger_stop(void)
{
    if (!s_logger_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!s_logger_state.running) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Stopping PPO2 logger");
    
    // Stop timers
    esp_timer_stop(s_logger_state.log_timer);
    esp_timer_stop(s_logger_state.save_timer);
    
    // Save any pending data
    if (s_logger_state.needs_save) {
        esp_err_t ret = save_to_nvs();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to save pending data on stop: %s", esp_err_to_name(ret));
        }
    }
    
    s_logger_state.running = false;
    
    ESP_LOGI(TAG, "PPO2 logger stopped");
    return ESP_OK;
}


esp_err_t ppo2_logger_get_stats(ppo2_log_stats_t *stats)
{
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_logger_state.initialized || !s_logger_state.storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    memset(stats, 0, sizeof(ppo2_log_stats_t));
    
    stats->total_entries = s_logger_state.storage->entry_count;
    stats->boot_counter = s_logger_state.storage->boot_counter;
    stats->total_entries_ever = s_logger_state.storage->total_entries_ever;
    
    if (stats->total_entries > 0) {
        // Find oldest and newest timestamps
        uint32_t oldest = UINT32_MAX;
        uint32_t newest = 0;
        uint8_t sensors_mask = 0;
        
        uint16_t start_idx;
        if (s_logger_state.storage->entry_count < PPO2_LOG_MAX_ENTRIES) {
            start_idx = 0;
        } else {
            start_idx = s_logger_state.storage->next_index;
        }
        
        for (uint16_t i = 0; i < s_logger_state.storage->entry_count; i++) {
            uint16_t idx = (start_idx + i) % PPO2_LOG_MAX_ENTRIES;
            const ppo2_log_entry_t* entry = &s_logger_state.storage->entries[idx];
            
            if (entry->timestamp_minutes < oldest) {
                oldest = entry->timestamp_minutes;
            }
            if (entry->timestamp_minutes > newest) {
                newest = entry->timestamp_minutes;
            }
            
            sensors_mask |= (1 << entry->sensor_id);
        }
        
        stats->oldest_timestamp = oldest;
        stats->newest_timestamp = newest;
        stats->minutes_logged = newest - oldest + 1;
        stats->sensors_active = sensors_mask;
    }
    
    return ESP_OK;
}

esp_err_t ppo2_logger_get_entries(ppo2_log_entry_t *entries, 
                                 uint16_t max_entries, 
                                 uint16_t *num_entries,
                                 uint16_t start_index)
{
    if (!entries || !num_entries) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_logger_state.initialized || !s_logger_state.storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    *num_entries = 0;
    
    if (start_index >= s_logger_state.storage->entry_count || max_entries == 0) {
        return ESP_OK;
    }
    
    // Calculate starting position in circular buffer
    uint16_t buffer_start;
    if (s_logger_state.storage->entry_count < PPO2_LOG_MAX_ENTRIES) {
        buffer_start = 0;
    } else {
        buffer_start = s_logger_state.storage->next_index;
    }
    
    uint16_t entries_to_copy = s_logger_state.storage->entry_count - start_index;
    if (entries_to_copy > max_entries) {
        entries_to_copy = max_entries;
    }
    
    // Copy entries in chronological order
    for (uint16_t i = 0; i < entries_to_copy; i++) {
        uint16_t idx = (buffer_start + start_index + i) % PPO2_LOG_MAX_ENTRIES;
        entries[i] = s_logger_state.storage->entries[idx];
    }
    
    *num_entries = entries_to_copy;
    return ESP_OK;
}

esp_err_t ppo2_logger_print_csv(void)
{
    if (!s_logger_state.initialized || !s_logger_state.storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "=== PPO2 LOG CSV EXPORT ===");
    ESP_LOGI(TAG, "SensorID,TimestampMin,CalibrationID,PPO2_bar");
    
    if (s_logger_state.storage->entry_count == 0) {
        ESP_LOGI(TAG, "No entries to export");
        return ESP_OK;
    }
    
    // Calculate starting position
    uint16_t start_idx;
    if (s_logger_state.storage->entry_count < PPO2_LOG_MAX_ENTRIES) {
        start_idx = 0;
    } else {
        start_idx = s_logger_state.storage->next_index;
    }
    
    // Print entries in chronological order
    for (uint16_t i = 0; i < s_logger_state.storage->entry_count; i++) {
        uint16_t idx = (start_idx + i) % PPO2_LOG_MAX_ENTRIES;
        const ppo2_log_entry_t* entry = &s_logger_state.storage->entries[idx];
        
        ESP_LOGI(TAG, "%d,%d,%lu,%.4f", 
                 entry->sensor_id, 
                 entry->timestamp_minutes, 
                 entry->calibration_id,
                 entry->ppo2_bar);
    }
    
    ESP_LOGI(TAG, "=== END CSV EXPORT (%d entries) ===", s_logger_state.storage->entry_count);
    return ESP_OK;
}

esp_err_t ppo2_logger_print_formatted(void)
{
    if (!s_logger_state.initialized || !s_logger_state.storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ppo2_log_stats_t stats;
    esp_err_t ret = ppo2_logger_get_stats(&stats);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(TAG, "=== PPO2 LOG SUMMARY ===");
    ESP_LOGI(TAG, "Total entries: %d/%d", stats.total_entries, PPO2_LOG_MAX_ENTRIES);
    ESP_LOGI(TAG, "Boot cycle: %lu", stats.boot_counter);
    ESP_LOGI(TAG, "Total entries ever: %lu", stats.total_entries_ever);
    ESP_LOGI(TAG, "Time span: %lu minutes", stats.minutes_logged);
    ESP_LOGI(TAG, "Active sensors: S1=%s S2=%s", 
             (stats.sensors_active & 1) ? "YES" : "NO",
             (stats.sensors_active & 2) ? "YES" : "NO");
    ESP_LOGI(TAG, "Logger status: %s", s_logger_state.running ? "RUNNING" : "STOPPED");
    
    if (stats.total_entries > 0) {
        ESP_LOGI(TAG, "\n=== RECENT ENTRIES ===");
        ppo2_log_entry_t recent_entries[10];
        uint16_t num_recent = 0;
        uint16_t start_idx = (stats.total_entries > 10) ? stats.total_entries - 10 : 0;
        
        ret = ppo2_logger_get_entries(recent_entries, 10, &num_recent, start_idx);
        if (ret == ESP_OK) {
            for (uint16_t i = 0; i < num_recent; i++) {
                const ppo2_log_entry_t* entry = &recent_entries[i];
                ESP_LOGI(TAG, "S%d @ %03dmin: PPO2=%.3f bar (cal:%lu)", 
                         entry->sensor_id, entry->timestamp_minutes, 
                         entry->ppo2_bar, entry->calibration_id);
            }
        }
    }
    
    ESP_LOGI(TAG, "=== END SUMMARY ===");
    return ESP_OK;
}

esp_err_t ppo2_logger_reset(void)
{
    if (!s_logger_state.initialized || !s_logger_state.storage) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Resetting PPO2 log (preserving boot counter)");
    
    // Preserve some counters
    uint32_t boot_counter = s_logger_state.storage->boot_counter;
    uint32_t total_entries = s_logger_state.storage->total_entries_ever;
    
    // Clear log data but preserve metadata
    s_logger_state.storage->next_index = 0;
    s_logger_state.storage->entry_count = 0;
    s_logger_state.storage->boot_counter = boot_counter;
    s_logger_state.storage->total_entries_ever = total_entries;
    
    // Clear all entries
    memset(s_logger_state.storage->entries, 0, sizeof(s_logger_state.storage->entries));
    
    // Update checksum and save
    s_logger_state.storage->checksum = calculate_checksum(s_logger_state.storage);
    s_logger_state.needs_save = true;
    
    esp_err_t ret = save_to_nvs();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "PPO2 log reset completed successfully");
    } else {
        ESP_LOGE(TAG, "Failed to save reset log: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t ppo2_logger_save_now(void)
{
    if (!s_logger_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return save_to_nvs();
}

bool ppo2_logger_is_running(void)
{
    return s_logger_state.running;
}

bool ppo2_logger_is_initialized(void)
{
    return s_logger_state.initialized;
}

void ppo2_logger_deinit(void)
{
    if (!s_logger_state.initialized) {
        return;
    }
    
    ESP_LOGI(TAG, "Deinitializing PPO2 logger");
    
    // Stop logging first
    ppo2_logger_stop();
    
    // Clean up timers
    if (s_logger_state.log_timer) {
        esp_timer_delete(s_logger_state.log_timer);
        s_logger_state.log_timer = NULL;
    }
    
    if (s_logger_state.save_timer) {
        esp_timer_delete(s_logger_state.save_timer);
        s_logger_state.save_timer = NULL;
    }
    
    // Close NVS
    if (s_logger_state.nvs_handle) {
        nvs_close(s_logger_state.nvs_handle);
        s_logger_state.nvs_handle = 0;
    }
    
    // Free storage
    if (s_logger_state.storage) {
        free(s_logger_state.storage);
        s_logger_state.storage = NULL;
    }
    
    // Reset state
    memset(&s_logger_state, 0, sizeof(s_logger_state));
    
    ESP_LOGI(TAG, "PPO2 logger deinitialized");
}