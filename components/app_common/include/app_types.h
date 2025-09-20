/**
 * @file app_types.h
 * @brief Core data structures for PPO2 HUD application
 * @version 1.0
 * @date 2025-01-27
 */

#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>  // For snprintf in inline functions

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Application operating modes
 */
typedef enum {
    MODE_MAIN = 0,        // Primary operational mode
    MODE_MENU,            // Menu selection mode
    MODE_CALIBRATION,     // Sensor calibration mode  
    MODE_CAL_RESET,       // Calibration reset mode
    MODE_SETUP,           // Configuration mode
    MODE_LOG,             // Log viewing mode
    MODE_PRINT_CALIBRATION, // Print calibration to debug
    MODE_SENSOR_HEALTH,   // Print sensor health to debug
    MODE_SLEEP,           // Low power mode
    MODE_MAX              // Total number of modes
} app_mode_t;

/**
 * @brief Button identifiers
 */
typedef enum {
    BUTTON_MODE = 0,      // Mode/menu button
    BUTTON_SELECT,        // Select/enter button
    BUTTON_MAX            // Total number of buttons
} button_id_t;

/**
 * @brief Button event types
 */
typedef enum {
    BUTTON_EVENT_NONE = 0,        // No event
    BUTTON_EVENT_PRESS,           // Button pressed (down)
    BUTTON_EVENT_RELEASE,         // Button released (up)
    BUTTON_EVENT_LONG_PRESS,      // Button held > 1 second
    BUTTON_EVENT_DOUBLE_PRESS,    // Two quick presses
    BUTTON_EVENT_MAX
} button_event_t;

/**
 * @brief Display mode types
 */
typedef enum {
    DISPLAY_MODE_MAIN = 0,    // PPO2 diving display with large numbers
    DISPLAY_MODE_TEXT,        // Simple text lines for menus/settings
    DISPLAY_MODE_MAX
} display_mode_t;

/**
 * @brief Display data structure - UI-specific data only
 * Note: Sensor values are passed separately to avoid duplication
 */
typedef struct {
    // Text mode data (for menus, settings, etc.)
    char line1[32];           // First line of text
    char line2[32];           // Second line of text  
    char line3[32];           // Third line of text
    char line4[32];           // Fourth line of text
    
    // UI state properties (not sensor data)
    display_mode_t mode;      // Display mode (main or text)
} display_data_t;

/**
 * @brief Sensor failure types for detailed error reporting
 */
typedef enum {
    SENSOR_FAIL_NONE = 0,          // No failure
    SENSOR_FAIL_O2_COMMUNICATION,  // O2 sensor communication error
    SENSOR_FAIL_O2_OUT_OF_RANGE,   // O2 sensor reading out of valid range
    // Pressure sensor errors removed - using fixed atmospheric pressure
    SENSOR_FAIL_CALIBRATION_INVALID,     // Invalid calibration data
    SENSOR_FAIL_DATA_STALE,             // Data too old
    SENSOR_FAIL_SYSTEM_ERROR            // General system error
} sensor_failure_t;

/**
 * @brief Sensor data structure - raw readings and calculated values for dual O2 sensors
 * Note: Raw readings use integer arithmetic to avoid FPU emulation on ESP32-C3
 */
typedef struct {
    // Raw sensor readings (dual O2 sensors) - INTEGER STORAGE for performance
    int32_t o2_sensor1_reading_mv;  // Raw O2 sensor #1 reading in millivolts (no FPU)
    int32_t o2_sensor2_reading_mv;  // Raw O2 sensor #2 reading in millivolts (no FPU)
    // pressure_reading_mv removed - using fixed atmospheric pressure

    // COMPATIBILITY: Legacy float fields (computed from integer values)
    float o2_sensor1_reading_mv_float;    // Float version of o2_sensor1_reading_mv (for compatibility)
    float o2_sensor2_reading_mv_float;    // Float version of o2_sensor2_reading_mv (for compatibility)

    // Calculated values (using double precision for critical calculations) - FLOAT preserved
    double o2_sensor1_ppo2;         // PPO2 from sensor #1 (bar)
    double o2_sensor2_ppo2;         // PPO2 from sensor #2 (bar)
    double o2_calculated_ppo2;      // Average PPO2 value for warnings (bar)
    float pressure_bar;            // Fixed atmospheric pressure (1.013 bar)

    // INTEGER PPO2 VALUES for performance (no FPU)
    int32_t o2_sensor1_ppo2_mbar;   // PPO2 from sensor #1 (mbar, integer)
    int32_t o2_sensor2_ppo2_mbar;   // PPO2 from sensor #2 (mbar, integer)
    int32_t o2_calculated_ppo2_mbar; // Average PPO2 value for warnings (mbar, integer)

    // Sensor validity flags
    bool sensor1_valid;            // Sensor #1 data validity
    bool sensor2_valid;            // Sensor #2 data validity

    // Battery data - PRIMARY INTEGER STORAGE for performance
    int32_t battery_voltage_mv;    // Battery voltage in millivolts (no FPU)
    uint8_t battery_percentage;    // Battery charge percentage (0-100)
    bool battery_low;              // Low battery warning flag

    // COMPATIBILITY: Legacy float field (computed from integer values)
    float battery_voltage_v;       // Computed from battery_voltage_mv (for compatibility)
    
    // Metadata
    uint32_t timestamp_ms;         // Reading timestamp
    bool valid;                    // Overall data validity flag (at least one sensor working)
    sensor_failure_t failure_type; // Specific failure type if not valid
    uint32_t consecutive_failures; // Count of consecutive failures
} sensor_data_t;

/**
 * @brief Battery status data
 * Note: Using integer millivolts to avoid FPU operations
 */
typedef struct {
    int32_t voltage_mv;       // Battery voltage in millivolts (integer)
    uint8_t percentage;       // Charge percentage (0-100)
    bool charging;            // Charging status
    bool low_battery;         // Low battery warning
} battery_data_t;

/**
 * @brief O2 sensor calibration data for simplified calibration
 */
typedef struct {
    float calibration_gas_o2_fraction;  // O2 fraction of gas used for calibration (0.18-1.0)
    // float calibration_sensor_mv;        // Sensor voltage reading during calibration (mV) - COMMENTED OUT: conflicts with modern calibration system
    bool calibrated;                     // Calibration validity flag
} o2_calibration_t;

/**
 * @brief Sensor voltage boundary constants
 * Note: Raw sensor constants now in microvolts for integer arithmetic
 */
#define SENSOR_VOLTAGE_MIN_UV           0           // Minimum reasonable sensor voltage (microvolts)
#define SENSOR_VOLTAGE_MAX_UV           1000000     // Maximum reasonable sensor voltage (microvolts)
#define CALIBRATION_VOLTAGE_MIN_UV      1000        // Minimum calibration voltage (microvolts)
#define CALIBRATION_VOLTAGE_MAX_UV      1000000     // Maximum calibration voltage (microvolts)
#define ADC_REF_VOLTAGE_UV              1100000     // ADC reference voltage (microvolts) for conversion
#define ADC_MAX_VALUE                   4095        // 12-bit ADC maximum value (integer)

// Direct raw ADC conversion constants (32-bit safe, high precision)
// Exact calculation: 1100000 / 4095 = 268.615...
// Use fractional approach: (raw * 268616) / 1000 for better accuracy
#define ADC_RAW_TO_UV_NUMERATOR         268616      // High precision numerator
#define ADC_RAW_TO_UV_DENOMINATOR       1000        // Scaling factor
#define ADC_RAW_TO_UV_MULTIPLIER        269         // Simple multiplier (fallback)

// Legacy float constants - kept for calibration system compatibility
#define SENSOR_VOLTAGE_MIN_MV           0.0f        // Minimum reasonable sensor voltage (mV)
#define SENSOR_VOLTAGE_MAX_MV           1000.0f     // Maximum reasonable sensor voltage (mV)
#define CALIBRATION_VOLTAGE_MIN_MV      1.0f        // Minimum calibration voltage (mV)
#define CALIBRATION_VOLTAGE_MAX_MV      1000.0f     // Maximum calibration voltage (mV)
#define ADC_REF_VOLTAGE_MV              1100.0f     // ADC reference voltage (mV) for uncalibrated conversion
#define ADC_MAX_VALUE_FLOAT             4095.0f     // 12-bit ADC maximum value (float)
#define PPO2_MIN_CLAMP                  0.0         // Minimum PPO2 clamp value
#define PPO2_MAX_CLAMP                  5.0         // Maximum PPO2 clamp value

/**
 * @brief Sensor calibration validation constants
 */
#define SENSOR_RAW_MIN_MV               0.1f        // Minimum valid raw sensor reading (mV)
#define SENSOR_RAW_MAX_MV               200.0f      // Maximum valid raw sensor reading (mV)
#define CALIBRATION_OFFSET_TOLERANCE_MV 10.0f       // Relaxed offset tolerance for single-point calibration (mV)
#define CALIBRATION_SENS_MIN_MV_PER_BAR 20.0f       // Minimum acceptable sensitivity (mV/bar)
#define CALIBRATION_SENS_MAX_MV_PER_BAR 150.0f      // Maximum acceptable sensitivity (mV/bar)

/**
 * @brief Test mode constants for emulation boards
 */
//#define SENSOR_EMULATION_MODE                       // Uncomment to enable emulation mode
#ifdef SENSOR_EMULATION_MODE
#define SENSOR_VOLTAGE_SCALE_FACTOR     10.0f       // Divide emulated readings by this factor (130mV -> 13mV)
#else
#define SENSOR_VOLTAGE_SCALE_FACTOR     1.0f        // No scaling for real hardware
#endif

/**
 * @brief System performance and memory management constants
 */
#define SYSTEM_HEAP_WARNING_THRESHOLD   10000       // Memory warning threshold (bytes)
#define SYSTEM_HEAP_CRITICAL_THRESHOLD  5000        // Critical memory threshold (bytes)
#define SYSTEM_MAIN_LOOP_DELAY_MS       50          // Main loop delay (ms) - 20Hz update rate
#define SYSTEM_STATUS_LOG_INTERVAL      200         // Status logging interval (loop count)

/**
 * @brief Sensor recovery and timeout constants
 */
#define SENSOR_FAILURE_THRESHOLD        5           // Consecutive failures before recovery
#define SENSOR_RECOVERY_MAX_ATTEMPTS    3           // Maximum recovery attempts before escalation
#define SENSOR_RECOVERY_RESET_THRESHOLD 5           // Recovery attempts before reset
#define SENSOR_READ_TIMEOUT_MS          10000       // Sensor read timeout (ms)
#define SENSOR_READ_CRITICAL_TIMEOUT_MS 30000       // Critical sensor read timeout (ms)
#define RUNTIME_UPDATE_INTERVAL_MS      1000        // Runtime counter update interval (ms)

/**
 * @brief O2 calibration limits
 */
#define O2_PERCENT_MIN                  10.0f       // Minimum O2 percentage
#define O2_PERCENT_MAX                  99.0f       // Maximum O2 percentage
#define O2_PERCENT_DEFAULT_AIR          21.0f       // Default air O2 percentage

/**
 * @brief Display update thresholds
 */
#define DISPLAY_FORCE_UPDATE_TIME_MS    100         // Force display update time after state change (ms)

/**
 * @brief Button timing constants
 */
#define BUTTON_DEBOUNCE_MS              50          // Button debounce time (ms)
#define BUTTON_LONG_PRESS_MS            500        // Long press threshold (ms)
#define BUTTON_DOUBLE_PRESS_MS          300         // Double press timing window (ms)

/**
 * @brief Sensor calibration quality thresholds
 */
#define SENSOR_MIN_SENSITIVITY_MV_BAR   45.0f       // Minimum acceptable sensitivity (mV/bar)
#define SENSOR_MAX_SENSITIVITY_MV_BAR   75.0f       // Maximum acceptable sensitivity (mV/bar)
#define SENSOR_MAX_OFFSET_MV            2.0f        // Maximum offset magnitude (mV)
#define SENSOR_MIN_CORRELATION_R2       0.995f      // Minimum calibration correlation coefficient
#define SENSOR_CAUTION_DEGRADATION      0.80f       // 80% sensitivity = caution threshold
#define SENSOR_FAIL_DEGRADATION         0.70f       // 70% sensitivity = failure threshold
#define SENSOR_O2_PERCENT_MIN           10.0f       // Minimum valid O2 percentage
#define SENSOR_O2_PERCENT_MAX           100.0f      // Maximum valid O2 percentage
#define SENSOR_RECOVERY_TIMEOUT_MS      10000       // Sensor recovery attempt interval (ms)

/**
 * @brief PPO2 warning and alarm thresholds
 */
#define PPO2_LOW_WARNING_DEFAULT        0.18f       // Low PPO2 warning threshold (bar)
#define PPO2_LOW_ALARM_DEFAULT          0.16f       // Low PPO2 alarm threshold (bar)
#define PPO2_HIGH_WARNING_DEFAULT       1.40f       // High PPO2 warning threshold (bar)
#define PPO2_HIGH_ALARM_DEFAULT         1.60f       // High PPO2 alarm threshold (bar)
#define PPO2_DISAGREEMENT_THRESHOLD     0.1f       // Sensor disagreement threshold (bar)

/**
 * @brief Warning system timing constants
 */
#define WARNING_BLINK_PERIOD_MS         250         // Warning blink rate (ms) - 2Hz
#define ALARM_BLINK_PERIOD_MS           125         // Alarm blink rate (ms) - 4Hz

/**
 * @brief Logger configuration constants
 */
#define LOGGER_MAX_SENSOR_ID            2           // Maximum number of sensors (0-1)
#define LOGGER_RECENT_ENTRIES_LIMIT     10          // Number of recent entries to display

/**
 * @brief Date and time structure
 */
typedef struct {
    uint16_t year;    // Year (2024-2099)
    uint8_t month;    // Month (1-12)
    uint8_t day;      // Day (1-31)
    uint8_t hour;     // Hour (0-23)
    uint8_t minute;   // Minute (0-59)
    uint8_t second;   // Second (0-59)
} datetime_t;

/**
 * @brief Application configuration with dual O2 sensor support
 */
typedef struct {
    // PPO2 alarm and warning limits
    float ppo2_low_warning;       // Low PPO2 warning threshold (bar)
    float ppo2_low_alarm;         // Low PPO2 alarm threshold (bar)
    float ppo2_high_warning;      // High PPO2 warning threshold (bar)
    float ppo2_high_alarm;        // High PPO2 alarm threshold (bar)

    // Sensor monitoring settings
    float sensor_disagreement_threshold; // PPO2 difference threshold between sensors (bar)
    float atmospheric_pressure;   // Current atmospheric pressure (bar)
    
    // Display settings
    uint8_t display_brightness;   // Display brightness (0-100)
    uint8_t display_contrast;     // Display contrast (0-100)
    bool auto_brightness;         // Auto brightness enable
    
    // System settings
    uint32_t sleep_timeout_s;     // Sleep timeout in seconds
    datetime_t current_datetime;  // Current date and time
    
    // Dual O2 sensor configuration
    o2_calibration_t o2_cal;        // O2 sensor #1 calibration data
    o2_calibration_t o2_cal_sensor2; // O2 sensor #2 calibration data
} app_config_t;

/**
 * @brief Menu item structure
 */
typedef struct {
    const char* title;            // Menu item display text
    app_mode_t target_mode;       // Mode to enter when selected
} menu_item_t;

/**
 * @brief Battery constants - INTEGER for performance on ESP32-C3
 * Note: Voltage divider calculation: actual_mV = adc_mV * 326 / 100 (3.26 ratio)
 */
#define BATTERY_VOLTAGE_DIVIDER_RATIO_NUM     326       // 3.26 * 100 for integer math
#define BATTERY_VOLTAGE_DIVIDER_RATIO_DENOM   100       // Denominator for 3.26 ratio
#define BATTERY_FULL_VOLTAGE_MV               3300      // Full charge voltage (millivolts)
#define BATTERY_LOW_VOLTAGE_MV                2800      // Low voltage threshold (millivolts)
#define BATTERY_HALF_VOLTAGE_MV               3100      // Half charge voltage (millivolts)

// Battery calculation bounds (32-bit overflow protection)
#define BATTERY_MAX_RAW_UV                    1200000   // Max expected battery ADC reading (1.2V)

// Legacy float constants - kept for any remaining float code
#define BATTERY_VOLTAGE_DIVIDER_RATIO   3.26f        // Input voltage = measured * 3.26
#define BATTERY_FULL_VOLTAGE_V          3.3f        // Full charge voltage
#define BATTERY_LOW_VOLTAGE_V           2.8f        // Low voltage threshold
#define BATTERY_HALF_VOLTAGE_V          3.1f        // Half charge voltage

/**
 * @brief Conversion helper macros for integer/float boundaries
 */
#define MV_TO_V_FLOAT(mv)       ((mv) / 1000.0f)     // Convert millivolts to volts float

/**
 * @brief Integer display formatting functions (no FPU required)
 */

/**
 * @brief Format sensor voltage for display: "12.34 mV"
 * @param microvolts Sensor voltage in microvolts (integer)
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 */
static inline void format_sensor_voltage_display(int32_t microvolts, char *buffer, size_t buffer_size) {
    int32_t millivolts = microvolts / 1000;
    int32_t fractional = (microvolts % 1000) / 10;  // 2 decimal places
    snprintf(buffer, buffer_size, "%ld.%02ld mV", millivolts, fractional);
}

/**
 * @brief Format battery voltage for display: "3.21 V"
 * @param millivolts Battery voltage in millivolts (integer)
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 */
static inline void format_battery_voltage_display(int32_t millivolts, char *buffer, size_t buffer_size) {
    // Handle invalid/uninitialized values
    if (millivolts < 0 || millivolts > 10000) {  // Outside reasonable range (0-10V)
        snprintf(buffer, buffer_size, "?.?? V");  // Show error indicator
        return;
    }

    int32_t volts = millivolts / 1000;
    int32_t fractional = (millivolts % 1000) / 10;  // 2 decimal places
    snprintf(buffer, buffer_size, "%ld.%02ld V", volts, fractional);
}

/**
 * @brief Format battery percentage for display: "85%"
 * @param percentage Battery percentage (0-100)
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 */
static inline void format_battery_percentage_display(uint8_t percentage, char *buffer, size_t buffer_size) {
    snprintf(buffer, buffer_size, "%d%%", percentage);
}

/**
 * @brief Format PPO2 value for display: "1.23 bar" (still uses float from calibration)
 * @param ppo2_bar PPO2 value in bar (double from calibration system)
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 */
static inline void format_ppo2_display(double ppo2_bar, char *buffer, size_t buffer_size) {
    snprintf(buffer, buffer_size, "%.2f bar", ppo2_bar);
}

/**
 * @brief Log error message to both ESP_LOGE and display warning label
 * @param tag Log tag (component name)
 * @param format Printf-style format string
 * @param ... Format arguments
 */
void app_log_error_to_display(const char* tag, const char* format, ...);

#ifdef __cplusplus
}
#endif

#endif // APP_TYPES_H