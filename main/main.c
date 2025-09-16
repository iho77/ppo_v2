/**
 * @file main.c
 * @brief PPO2 HUD main application - Simplified architecture
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include <math.h>
#include <string.h>

// Application components
#include "button_manager.h"
#include "display_manager.h"
#include "sensor_manager.h"
#include "sensor_calibration.h"  // For sensor health enums
#include "ppo2_logger.h"          // PPO2 measurement logging
// Using internal ADC and no external sensor managers
#include "warning_manager.h"
#include "app_types.h"
#include "app_config.h"
#include "driver/i2c_master.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"
#include "esp_task_wdt.h"

static const char *TAG = "PPO2_HUD";

// ========== HARDWARE CONFIGURATION ==========
// GPIO Pin Assignments for ESP32-C3 Development Board
#define GPIO_I2C_SDA            GPIO_NUM_6  // I2C Data line
#define GPIO_I2C_SCL            GPIO_NUM_7  // I2C Clock line  
#define GPIO_BUTTON_MODE        GPIO_NUM_5  // Mode/menu button
#define GPIO_BUTTON_SELECT      GPIO_NUM_10  // Select/enter button
#define GPIO_WARNING_LED        GPIO_NUM_8  // Built-in RGB LED


// I2C bus configuration
#define I2C_BUS_SDA_PIN         GPIO_I2C_SDA
#define I2C_BUS_SCL_PIN         GPIO_I2C_SCL
#define I2C_BUS_FREQ_HZ         200000
#define I2C_BUS_PORT            I2C_NUM_0

// I2C Device addresses
// Display I2C address is handled by display_manager (uses 0x3D default for SH1107)
// Using internal ADC - no external I2C sensor addresses needed

// Warning system configuration
#define WARNING_LED_GPIO        GPIO_WARNING_LED

// Application states (simplified from complex mode system)
typedef enum {
    APP_STATE_MAIN = 0,        // Primary diving display
    APP_STATE_MENU,            // Menu overlay
    APP_STATE_CALIBRATION,     // Calibration overlay
    APP_STATE_CAL_RESET,       // Calibration reset screen
    APP_STATE_SETUP,           // Setup overlay
    APP_STATE_LOG,             // Log viewer overlay  
    APP_STATE_PRINT_CALIBRATION, // Print calibration to debug and show message
    APP_STATE_SENSOR_HEALTH,   // Print sensor health to debug and show message
    APP_STATE_SYSTEM_MESSAGE,  // System message display
    APP_STATE_MAX
} app_state_t;

// Setup mode menu items
typedef enum {
    SETUP_ITEM_WARN_LOW_PPO2 = 0,
    SETUP_ITEM_ALARM_LOW_PPO2,
    SETUP_ITEM_WARN_HIGH_PPO2,
    SETUP_ITEM_ALARM_HIGH_PPO2,
    SETUP_ITEM_SENSOR_DISAGREEMENT,
    SETUP_ITEM_ATMOSPHERIC_PRESSURE,
    SETUP_ITEM_SAVE,
    SETUP_ITEM_BACK,
    SETUP_ITEM_COUNT
} setup_item_t;

// Display update thresholds
#define PPO2_DISPLAY_UPDATE_THRESHOLD 0.02f  // Update display if PPO2 changes by >= 0.02
#define DISPLAY_MAX_STALENESS_MS      1000   // Always refresh at least once per second

// Default sensor values for calibration reset
#define DEFAULT_SENSOR0_AIR_MV 10.0f  // Default sensor 0 voltage in air (mV)
#define DEFAULT_SENSOR1_AIR_MV 10.0f   // Default sensor 1 voltage in air (mV)
#define DEFAULT_SENSOR0_SERIAL "S000001"  // Default sensor 0 serial number
#define DEFAULT_SENSOR1_SERIAL "S000002"  // Default sensor 1 serial number

// Application context
typedef struct {
    app_state_t current_state;
    app_state_t previous_state;
    uint32_t state_entry_time;
    uint8_t menu_selected_item;
    uint8_t menu_item_count;
    // Calibration mode state (dual sensor simultaneous calibration)
    float calibration_o2_percent;  // Known O2 percentage for calibration
    uint8_t calibration_menu_item;  // Current menu item being navigated
    uint8_t calibration_selected_gas; // Selected gas type for calibration (1=Air, 2=O2, 3=Custom)
    bool calibration_custom_editing; // True when editing custom percentage
    bool calibration_in_action_mode; // True when navigating calibrate/back, false when selecting gas
    bool calibration_session_active; // True if first gas has been calibrated, waiting for second gas
    float calibration_first_gas_percent; // O2 percentage of first calibrated gas
    // Setup mode state
    uint8_t setup_selected_item;   // Current setup menu item
    bool setup_editing;            // True when editing a value
    uint8_t setup_digit_index;     // Current digit being edited (0-based)
    uint8_t setup_digit_count;     // Total number of digits for current value
    app_config_t setup_temp_config; // Temporary config for editing
    // Calibration reset mode state
    uint8_t cal_reset_menu_item;   // 0=reset live, 1=reset default, 2=back
    float current_sensor0_mv;      // Current sensor 0 reading (mV)
    float current_sensor1_mv;      // Current sensor 1 reading (mV)
    // System message state
    char system_message[256];      // Current system message text
} app_context_t;

// Global application context
static app_context_t s_app_ctx = {
    .current_state = APP_STATE_MAIN,
    .previous_state = APP_STATE_MAIN,
    .menu_selected_item = 0,
    .menu_item_count = 6, // Will be updated dynamically in app_main()
    .calibration_o2_percent = O2_PERCENT_DEFAULT_AIR,
    .calibration_menu_item = 1,
    .calibration_selected_gas = 1, // Start with Air selected
    .calibration_custom_editing = false,
    .calibration_in_action_mode = false, // Start in gas selection mode
    .calibration_session_active = false,
    .calibration_first_gas_percent = 0.0f,
    .setup_selected_item = 0,
    .setup_editing = false,
    .setup_digit_index = 0,
    .setup_digit_count = 0,
    .cal_reset_menu_item = 0,
    .current_sensor0_mv = 0.0f,
    .current_sensor1_mv = 0.0f,
    .system_message = ""
};

// Global I2C bus handle for display (new driver)
static i2c_master_bus_handle_t s_i2c_bus = NULL;

// Application runtime state (simplified) with watchdog
static struct {
    uint32_t runtime_seconds;
    uint32_t last_update_time;
    uint32_t last_successful_sensor_read;
    uint32_t sensor_recovery_attempts;
    bool in_recovery_mode;
} s_runtime_state = {
    .runtime_seconds = 0,
    .last_update_time = 0,
    .last_successful_sensor_read = 0,
    .sensor_recovery_attempts = 0,
    .in_recovery_mode = false
};

// Display change-detection cache
static struct {
    bool initialized;
    float last_s1_ppo2;
    float last_s2_ppo2;
    char last_cal_status_s1[8];
    char last_cal_status_s2[8];
    bool state_changed; // force redraw on state transitions
    uint32_t last_ui_update_ms;
    bool last_battery_low;
    bool last_valid;
    sensor_failure_t last_failure_type;
} s_display_cache = {
    .initialized = false,
    .last_s1_ppo2 = 0.0f,
    .last_s2_ppo2 = 0.0f,
    .last_cal_status_s1 = "",
    .last_cal_status_s2 = "",
    .state_changed = false,
    .last_ui_update_ms = 0,
    .last_battery_low = false,
    .last_valid = true,
    .last_failure_type = SENSOR_FAIL_NONE
};

// Menu items - updated to match new UI architecture
static const menu_item_t s_menu_items[] = {
    {"sensor calibration", MODE_CALIBRATION},
    {"reset calibration", MODE_CAL_RESET},
    {"print logs", MODE_LOG},
    {"sensor health", MODE_SENSOR_HEALTH},
    {"setup", MODE_SETUP},
    {"power off", MODE_SLEEP},
    {"back", MODE_MAIN}
};

// Function declarations
static esp_err_t init_i2c_bus(void);
static void handle_button_events(void);
static void update_runtime_state(void);
static void update_display(void);
static void show_main_display(void);
static void show_menu_overlay(void);
static void show_calibration_overlay(void);
static void show_calibration_reset_overlay(void);
static void show_setup_overlay(void);
static void show_print_calibration_overlay(void);
static void show_sensor_health_overlay(void);
static void show_system_message_overlay(void);
static void enter_state(app_state_t new_state);
static const char* state_to_string(app_state_t state);
static void calibration_increment_custom_o2(void);
static void calibration_perform_with_o2(float o2_percent);
static void setup_increment_digit(void);
static void setup_save_config(void);
static void setup_get_digit_info(uint8_t item, uint8_t *digit_count);
static void format_ppo2_item(display_data_t *display_data, const char *title, float value);
static void increment_ppo2_digit(float *value);
static void perform_calibration_reset_all(void);
static void perform_calibration_reset_sensor_0(void);
static void perform_calibration_reset_sensor_1(void);
static void update_calibration_reset_readings(void);
static void attempt_sensor_recovery(void);
static void check_system_watchdog(void);
static void enter_deep_sleep_mode(void);



void app_main(void)
{
    // Check wake-up cause and force reinitialization if waking from deep sleep
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_GPIO: {
            uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                ESP_LOGI(TAG, "Wake-up caused by GPIO %d - forcing manager reinitialization", pin);
            } else {
                ESP_LOGI(TAG, "Wake-up caused by GPIO - forcing manager reinitialization");
            }
            
            // Force reinitialization of managers after deep sleep
            // Static variables retain their values, but hardware state is reset
            ESP_LOGI(TAG, "Deinitializing managers to force clean reinitialization");
            display_manager_deinit();
            sensor_manager_deinit();
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            ESP_LOGI(TAG, "Wake-up not caused by deep sleep (normal boot)");
            break;
    }
    
    ESP_LOGI(TAG, "Starting PPO2 HUD application (Simplified Architecture)");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Free heap: %d bytes", esp_get_free_heap_size());

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize I2C systems
    ESP_LOGI(TAG, "Initializing I2C systems...");
    ESP_ERROR_CHECK_WITHOUT_ABORT(init_i2c_bus());  // For both display and ADS1115 (new driver)

    // Using internal ADC for O2 sensors - no external sensor initialization needed

    // Initialize core components
    ESP_LOGI(TAG, "Initializing components...");
    
    esp_err_t app_config_ret = app_config_init();
    if (app_config_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize app config: %s", esp_err_to_name(app_config_ret));
    }
    
    esp_err_t sensor_mgr_ret = sensor_manager_init();
    if (sensor_mgr_ret != ESP_OK) {
        app_log_error_to_display(TAG, "Failed to initialize sensor manager: %s", esp_err_to_name(sensor_mgr_ret));
    } else {
        ESP_LOGI(TAG, "Sensor manager initialized successfully");
    }
    
    // Initialize PPO2 logger (depends on sensor manager)
    esp_err_t ppo2_logger_ret = ppo2_logger_init();
    if (ppo2_logger_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize PPO2 logger: %s", esp_err_to_name(ppo2_logger_ret));
    } else {
        ESP_LOGI(TAG, "PPO2 logger initialized successfully");
    }
    
    // Check if sensor manager can read data (for informational purposes only)
    sensor_data_t test_data;
    esp_err_t sensor_test = sensor_manager_read(&test_data);
    if (sensor_test != ESP_OK || !test_data.valid) {
        if (test_data.failure_type == SENSOR_FAIL_CALIBRATION_INVALID) {
            ESP_LOGW(TAG, "No valid calibrations found - user must reset calibrations through menu");
        } else {
            ESP_LOGW(TAG, "Sensor readings failed - sensors may not be connected");
        }
    } else {
        ESP_LOGI(TAG, "Sensor readings successful, calibrations valid");
    }
    
   
    // Initialize button manager with GPIO configuration
    button_config_t button_config = {
        .mode_gpio = GPIO_BUTTON_MODE,
        .select_gpio = GPIO_BUTTON_SELECT
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(button_manager_init(&button_config));

    // Initialize warning manager with built-in LED
    ESP_LOGI(TAG, "Initializing warning manager...");
    warning_config_t warning_config = {
        .led_gpio = WARNING_LED_GPIO,
        .use_led_strip = true  // ESP32-C3 has addressable LED
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(warning_manager_init(&warning_config));

    // Initialize display manager with new I2C master driver
    display_config_t display_config = {
        .i2c_bus = s_i2c_bus,
        .i2c_address = 0,  // Use display manager's default (0x3D for SH1107)
        .i2c_freq_hz = I2C_BUS_FREQ_HZ
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(display_manager_init(&display_config));

    ESP_LOGI(TAG, "All components initialized successfully");

    // Start PPO2 logger after all components are ready
    if (ppo2_logger_ret == ESP_OK) {
        esp_err_t start_ret = ppo2_logger_start();
        if (start_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to start PPO2 logger: %s", esp_err_to_name(start_ret));
        } else {
            ESP_LOGI(TAG, "PPO2 logger started - logging every minute");
        }
    }

    // Add current task to existing watchdog for safety monitoring
    esp_err_t wdt_ret = esp_task_wdt_add(NULL);  // Add current task to existing watchdog
    if (wdt_ret == ESP_OK) {
        ESP_LOGI(TAG, "Main task added to hardware watchdog");
    } else {
        ESP_LOGW(TAG, "Failed to add task to watchdog: %s", esp_err_to_name(wdt_ret));
    }

    // Display startup message
    display_data_t startup_msg = {0};
    startup_msg.mode = DISPLAY_MODE_TEXT;
    snprintf(startup_msg.line1, sizeof(startup_msg.line1), "HUD v2.0");
    snprintf(startup_msg.line2, sizeof(startup_msg.line2), "2025");
    snprintf(startup_msg.line3, sizeof(startup_msg.line3), "Starting...");
    display_manager_update_system_message("Starting....");
  //  display_manager_update(&startup_msg);
//    display_manager_update_system_message   (&startup_msg);
    
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Initialize application state
    s_app_ctx.current_state = APP_STATE_MAIN;
    s_app_ctx.state_entry_time = esp_timer_get_time() / 1000;
    s_app_ctx.menu_item_count = sizeof(s_menu_items) / sizeof(s_menu_items[0]);

    ESP_LOGI(TAG, "Starting main application loop in %s state", state_to_string(s_app_ctx.current_state));

    // Initialize recovery state
    s_runtime_state.last_successful_sensor_read = esp_timer_get_time() / 1000;
    
    // Main application loop with enhanced safety monitoring
    uint32_t loop_count = 0;
    const TickType_t loop_period = pdMS_TO_TICKS(SYSTEM_MAIN_LOOP_DELAY_MS);
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        // Feed watchdog to prevent system reset
        esp_task_wdt_reset();
        
        // Update hardware (always active)
        button_manager_update();
        sensor_manager_update();
        
        // Update warning system based on latest sensor data
        sensor_data_t sensor_data;
        esp_err_t sensor_ret = sensor_manager_read(&sensor_data);
        if (sensor_ret == ESP_OK && sensor_data.valid) {
            s_runtime_state.last_successful_sensor_read = esp_timer_get_time() / 1000;
            s_runtime_state.in_recovery_mode = false;
            warning_manager_update(&sensor_data);
        } else {
            // Sensor read failed or data invalid - check for recovery
            check_system_watchdog();
        }
        
        // Handle button events
        handle_button_events();
        
        // Update runtime state (simplified)
        update_runtime_state();
        
        // Update display based on current state
        update_display();
        
        // Periodic logging and health checks
        if ((++loop_count % SYSTEM_STATUS_LOG_INTERVAL) == 0) {  // Every 10 seconds at 20Hz
            ESP_LOGI(TAG, "Loop #%lu in %s state, heap: %d bytes, recovery: %s", 
                     loop_count, state_to_string(s_app_ctx.current_state), 
                     esp_get_free_heap_size(), s_runtime_state.in_recovery_mode ? "YES" : "NO");
                     
            // Check for memory leaks
            if (esp_get_free_heap_size() < SYSTEM_HEAP_WARNING_THRESHOLD) {  // Memory warning threshold
                ESP_LOGW(TAG, "Low memory warning: %d bytes free", esp_get_free_heap_size());
            }
        }
        
        vTaskDelayUntil(&last_wake, loop_period);  // 20Hz update rate, no drift
    }

    // Cleanup (never reached)
    ESP_LOGI(TAG, "Shutting down application");
    sensor_manager_deinit();
    // No external sensor managers to deinitialize
    warning_manager_deinit();
    display_manager_deinit();
    button_manager_deinit();
    
    // Cleanup I2C systems
    if (s_i2c_bus) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }
}

// Initialize I2C master bus for display
static esp_err_t init_i2c_bus(void)
{
    ESP_LOGI(TAG, "Initializing I2C master bus (SDA: %d, SCL: %d)", I2C_BUS_SDA_PIN, I2C_BUS_SCL_PIN);

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = I2C_BUS_SDA_PIN,
        .scl_io_num = I2C_BUS_SCL_PIN,
        .flags.enable_internal_pullup = false,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &s_i2c_bus);
    if (ret != ESP_OK) {
        app_log_error_to_display(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C master bus initialized successfully");
    return ESP_OK;
}

// Centralized button handling - much simpler than mode manager approach
static void handle_button_events(void)
{
    button_event_t mode_event = button_manager_get_event(BUTTON_MODE);
    button_event_t select_event = button_manager_get_event(BUTTON_SELECT);

    // Handle MODE button long press first (universal exit from editing modes)
    if (mode_event == BUTTON_EVENT_LONG_PRESS) {
        if (s_app_ctx.current_state == APP_STATE_MAIN) {
            // Long press from main: enter menu
            ESP_LOGI(TAG, "MODE long press - entering menu");
            enter_state(APP_STATE_MENU);
        } else if (s_app_ctx.current_state == APP_STATE_SETUP && s_app_ctx.setup_editing) {
            // Long press in setup editing mode: force exit editing (emergency exit)
            ESP_LOGI(TAG, "MODE long press - force exit setup editing mode");
            s_app_ctx.setup_editing = false;
            s_app_ctx.setup_digit_index = 0;
        } else if (s_app_ctx.current_state == APP_STATE_CALIBRATION && s_app_ctx.calibration_custom_editing) {
            // Long press in calibration editing mode: force exit editing
            ESP_LOGI(TAG, "MODE long press - force exit calibration editing mode");
            s_app_ctx.calibration_custom_editing = false;
        } else {
            // Long press in other states: return to main (universal escape)
            ESP_LOGI(TAG, "MODE long press - universal escape to main");
            enter_state(APP_STATE_MAIN);
        }
    } else if (mode_event == BUTTON_EVENT_PRESS) {
        switch (s_app_ctx.current_state) {
            case APP_STATE_MAIN:
                // Short press in main: manual sensor update
                ESP_LOGI(TAG, "MODE press - manual sensor update");
                sensor_manager_update();
                break;
            case APP_STATE_MENU:
                // MODE press in menu: enter selected mode
                if (s_menu_items[s_app_ctx.menu_selected_item].target_mode == MODE_MAIN) {
                    enter_state(APP_STATE_MAIN);
                } else {
                    // Map mode enum to state enum for menu items
                    app_state_t target_state = APP_STATE_MAIN;
                    switch (s_menu_items[s_app_ctx.menu_selected_item].target_mode) {
                        case MODE_CALIBRATION: target_state = APP_STATE_CALIBRATION; break;
                        case MODE_CAL_RESET: target_state = APP_STATE_CAL_RESET; break;
                        case MODE_SETUP: target_state = APP_STATE_SETUP; break;
                        case MODE_LOG: 
                            // Print logs immediately and return to main
                            ESP_LOGI(TAG, "SELECT press in LOG - printing all logs");
                            
                            // Print calibration logs for both sensors
                            ESP_LOGI(TAG, "=== PRINTING CALIBRATION LOGS ===");
                            sensor_calibration_print_csv_log(255, 0);  // All sensors, all entries
                            
                            // Print PPO2 logs
                            ESP_LOGI(TAG, "=== PRINTING PPO2 LOGS ===");
                            ppo2_logger_print_csv();
                            
                            ESP_LOGI(TAG, "=== ALL LOGS PRINTED ===");
                            
                            // Show confirmation message
                            enter_state(APP_STATE_SYSTEM_MESSAGE);
                            snprintf(s_app_ctx.system_message, sizeof(s_app_ctx.system_message), 
                                     "All logs printed\nto debug console");
                            return; // Don't continue to enter_state below
                        case MODE_PRINT_CALIBRATION: target_state = APP_STATE_PRINT_CALIBRATION; break;
                        case MODE_SENSOR_HEALTH: target_state = APP_STATE_SENSOR_HEALTH; break;
                        case MODE_SLEEP: 
                            // Handle deep sleep mode
                            ESP_LOGI(TAG, "Entering deep sleep mode");
                            enter_deep_sleep_mode();
                            return; // Never reached
                        default: target_state = APP_STATE_MAIN; break;
                    }
                    ESP_LOGI(TAG, "Menu item selected: %s", s_menu_items[s_app_ctx.menu_selected_item].title);
                    enter_state(target_state);
                }
                break;
            case APP_STATE_CALIBRATION:
                // MODE press in calibration: handle two-mode navigation per UI architecture
                if (!s_app_ctx.calibration_in_action_mode) {
                    // Gas selection mode
                    if (s_app_ctx.calibration_menu_item == 3 && !s_app_ctx.calibration_custom_editing) {
                        // Custom item selected but not in edit mode - enter edit mode
                        s_app_ctx.calibration_custom_editing = true;
                        ESP_LOGI(TAG, "MODE press - entering custom O2 edit mode");
                    } else {
                        // Gas selection confirmed - switch to action mode and select the gas
                        s_app_ctx.calibration_selected_gas = s_app_ctx.calibration_menu_item;
                        s_app_ctx.calibration_in_action_mode = true;
                        s_app_ctx.calibration_menu_item = 1; // Start with "calibrate" in action mode
                        ESP_LOGI(TAG, "MODE press - gas type selected: %d, switching to action mode", s_app_ctx.calibration_selected_gas);
                    }
                } else {
                    // Action mode
                    if (s_app_ctx.calibration_menu_item == 1) {
                        // "calibrate" selected - perform calibration with selected gas
                        float selected_o2_percent = O2_PERCENT_DEFAULT_AIR; // Default to air
                        switch (s_app_ctx.calibration_selected_gas) {
                            case 1: selected_o2_percent = O2_PERCENT_DEFAULT_AIR; break;  // Air
                            case 2: selected_o2_percent = 100.0f; break; // O2
                            case 3: selected_o2_percent = s_app_ctx.calibration_o2_percent; break; // Custom
                            default: selected_o2_percent = O2_PERCENT_DEFAULT_AIR; break;
                        }
                        ESP_LOGI(TAG, "MODE press - performing dual sensor calibration with %.1f%% O2", 
                                 selected_o2_percent);
                        calibration_perform_with_o2(selected_o2_percent);
                    } else if (s_app_ctx.calibration_menu_item == 2) {
                        // "back" selected - return to main
                        ESP_LOGI(TAG, "MODE press - returning to main from calibration");
                        enter_state(APP_STATE_MAIN);
                    }
                }
                break;
            case APP_STATE_SETUP:
                // MODE press in setup: save if editing, or navigate/return
                if (s_app_ctx.setup_editing) {
                    if (s_app_ctx.setup_digit_index + 1 < s_app_ctx.setup_digit_count) {
                        // Move to next digit
                        s_app_ctx.setup_digit_index++;
                        ESP_LOGI(TAG, "MODE press - next digit: %d", s_app_ctx.setup_digit_index);
                    } else {
                        // Finished editing all digits - exit edit mode (don't save yet)
                        s_app_ctx.setup_editing = false;
                        s_app_ctx.setup_digit_index = 0;
                        ESP_LOGI(TAG, "MODE press - finished editing");
                    }
                } else if (s_app_ctx.setup_selected_item == SETUP_ITEM_SAVE) {
                    // Save config and return to main
                    setup_save_config();
                    enter_state(APP_STATE_MAIN);
                } else if (s_app_ctx.setup_selected_item == SETUP_ITEM_BACK) {
                    // Exit without saving
                    ESP_LOGI(TAG, "MODE press - exiting setup without saving");
                    enter_state(APP_STATE_MAIN);
                } else {
                    // Enter edit mode for selected item (temp config already loaded)
                    s_app_ctx.setup_editing = true;
                    s_app_ctx.setup_digit_index = 0;
                    setup_get_digit_info(s_app_ctx.setup_selected_item, &s_app_ctx.setup_digit_count);
                    ESP_LOGI(TAG, "MODE press - entering edit mode for item %d", s_app_ctx.setup_selected_item);
                }
                break;
            case APP_STATE_CAL_RESET:
                // MODE press in calibration reset: perform selected action
                switch (s_app_ctx.cal_reset_menu_item) {
                    case 0: // "reset all" - reset both sensors
                        ESP_LOGI(TAG, "MODE press - performing reset all sensors");
                        perform_calibration_reset_all();
                        break;
                    case 1: // "reset S1" - reset sensor 0 only
                        ESP_LOGI(TAG, "MODE press - performing reset sensor 0");
                        perform_calibration_reset_sensor_0();
                        break;
                    case 2: // "reset S2" - reset sensor 1 only
                        ESP_LOGI(TAG, "MODE press - performing reset sensor 1");
                        perform_calibration_reset_sensor_1();
                        break;
                    case 3: // "back" - return to main
                        ESP_LOGI(TAG, "MODE press - returning to main from calibration reset");
                        enter_state(APP_STATE_MAIN);
                        break;
                }
                break;
            case APP_STATE_SYSTEM_MESSAGE:
                // MODE press in system message: return to main
                ESP_LOGI(TAG, "MODE press - returning to main from system message");
                enter_state(APP_STATE_MAIN);
                break;
            default:
                break;
        }
    }

    // Handle SELECT button
    if (select_event == BUTTON_EVENT_PRESS) {
        switch (s_app_ctx.current_state) {
            case APP_STATE_MAIN:
                // SELECT in main: toggle display brightness
                ESP_LOGI(TAG, "SELECT press - brightness toggle");
                break;
            case APP_STATE_MENU:
                // SELECT in menu: switch active menu item (cycle through)
                s_app_ctx.menu_selected_item = (s_app_ctx.menu_selected_item + 1) % s_app_ctx.menu_item_count;
                ESP_LOGI(TAG, "Menu selection: %d (%s)", s_app_ctx.menu_selected_item, s_menu_items[s_app_ctx.menu_selected_item].title);
                break;
            case APP_STATE_CALIBRATION:
                // SELECT in calibration: two-mode navigation per UI architecture
                if (s_app_ctx.calibration_menu_item == 3 && s_app_ctx.calibration_custom_editing) {
                    // Custom item is selected and in edit mode - increment percentage
                    calibration_increment_custom_o2();
                } else if (!s_app_ctx.calibration_in_action_mode) {
                    // Gas selection mode: Navigate Air -> O2 -> Custom
                    s_app_ctx.calibration_menu_item = (s_app_ctx.calibration_menu_item + 1);
                    if (s_app_ctx.calibration_menu_item > 3) {
                        s_app_ctx.calibration_menu_item = 1; // Wrap from Custom to Air
                    }
                    s_app_ctx.calibration_custom_editing = false; // Exit custom editing when switching items
                    ESP_LOGI(TAG, "SELECT - gas selection: %d", s_app_ctx.calibration_menu_item);
                } else {
                    // Action mode: Navigate Calibrate -> Back
                    s_app_ctx.calibration_menu_item = (s_app_ctx.calibration_menu_item == 1) ? 2 : 1;
                    ESP_LOGI(TAG, "SELECT - action selection: %d", s_app_ctx.calibration_menu_item);
                }
                break;
            case APP_STATE_SETUP:
                // SELECT in setup: cycle through items or increment digit
                if (s_app_ctx.setup_editing) {
                    // Increment current digit
                    setup_increment_digit();
                } else {
                    // Navigate through setup menu items
                    s_app_ctx.setup_selected_item = (s_app_ctx.setup_selected_item + 1) % SETUP_ITEM_COUNT;
                    ESP_LOGI(TAG, "SELECT - setup menu item: %d", s_app_ctx.setup_selected_item);
                }
                break;
            case APP_STATE_CAL_RESET:
                // SELECT in calibration reset: cycle through menu options
                s_app_ctx.cal_reset_menu_item = (s_app_ctx.cal_reset_menu_item + 1) % 4; // 0, 1, 2, 3
                ESP_LOGI(TAG, "SELECT - cal reset menu item: %d", s_app_ctx.cal_reset_menu_item);
                break;
            case APP_STATE_SYSTEM_MESSAGE:
                // SELECT in system message: return to main (only option is "back")
                ESP_LOGI(TAG, "SELECT press - returning to main from system message");
                enter_state(APP_STATE_MAIN);
                break;
            default:
                break;
        }
    }
}

// Update runtime counter - simplified state management
static void update_runtime_state(void)
{
    uint32_t current_time = esp_timer_get_time() / 1000; // milliseconds
    
    // Update every second
    if (current_time - s_runtime_state.last_update_time >= RUNTIME_UPDATE_INTERVAL_MS) {
        s_runtime_state.runtime_seconds++;
        s_runtime_state.last_update_time = current_time;
        
        ESP_LOGV(TAG, "Runtime: %lu seconds", s_runtime_state.runtime_seconds);
    }
}

// Single display update function - routes to appropriate display
static void update_display(void)
{
    switch (s_app_ctx.current_state) {
        case APP_STATE_MAIN:
            show_main_display();
            break;
        case APP_STATE_MENU:
            show_menu_overlay();
            break;
        case APP_STATE_CALIBRATION:
            show_calibration_overlay();
            break;
        case APP_STATE_CAL_RESET:
            update_calibration_reset_readings(); // Update current sensor readings
            show_calibration_reset_overlay();
            break;
        case APP_STATE_SETUP:
            show_setup_overlay();
            break;
        case APP_STATE_PRINT_CALIBRATION:
            show_print_calibration_overlay();
            break;
        case APP_STATE_SENSOR_HEALTH:
            show_sensor_health_overlay();
            break;
        case APP_STATE_SYSTEM_MESSAGE:
            show_system_message_overlay();
            break;
        default:
            show_main_display(); // Fallback to main
            break;
    }

    // If we just transitioned state, ensure we only force one redraw
    if (s_display_cache.state_changed) {
        s_display_cache.state_changed = false;
    }
}

// Buffer overflow protection macro
#define SAFE_SNPRINTF(dst, size, fmt, ...) do { \
    int written = snprintf(dst, size, fmt, ##__VA_ARGS__); \
    if (written >= (int)size || written < 0) { \
        ESP_LOGE(TAG, "Buffer overflow detected in snprintf: %d bytes needed, %d available", written, (int)size); \
        dst[size-1] = '\0'; \
    } \
} while(0)

// Main diving display - core functionality with enhanced error handling
static void show_main_display(void)
{
    // Static variables to track previous PPO2 values for threshold comparison
    
    // Get current sensor data from sensor manager
    sensor_data_t sensor_data = {0};
    esp_err_t ret = sensor_manager_read(&sensor_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data for display: %s", esp_err_to_name(ret));
        // Show critical system error using system message
        display_manager_update_system_message("SYSTEM ERROR\nCannot read data\nCheck system");
        return;
    }
    
    // Always update main display, but add error info to sensor data if needed
    if (!sensor_data.valid) {
        // For intermittent failures, show main display with error info
        // Only show full error screen for critical/persistent failures
        if (sensor_data.consecutive_failures >= SENSOR_FAILURE_THRESHOLD) {
            // Persistent failure - show full error screen
            display_data_t failure_display = {0};
            failure_display.mode = DISPLAY_MODE_TEXT;
            
            switch (sensor_data.failure_type) {
                case SENSOR_FAIL_O2_COMMUNICATION:
                    SAFE_SNPRINTF(failure_display.line1, sizeof(failure_display.line1), "O2 SENSOR FAIL");
                    SAFE_SNPRINTF(failure_display.line2, sizeof(failure_display.line2), "Comm Error");
                    SAFE_SNPRINTF(failure_display.line3, sizeof(failure_display.line3), "USING FAIL-SAFE");
                    break;
                case SENSOR_FAIL_CALIBRATION_INVALID:
                    SAFE_SNPRINTF(failure_display.line1, sizeof(failure_display.line1), "CALIBRATION");
                    SAFE_SNPRINTF(failure_display.line2, sizeof(failure_display.line2), "INVALID");
                    SAFE_SNPRINTF(failure_display.line3, sizeof(failure_display.line3), "RECALIBRATE");
                    break;
                default:
                    SAFE_SNPRINTF(failure_display.line1, sizeof(failure_display.line1), "SENSOR FAIL");
                    SAFE_SNPRINTF(failure_display.line2, sizeof(failure_display.line2), "Multiple errors");
                    SAFE_SNPRINTF(failure_display.line3, sizeof(failure_display.line3), "PPO2: %.2f", sensor_data.o2_calculated_ppo2);
                    break;
            }
            
            // Convert failure display to system message format
            char failure_msg[256];
            snprintf(failure_msg, sizeof(failure_msg), "%s\n%s\n%s", 
                     failure_display.line1, failure_display.line2, failure_display.line3);
            display_manager_update_system_message(failure_msg);
            return;
        }
    }
    // 1 Hz UI policy: update LVGL strictly every DISPLAY_MAX_STALENESS_MS
    const float s1 = sensor_data.o2_sensor1_ppo2;
    const float s2 = sensor_data.o2_sensor2_ppo2;
    const char *cal1 = sensor_manager_get_calibration_display_status(0);
    const char *cal2 = sensor_manager_get_calibration_display_status(1);
    char cur_cal1[8] = {0};
    char cur_cal2[8] = {0};
    if (cal1) strncpy(cur_cal1, cal1, sizeof(cur_cal1) - 1); else strncpy(cur_cal1, "na", sizeof(cur_cal1) - 1);
    if (cal2) strncpy(cur_cal2, cal2, sizeof(cur_cal2) - 1); else strncpy(cur_cal2, "na", sizeof(cur_cal2) - 1);

    bool need_update = false;
    uint32_t now_ms = esp_timer_get_time() / 1000; // milliseconds
    if (!s_display_cache.initialized) {
        need_update = true;
    } else {
        // Time-based refresh (strict 1 Hz)
        if ((now_ms - s_display_cache.last_ui_update_ms) >= DISPLAY_MAX_STALENESS_MS) need_update = true;
        // Critical status changes
        if (strcmp(cur_cal1, s_display_cache.last_cal_status_s1) != 0) need_update = true;
        if (strcmp(cur_cal2, s_display_cache.last_cal_status_s2) != 0) need_update = true;
        if (sensor_data.battery_low != s_display_cache.last_battery_low) need_update = true;
        if (sensor_data.valid != s_display_cache.last_valid) need_update = true;
        if (sensor_data.failure_type != s_display_cache.last_failure_type) need_update = true;
    }

    // Force a full redraw once after state transitions to MAIN
    if (s_display_cache.state_changed) {
        need_update = true;
    }

    if (need_update) {
        display_manager_update_main(&sensor_data);
        s_display_cache.initialized = true;
        s_display_cache.last_s1_ppo2 = s1;
        s_display_cache.last_s2_ppo2 = s2;
        strncpy(s_display_cache.last_cal_status_s1, cur_cal1, sizeof(s_display_cache.last_cal_status_s1) - 1);
        s_display_cache.last_cal_status_s1[sizeof(s_display_cache.last_cal_status_s1) - 1] = '\0';
        strncpy(s_display_cache.last_cal_status_s2, cur_cal2, sizeof(s_display_cache.last_cal_status_s2) - 1);
        s_display_cache.last_cal_status_s2[sizeof(s_display_cache.last_cal_status_s2) - 1] = '\0';
        s_display_cache.state_changed = false; // consumed
        s_display_cache.last_ui_update_ms = now_ms;
        s_display_cache.last_battery_low = sensor_data.battery_low;
        s_display_cache.last_valid = sensor_data.valid;
        s_display_cache.last_failure_type = sensor_data.failure_type;
        ESP_LOGV(TAG, "Display updated: S1=%.3f, S2=%.3f, cal1=%s, cal2=%s", s1, s2, cur_cal1, cur_cal2);
    } else {
        ESP_LOGV(TAG, "Display skipped (S1=%.3f, S2=%.3f, cal1=%s, cal2=%s)", s1, s2, cur_cal1, cur_cal2);
    }

}

// Menu overlay - displays all 4 menu items vertically centered with ">" for active item
static void show_menu_overlay(void)
{
    // Use the new dedicated menu display function
    display_manager_update_menu(s_app_ctx.menu_selected_item);
}

// Calibration overlay - Dual O2 sensor calibration interface per UI architecture
static void show_calibration_overlay(void)
{
    // Get current sensor readings
    sensor_data_t sensor_data;
    esp_err_t ret = sensor_manager_read(&sensor_data);
    
    // Get individual sensor readings
    float sensor1_mv = 0.0f, sensor1_ppo2 = 0.0f;
    float sensor2_mv = 0.0f, sensor2_ppo2 = 0.0f;
    
    if (ret == ESP_OK) {
        sensor1_mv = sensor_data.o2_sensor1_reading_mv;
        sensor1_ppo2 = sensor_data.o2_sensor1_ppo2;
        sensor2_mv = sensor_data.o2_sensor2_reading_mv;
        sensor2_ppo2 = sensor_data.o2_sensor2_ppo2;
    }
    
    // Map menu items based on current navigation mode
    uint8_t display_selected_item = s_app_ctx.calibration_menu_item - 1; // Convert 1-based to 0-based
    
    // Use the dual calibration display function with two-mode navigation:
    // Gas selection mode: Air/O2/Custom selection
    // Action mode: Calibrate/Back with selected gas shown
    display_manager_update_dual_calibration(sensor1_mv, sensor1_ppo2, sensor2_mv, sensor2_ppo2,
                                           display_selected_item, s_app_ctx.calibration_selected_gas,
                                           s_app_ctx.calibration_o2_percent, s_app_ctx.calibration_custom_editing,
                                           s_app_ctx.calibration_in_action_mode, s_app_ctx.calibration_session_active);
}

// Calibration reset overlay - Shows current sensor readings and reset options
static void show_calibration_reset_overlay(void)
{
    // Use the dedicated reset calibration display function
    display_manager_update_reset_calibration(s_app_ctx.current_sensor0_mv, 
                                            s_app_ctx.current_sensor1_mv, 
                                            s_app_ctx.cal_reset_menu_item);
}

// Setup overlay - Multi-parameter configuration with digit editing
static void show_setup_overlay(void)
{
    display_data_t display_data = {0};
    display_data.mode = DISPLAY_MODE_TEXT;
    
    // Use temporary config if we have one loaded (during/after editing), or current config
    static bool temp_config_loaded = false;
    const app_config_t *config;
    
    if (s_app_ctx.setup_editing) {
        // Currently editing - use temp config and mark as loaded
        config = &s_app_ctx.setup_temp_config;
        temp_config_loaded = true;
    } else if (temp_config_loaded && s_app_ctx.current_state == APP_STATE_SETUP) {
        // Not editing but temp config exists and still in setup - keep using temp config
        config = &s_app_ctx.setup_temp_config;
    } else {
        // Fresh setup entry or left setup mode - use current config and reset flag
        config = app_config_get_current();
        temp_config_loaded = false;
    }
    
    // Format lines based on current selection and editing state
    switch (s_app_ctx.setup_selected_item) {
        case SETUP_ITEM_WARN_LOW_PPO2:
            format_ppo2_item(&display_data, "Warn Low PPO2:", config->ppo2_low_warning);
            break;
        case SETUP_ITEM_ALARM_LOW_PPO2:
            format_ppo2_item(&display_data, "Alarm Low PPO2:", config->ppo2_low_alarm);
            break;
        case SETUP_ITEM_WARN_HIGH_PPO2:
            format_ppo2_item(&display_data, "Warn High PPO2:", config->ppo2_high_warning);
            break;
        case SETUP_ITEM_ALARM_HIGH_PPO2:
            format_ppo2_item(&display_data, "Alarm High PPO2:", config->ppo2_high_alarm);
            break;
        case SETUP_ITEM_SENSOR_DISAGREEMENT:
            format_ppo2_item(&display_data, "Sensor Disagree:", config->sensor_disagreement_threshold);
            break;
        case SETUP_ITEM_ATMOSPHERIC_PRESSURE:
            format_ppo2_item(&display_data, "Atm Pressure:", config->atmospheric_pressure);
            break;
        case SETUP_ITEM_SAVE:
            snprintf(display_data.line1, sizeof(display_data.line1), "save");
            snprintf(display_data.line2, sizeof(display_data.line2), ">Save & exit");
            break;
        case SETUP_ITEM_BACK:
            snprintf(display_data.line1, sizeof(display_data.line1), "back");
            snprintf(display_data.line2, sizeof(display_data.line2), ">Exit without save");
            break;
    }
    
    snprintf(display_data.line3, sizeof(display_data.line3), "MODE:%s", 
             s_app_ctx.setup_editing ? (s_app_ctx.setup_digit_index + 1 < s_app_ctx.setup_digit_count ? "Next" : "Done") : "Enter");
    
    // Convert setup display to system message format
    char setup_msg[256];
    snprintf(setup_msg, sizeof(setup_msg), "%s\n%s\n%s", 
             display_data.line1, display_data.line2, display_data.line3);
    display_manager_update_system_message(setup_msg);
}

// System message overlay - displays calibration results, health info, etc.
static void show_system_message_overlay(void)
{
    // Use the dedicated system message display function
    display_manager_update_system_message(s_app_ctx.system_message);
}

// Print calibration overlay - prints calibration data to debug and shows message
static void show_print_calibration_overlay(void)
{
    ESP_LOGI(TAG, "Printing calibration data to debug log");
    
    // Print calibration data for both sensors to debug log
    esp_err_t ret1 = sensor_manager_print_csv_log(0, 0); // Sensor 0, all entries
    esp_err_t ret2 = sensor_manager_print_csv_log(1, 0); // Sensor 1, all entries
    
    // Also print sensor summaries
    esp_err_t ret3 = sensor_manager_print_sensor_summary(0);
    esp_err_t ret4 = sensor_manager_print_sensor_summary(1);
    
    // Create system message per UI architecture
    if (ret1 == ESP_OK && ret2 == ESP_OK && ret3 == ESP_OK && ret4 == ESP_OK) {
        snprintf(s_app_ctx.system_message, sizeof(s_app_ctx.system_message),
                "CALIBRATION DATA PRINTED\n\n"
                "Sensor 1 & 2 calibration\n"
                "logs and summaries have\n"
                "been printed to debug\n"
                "console.\n\n"
                "Check serial monitor\n"
                "for CSV data.");
    } else {
        snprintf(s_app_ctx.system_message, sizeof(s_app_ctx.system_message),
                "PRINT CALIBRATION FAILED\n\n"
                "Error printing calibration\n"
                "data to debug console.\n\n"
                "Check sensor connections.");
    }
    
    // Switch to system message display
    enter_state(APP_STATE_SYSTEM_MESSAGE);
}

// Sensor health overlay - prints sensor health to debug and shows message  
static void show_sensor_health_overlay(void)
{
    ESP_LOGI(TAG, "Printing sensor health data to debug log");
    
    // Print health summaries for both sensors
    esp_err_t ret1 = sensor_manager_print_sensor_summary(0);
    esp_err_t ret2 = sensor_manager_print_sensor_summary(1);
    
    // Get health status for message content
    sensor_health_info_t health_s1, health_s2;
    esp_err_t health_ret1 = sensor_manager_get_health_status(0, &health_s1);
    esp_err_t health_ret2 = sensor_manager_get_health_status(1, &health_s2);
    
    // Create system message per UI architecture
    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        char* msg_ptr = s_app_ctx.system_message;
        int remaining = sizeof(s_app_ctx.system_message);
        int written = 0;
        
        written = snprintf(msg_ptr, remaining, "SENSOR HEALTH DATA\n\n");
        msg_ptr += written; remaining -= written;
        
        // Sensor 1 health
        if (health_ret1 == ESP_OK) {
            const char* health_str = (health_s1.health_status == SENSOR_HEALTH_GOOD) ? "GOOD" :
                                   (health_s1.health_status == SENSOR_HEALTH_CAUTION) ? "CAUTION" :
                                   (health_s1.health_status == SENSOR_HEALTH_FAIL) ? "FAIL" : "UNKNOWN";
            written = snprintf(msg_ptr, remaining, "Sensor 1: %s\n", health_str);
            msg_ptr += written; remaining -= written;
        }
        
        // Sensor 2 health
        if (health_ret2 == ESP_OK) {
            const char* health_str = (health_s2.health_status == SENSOR_HEALTH_GOOD) ? "GOOD" :
                                   (health_s2.health_status == SENSOR_HEALTH_CAUTION) ? "CAUTION" :
                                   (health_s2.health_status == SENSOR_HEALTH_FAIL) ? "FAIL" : "UNKNOWN";
            written = snprintf(msg_ptr, remaining, "Sensor 2: %s\n\n", health_str);
            msg_ptr += written; remaining -= written;
        }
        
        snprintf(msg_ptr, remaining, "Detailed health data\nprinted to debug console.");
    } else {
        snprintf(s_app_ctx.system_message, sizeof(s_app_ctx.system_message),
                "SENSOR HEALTH FAILED\n\n"
                "Error reading sensor\n" 
                "health information.\n\n"
                "Check sensor connections.");
    }
    
    // Switch to system message display
    enter_state(APP_STATE_SYSTEM_MESSAGE);
}

// State management functions
static void enter_state(app_state_t new_state)
{
    if (new_state == s_app_ctx.current_state) {
        return;
    }
    
    ESP_LOGI(TAG, "State change: %s -> %s", 
             state_to_string(s_app_ctx.current_state), 
             state_to_string(new_state));
    
    s_app_ctx.previous_state = s_app_ctx.current_state;
    s_app_ctx.current_state = new_state;
    s_app_ctx.state_entry_time = esp_timer_get_time() / 1000;
    // Ensure UI redraw on next loop after any state change
    s_display_cache.state_changed = true;
    
    // Reset menu selection when entering menu
    if (new_state == APP_STATE_MENU) {
        s_app_ctx.menu_selected_item = 0;
    }
    
    // Reset calibration mode state when entering calibration (but preserve session state)
    if (new_state == APP_STATE_CALIBRATION) {
        s_app_ctx.calibration_menu_item = 1; // Start with Air selection
        s_app_ctx.calibration_selected_gas = 1; // Air is selected by default
        s_app_ctx.calibration_custom_editing = false;
        s_app_ctx.calibration_in_action_mode = false; // Start in gas selection mode
        // Set default custom O2 value if not already valid
        if (s_app_ctx.calibration_o2_percent < O2_PERCENT_MIN || s_app_ctx.calibration_o2_percent > O2_PERCENT_MAX) {
            s_app_ctx.calibration_o2_percent = O2_PERCENT_DEFAULT_AIR; // Default custom value
        }
        // Note: calibration_session_active and calibration_first_gas_percent are preserved
        // so user can return to continue 2-point calibration
    }
    
    // Reset calibration reset mode state when entering
    if (new_state == APP_STATE_CAL_RESET) {
        s_app_ctx.cal_reset_menu_item = 0; // Start with "reset live"
        s_app_ctx.current_sensor0_mv = 0.0f;
        s_app_ctx.current_sensor1_mv = 0.0f;
    }
    
    // Reset setup mode state when entering setup
    if (new_state == APP_STATE_SETUP) {
        s_app_ctx.setup_selected_item = 0;  // Start with first item
        s_app_ctx.setup_editing = false;
        s_app_ctx.setup_digit_index = 0;
        s_app_ctx.setup_digit_count = 0;
        // Load current config into temp config for potential editing
        const app_config_t *current_config = app_config_get_current();
        s_app_ctx.setup_temp_config = *current_config;
    }
}


static const char* state_to_string(app_state_t state)
{
    switch (state) {
        case APP_STATE_MAIN: return "MAIN";
        case APP_STATE_MENU: return "MENU";
        case APP_STATE_CALIBRATION: return "CALIBRATION";
        case APP_STATE_CAL_RESET: return "CAL_RESET";
        case APP_STATE_SETUP: return "SETUP";
        case APP_STATE_LOG: return "LOG";
        case APP_STATE_PRINT_CALIBRATION: return "PRINT_CALIBRATION";
        case APP_STATE_SENSOR_HEALTH: return "SENSOR_HEALTH";
        case APP_STATE_SYSTEM_MESSAGE: return "SYSTEM_MESSAGE";
        default: return "UNKNOWN";
    }
}

// Calibration mode helper functions
// New calibration helper functions for updated design
static void calibration_increment_custom_o2(void)
{
    // Increment custom O2 percentage (18-99 range)
    s_app_ctx.calibration_o2_percent += 1.0f;
    if (s_app_ctx.calibration_o2_percent > O2_PERCENT_MAX) {
        s_app_ctx.calibration_o2_percent = O2_PERCENT_MIN; // Wrap to minimum
    }
    
    ESP_LOGI(TAG, "Custom O2 incremented to %.1f%%", s_app_ctx.calibration_o2_percent);
}

// Setup mode helper functions
static void setup_get_digit_info(uint8_t item, uint8_t *digit_count)
{
    switch (item) {
        case SETUP_ITEM_WARN_LOW_PPO2:
        case SETUP_ITEM_ALARM_LOW_PPO2:
        case SETUP_ITEM_WARN_HIGH_PPO2:
        case SETUP_ITEM_ALARM_HIGH_PPO2:
        case SETUP_ITEM_SENSOR_DISAGREEMENT:
        case SETUP_ITEM_ATMOSPHERIC_PRESSURE:
            *digit_count = 3; // X.XX format (3 digits: whole + 2 decimal)
            break;
        default:
            *digit_count = 0;
            break;
    }
}

static void setup_increment_digit(void)
{
    switch (s_app_ctx.setup_selected_item) {
        case SETUP_ITEM_WARN_LOW_PPO2:
            increment_ppo2_digit(&s_app_ctx.setup_temp_config.ppo2_low_warning);
            break;
        case SETUP_ITEM_ALARM_LOW_PPO2:
            increment_ppo2_digit(&s_app_ctx.setup_temp_config.ppo2_low_alarm);
            break;
        case SETUP_ITEM_WARN_HIGH_PPO2:
            increment_ppo2_digit(&s_app_ctx.setup_temp_config.ppo2_high_warning);
            break;
        case SETUP_ITEM_ALARM_HIGH_PPO2:
            increment_ppo2_digit(&s_app_ctx.setup_temp_config.ppo2_high_alarm);
            break;
        case SETUP_ITEM_SENSOR_DISAGREEMENT:
            increment_ppo2_digit(&s_app_ctx.setup_temp_config.sensor_disagreement_threshold);
            break;
        case SETUP_ITEM_ATMOSPHERIC_PRESSURE:
            increment_ppo2_digit(&s_app_ctx.setup_temp_config.atmospheric_pressure);
            break;
        default:
            break;
    }
    
    ESP_LOGI(TAG, "Setup digit incremented: item=%d, digit=%d", s_app_ctx.setup_selected_item, s_app_ctx.setup_digit_index);
}

static void setup_save_config(void)
{
    ESP_LOGI(TAG, "Saving setup configuration to NVS");
    
    esp_err_t ret = app_config_save(&s_app_ctx.setup_temp_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Setup configuration saved successfully");
        
        // Apply display contrast immediately
        display_manager_set_brightness(s_app_ctx.setup_temp_config.display_contrast);
        
        // Show success message briefly
        display_data_t success_msg = {0};
        success_msg.mode = DISPLAY_MODE_TEXT;
        snprintf(success_msg.line1, sizeof(success_msg.line1), "SETUP SAVED");
        snprintf(success_msg.line2, sizeof(success_msg.line2), "Contrast: next boot");
        snprintf(success_msg.line3, sizeof(success_msg.line3), "Other: immediate");
        display_manager_update_system_message("SETUP SAVED\nContrast: next boot\nOther: immediate");
        vTaskDelay(pdMS_TO_TICKS(2500)); // Longer display time for user to read
    } else {
        ESP_LOGE(TAG, "Failed to save setup configuration: %s", esp_err_to_name(ret));
        
        // Show error message briefly
        display_data_t error_msg = {0};
        error_msg.mode = DISPLAY_MODE_TEXT;
        snprintf(error_msg.line1, sizeof(error_msg.line1), "SETUP ERROR");
        snprintf(error_msg.line2, sizeof(error_msg.line2), "Save failed");
        snprintf(error_msg.line3, sizeof(error_msg.line3), "Try again");
        display_manager_update_system_message("SETUP ERROR\nSave failed\nTry again");
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

// System recovery and watchdog functions
static void attempt_sensor_recovery(void)
{
    ESP_LOGW(TAG, "Attempting sensor recovery (attempt %lu)", s_runtime_state.sensor_recovery_attempts + 1);
    
    s_runtime_state.sensor_recovery_attempts++;
    s_runtime_state.in_recovery_mode = true;
    
    // Progressive recovery strategy
    if (s_runtime_state.sensor_recovery_attempts == 1) {
        // First attempt: simple sensor re-initialization
        ESP_LOGI(TAG, "Recovery step 1: Re-initializing sensors");
        sensor_manager_deinit();
        vTaskDelay(pdMS_TO_TICKS(100));
        sensor_manager_init();
    } else if (s_runtime_state.sensor_recovery_attempts == 2) {
        // Second attempt: Re-initialize I2C subsystem
        ESP_LOGI(TAG, "Recovery step 2: Re-initializing I2C subsystem");
        // No external sensor managers to deinitialize
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // No external I2C sensor components to reinitialize
        
        sensor_manager_init();
    } else if (s_runtime_state.sensor_recovery_attempts >= SENSOR_RECOVERY_MAX_ATTEMPTS) {
        // Final attempt: System reset after multiple failures
        ESP_LOGE(TAG, "Multiple recovery attempts failed. System may be in critical state.");
        ESP_LOGE(TAG, "Consider manual intervention or system restart.");
        
        // Reset recovery counter to prevent infinite attempts
        if (s_runtime_state.sensor_recovery_attempts >= SENSOR_RECOVERY_RESET_THRESHOLD) {
            ESP_LOGE(TAG, "Maximum recovery attempts reached. Resetting counter.");
            s_runtime_state.sensor_recovery_attempts = 0;
        }
    }
}

static void check_system_watchdog(void)
{
    uint32_t current_time = esp_timer_get_time() / 1000;
    uint32_t time_since_last_read = current_time - s_runtime_state.last_successful_sensor_read;
    
    // Check if sensors have been failing for too long
    if (time_since_last_read > SENSOR_READ_TIMEOUT_MS && !s_runtime_state.in_recovery_mode) {  // Sensor timeout
        ESP_LOGW(TAG, "Sensors failing for %lums - initiating recovery", time_since_last_read);
        attempt_sensor_recovery();
    } else if (time_since_last_read > SENSOR_READ_CRITICAL_TIMEOUT_MS) {  // Critical timeout
        ESP_LOGE(TAG, "CRITICAL: Sensors have been failing for %lums", time_since_last_read);
        ESP_LOGE(TAG, "System may be unsafe for diving operations!");
    }
    
    // Memory watchdog
    uint32_t free_heap = esp_get_free_heap_size();
    if (free_heap < SYSTEM_HEAP_CRITICAL_THRESHOLD) {  // Critical heap threshold
        ESP_LOGE(TAG, "CRITICAL: Very low memory: %lu bytes", free_heap);
        // Force garbage collection attempt by freeing any temporary allocations
        // In a real system, you might want to reduce functionality or restart
    }
}

static void calibration_perform_with_o2(float o2_percent)
{
    ESP_LOGI(TAG, "Performing calibration with %.1f%% O2 (session active: %d)", 
             o2_percent, s_app_ctx.calibration_session_active);
    
    esp_err_t ret;
    
    if (!s_app_ctx.calibration_session_active) {
        // First gas calibration - start new session
        ESP_LOGI(TAG, "Starting new calibration session for both sensors");
        
        // Start calibration sessions for both sensors
        esp_err_t ret1 = sensor_calibration_start_session(0); // Sensor 0
        esp_err_t ret2 = sensor_calibration_start_session(1); // Sensor 1
        
        if (ret1 != ESP_OK || ret2 != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start calibration sessions: S0=%s, S1=%s", 
                     esp_err_to_name(ret1), esp_err_to_name(ret2));
            ret = (ret1 != ESP_OK) ? ret1 : ret2;
        } else {
            // Add first calibration point for both sensors
            ret = sensor_manager_perform_dual_o2_calibration(o2_percent);
            
            if (ret == ESP_OK) {
                // Mark session as active and store first gas info
                s_app_ctx.calibration_session_active = true;
                s_app_ctx.calibration_first_gas_percent = o2_percent;
                ESP_LOGI(TAG, "First gas calibration complete, session active");
            }
        }
    } else {
        // Second gas calibration - add to existing session
        ESP_LOGI(TAG, "Adding second gas to existing calibration session");
        
        // Add second calibration point and finalize
        ret = sensor_manager_perform_dual_o2_calibration(o2_percent);
        
        if (ret == ESP_OK) {
            // Session complete
            s_app_ctx.calibration_session_active = false;
            ESP_LOGI(TAG, "Two-point calibration complete: %.1f%% -> %.1f%%", 
                     s_app_ctx.calibration_first_gas_percent, o2_percent);
        }
    }
    
    // Get sensor health information for both sensors
    sensor_health_info_t health_s1, health_s2;
    esp_err_t health_ret_s1 = sensor_manager_get_health_status(0, &health_s1);
    esp_err_t health_ret_s2 = sensor_manager_get_health_status(1, &health_s2);
    
    // Create calibration results message per UI architecture
    char* msg_ptr = s_app_ctx.system_message;
    int remaining = sizeof(s_app_ctx.system_message);
    int written = 0;
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Dual calibration completed successfully");
        
        // Build success message with calibration and health results
        written = snprintf(msg_ptr, remaining, "CALIBRATION COMPLETE\n\n");
        msg_ptr += written; remaining -= written;
        
        // Sensor 1 results
        written = snprintf(msg_ptr, remaining, "Sensor 1: PASS\n");
        msg_ptr += written; remaining -= written;
        if (health_ret_s1 == ESP_OK) {
            const char* health_str = (health_s1.health_status == SENSOR_HEALTH_GOOD) ? "GOOD" :
                                   (health_s1.health_status == SENSOR_HEALTH_CAUTION) ? "CAUTION" :
                                   (health_s1.health_status == SENSOR_HEALTH_FAIL) ? "FAIL" : "UNKNOWN";
            written = snprintf(msg_ptr, remaining, "Health: %s\n", health_str);
            msg_ptr += written; remaining -= written;
        }
        
        // Sensor 2 results  
        written = snprintf(msg_ptr, remaining, "\nSensor 2: PASS\n");
        msg_ptr += written; remaining -= written;
        if (health_ret_s2 == ESP_OK) {
            const char* health_str = (health_s2.health_status == SENSOR_HEALTH_GOOD) ? "GOOD" :
                                   (health_s2.health_status == SENSOR_HEALTH_CAUTION) ? "CAUTION" :
                                   (health_s2.health_status == SENSOR_HEALTH_FAIL) ? "FAIL" : "UNKNOWN";
            written = snprintf(msg_ptr, remaining, "Health: %s\n", health_str);
            msg_ptr += written; remaining -= written;
        }
        
        // Add gas type info
        snprintf(msg_ptr, remaining, "\nGas: %.1f%% O2", o2_percent);
        
    } else {
        ESP_LOGE(TAG, "Calibration failed: %s", esp_err_to_name(ret));
        
        // Build failure message
        written = snprintf(msg_ptr, remaining, "CALIBRATION FAILED\n\n");
        msg_ptr += written; remaining -= written;
        
        written = snprintf(msg_ptr, remaining, "Sensor 1: FAILED\n");
        msg_ptr += written; remaining -= written;
        written = snprintf(msg_ptr, remaining, "Sensor 2: FAILED\n\n");
        msg_ptr += written; remaining -= written;
        
        snprintf(msg_ptr, remaining, "Check sensor connections\nand try again");
    }
    
    // Show system message display instead of staying on calibration screen
    enter_state(APP_STATE_SYSTEM_MESSAGE);
}

// Calibration reset helper functions
static void update_calibration_reset_readings(void)
{
    // Update current sensor readings for display
    sensor_data_t sensor_data;
    esp_err_t ret = sensor_manager_read(&sensor_data);
    if (ret == ESP_OK) {
        s_app_ctx.current_sensor0_mv = sensor_data.o2_sensor1_reading_mv;
        s_app_ctx.current_sensor1_mv = sensor_data.o2_sensor2_reading_mv;
    } else {
        ESP_LOGW(TAG, "Failed to read sensor data for calibration reset: %s", esp_err_to_name(ret));
        s_app_ctx.current_sensor0_mv = 0.0f;
        s_app_ctx.current_sensor1_mv = 0.0f;
    }
}

static void perform_calibration_reset_all(void)
{
    ESP_LOGI(TAG, "Performing calibration reset for all sensors and PPO2 logs");
    
    // Reset both sensors individually using the proper reset function
    esp_err_t ret1 = sensor_manager_reset_sensor(0);
    esp_err_t ret2 = sensor_manager_reset_sensor(1);
    
    // Also reset PPO2 logs
    esp_err_t ret3 = ppo2_logger_reset();
    if (ret3 == ESP_OK) {
        ESP_LOGI(TAG, "PPO2 logs reset completed successfully");
    } else {
        ESP_LOGW(TAG, "Failed to reset PPO2 logs: %s", esp_err_to_name(ret3));
    }
    
    // Create system message per UI architecture
    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        ESP_LOGI(TAG, "All sensors reset completed successfully - both sensors are now UNCALIBRATED");
        snprintf(s_app_ctx.system_message, sizeof(s_app_ctx.system_message),
                "FULL RESET COMPLETE\n\n"
                "All sensor calibration\n"
                "data and PPO2 logs\n"
                "have been cleared.\n\n"
                "Both sensors are now\n"
                "UNCALIBRATED and must be\n"
                "recalibrated before use.\n\n"
                "Warning: Do not use for\n"
                "diving until recalibrated!");
    } else {
        ESP_LOGE(TAG, "Failed to reset calibration for sensors: S0=%s, S1=%s", 
                esp_err_to_name(ret1), esp_err_to_name(ret2));
        snprintf(s_app_ctx.system_message, sizeof(s_app_ctx.system_message),
                "CALIBRATION RESET FAILED\n\n"
                "Error resetting calibration\n"
                "data for sensors.\n\n"
                "Check sensor connections\n"
                "and try again.");
    }
    
    // Show system message display
    enter_state(APP_STATE_SYSTEM_MESSAGE);
}

static void perform_calibration_reset_sensor_0(void)
{
    ESP_LOGI(TAG, "Performing calibration reset for sensor 0 (s1) only");
    
    // Reset individual sensor using new function
    esp_err_t ret = sensor_manager_reset_sensor(0);
    
    // Create system message per UI architecture
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sensor 0 reset completed successfully - sensor is now UNCALIBRATED");
        snprintf(s_app_ctx.system_message, sizeof(s_app_ctx.system_message),
                "SENSOR 1 RESET COMPLETE\n\n"
                "Sensor 1 calibration data\n"
                "has been cleared.\n\n"
                "Sensor 1 is now\n"
                "UNCALIBRATED and must be\n"
                "recalibrated before use.\n\n"
                "Sensor 2 calibration\n"
                "remains unchanged.");
    } else {
        ESP_LOGE(TAG, "Sensor 0 reset failed: %s", esp_err_to_name(ret));
        snprintf(s_app_ctx.system_message, sizeof(s_app_ctx.system_message),
                "SENSOR 1 RESET FAILED\n\n"
                "Error resetting calibration\n"
                "data for Sensor 1.\n\n"
                "Check sensor connections\n"
                "and try again.");
    }
    
    // Show system message display
    enter_state(APP_STATE_SYSTEM_MESSAGE);
}

static void perform_calibration_reset_sensor_1(void)
{
    ESP_LOGI(TAG, "Performing calibration reset for sensor 1 (s2) only");
    
    // Reset individual sensor using new function
    esp_err_t ret = sensor_manager_reset_sensor(1);
    
    // Create system message per UI architecture
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sensor 1 reset completed successfully - sensor is now UNCALIBRATED");
        snprintf(s_app_ctx.system_message, sizeof(s_app_ctx.system_message),
                "SENSOR 2 RESET COMPLETE\n\n"
                "Sensor 2 calibration data\n"
                "has been cleared.\n\n"
                "Sensor 2 is now\n"
                "UNCALIBRATED and must be\n"
                "recalibrated before use.\n\n"
                "Sensor 1 calibration\n"
                "remains unchanged.");
    } else {
        ESP_LOGE(TAG, "Sensor 1 reset failed: %s", esp_err_to_name(ret));
        snprintf(s_app_ctx.system_message, sizeof(s_app_ctx.system_message),
                "SENSOR 2 RESET FAILED\n\n"
                "Error resetting calibration\n"
                "data for Sensor 2.\n\n"
                "Check sensor connections\n"
                "and try again.");
    }
    
    // Show system message display
    enter_state(APP_STATE_SYSTEM_MESSAGE);
}

// Deep sleep function with peripheral power off
static void enter_deep_sleep_mode(void)
{
    ESP_LOGI(TAG, "Preparing for deep sleep mode");
    
    // Show sleep message on display
    display_data_t sleep_msg = {0};
    sleep_msg.mode = DISPLAY_MODE_TEXT;
    snprintf(sleep_msg.line1, sizeof(sleep_msg.line1), "POWERING OFF");
    snprintf(sleep_msg.line2, sizeof(sleep_msg.line2), "Long press MODE");
    snprintf(sleep_msg.line3, sizeof(sleep_msg.line3), "to wake up");
    display_manager_update_system_message("POWERING OFF\nLong press MODE\nto wake up");
    
    // Wait for user to see the message
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Deinitialize all peripherals to save power
    ESP_LOGI(TAG, "Deinitializing peripherals for power saving");
    
    // Deinitialize managers in reverse order
    warning_manager_deinit();
    display_manager_deinit();
    // No external sensor managers to deinitialize
    sensor_manager_deinit();
    button_manager_deinit();
    
    // Close I2C bus
    if (s_i2c_bus) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }
    
    // Configure wake-up source: MODE button (GPIO_6)
    // Use proper ESP32-C3 GPIO wake-up method from ESP-IDF examples
    
    // Configure GPIO as input (button press = low due to pull-up)
    const gpio_config_t config = {
        .pin_bit_mask = BIT64(GPIO_BUTTON_MODE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    
    // Wait for GPIO to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Enable GPIO wake-up on low level (button press)
    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(BIT64(GPIO_BUTTON_MODE), ESP_GPIO_WAKEUP_GPIO_LOW));
    ESP_LOGI(TAG, "GPIO wake-up enabled on pin %d (wake on LOW)", GPIO_BUTTON_MODE);
    
    // Skip power domain configuration to avoid crashes
    // The ESP32-C3 will use default power domain settings for deep sleep
    
    ESP_LOGI(TAG, "Entering deep sleep mode - wake with MODE button press");

    // Enter deep sleep
    esp_deep_sleep_start();

    // This line will never be reached
}

// Helper function to format PPO2 configuration items with digit-by-digit editing
static void format_ppo2_item(display_data_t *display_data, const char *title, float value)
{
    snprintf(display_data->line1, sizeof(display_data->line1), "%s", title);

    if (s_app_ctx.setup_editing) {
        // Convert float to string with 2 decimal places: "X.XX"
        int whole = (int)value;
        int decimal = (int)((value - whole) * 100 + 0.5f); // Round to nearest centisimal

        // Format as string: "1.23" -> "123"
        char value_str[8];
        snprintf(value_str, sizeof(value_str), "%d%02d", whole, decimal);

        // Build display string with brackets around current digit
        char formatted[32] = ">";
        int len = strlen(value_str);

        for (int i = 0; i < len; i++) {
            if (i == s_app_ctx.setup_digit_index) {
                snprintf(formatted + strlen(formatted), sizeof(formatted) - strlen(formatted), "[%c]", value_str[i]);
            } else {
                snprintf(formatted + strlen(formatted), sizeof(formatted) - strlen(formatted), "%c", value_str[i]);
            }
            // Add decimal point after first digit
            if (i == 0 && len > 1) strcat(formatted, ".");
        }
        strcat(formatted, " bar");
        snprintf(display_data->line2, sizeof(display_data->line2), "%s", formatted);
    } else {
        snprintf(display_data->line2, sizeof(display_data->line2), ">%.2f bar", value);
    }
}

// Helper function to increment a digit in a PPO2 value (X.XX format)
static void increment_ppo2_digit(float *value)
{
    // Convert float to integer representation (X.XX -> XXX)
    int int_value = (int)(*value * 100 + 0.5f); // Round to nearest centisimal

    // Digit positions: [0]=ones, [1]=tenths, [2]=hundredths
    int digit_pos[] = {100, 10, 1};

    // Extract current digit value
    int current_digit = (int_value / digit_pos[s_app_ctx.setup_digit_index]) % 10;

    // Increment digit (0-9 cycle)
    current_digit = (current_digit + 1) % 10;

    // Update the integer value by replacing the digit
    int_value = int_value - ((int_value / digit_pos[s_app_ctx.setup_digit_index]) % 10) * digit_pos[s_app_ctx.setup_digit_index];
    int_value += current_digit * digit_pos[s_app_ctx.setup_digit_index];

    // Convert back to float
    *value = (float)int_value / 100.0f;

    // Apply reasonable limits based on the parameter type
    if (*value < 0.0f) *value = 0.0f;
    if (*value > 9.99f) *value = 9.99f;

    ESP_LOGI(TAG, "PPO2 value updated: %.2f bar (digit %d = %d)",
             *value, s_app_ctx.setup_digit_index, current_digit);
}
