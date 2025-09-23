/**
 * @file display_manager.c
 * @brief Display Manager - LVGL Implementation for SH1107
 */

#include "display_manager.h"
#include "sensor_manager.h"
#include "warning_manager.h"
#include "app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c_master.h"
#include "esp_lcd_sh1107.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "DISPLAY_MGR";

// Display specifications
#define DISPLAY_WIDTH       128
#define DISPLAY_HEIGHT      128
#define I2C_HOST            0

// Default hardware configuration  
#define DEFAULT_I2C_ADDR    0x3D
#define DEFAULT_I2C_FREQ    400000

// Display state
static bool s_initialized = false;
static lv_disp_t *s_disp = NULL;
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static esp_lcd_panel_io_handle_t s_io_handle = NULL;
static esp_lcd_panel_handle_t s_panel_handle = NULL;

// LVGL objects - REFACTORED FOR CLEAR ARCHITECTURE
static lv_obj_t *s_screen = NULL;

// === MAIN DISPLAY ELEMENTS ===
static lv_obj_t *s_battery = NULL;                // Battery symbol (top right)
static lv_obj_t *s_battery_voltage = NULL;        // Battery voltage text (near battery symbol) - montserrat10
static lv_obj_t *s_sensor1_cal_status = NULL;     // S1 calibration status (1pt/2pt/na) - montserrat14
static lv_obj_t *s_sensor1_name = NULL;           // "S1:" label - montserrat14
static lv_obj_t *s_sensor1_ppo2 = NULL;           // S1 PPO2 value (X.XX) - montserrat48
static lv_obj_t *s_sensor2_cal_status = NULL;     // S2 calibration status (1pt/2pt/na) - montserrat14
static lv_obj_t *s_sensor2_name = NULL;           // "S2:" label - montserrat14  
static lv_obj_t *s_sensor2_ppo2 = NULL;           // S2 PPO2 value (X.XX) - montserrat48
static lv_obj_t *s_warning_label = NULL;          // Warning/error messages (bottom) - montserrat14

// === MENU DISPLAY ELEMENTS (7 items) ===
static lv_obj_t *s_menu_sensor_cal = NULL;        // "sensor calibration" - montserrat14
static lv_obj_t *s_menu_reset_cal = NULL;         // "reset calibration" - montserrat14
static lv_obj_t *s_menu_print_cal = NULL;         // "print calibration" - montserrat14
static lv_obj_t *s_menu_sensor_health = NULL;     // "sensor health" - montserrat14
static lv_obj_t *s_menu_system_setup = NULL;      // "system setup" - montserrat14
static lv_obj_t *s_menu_power_off = NULL;         // "power off" - montserrat14
static lv_obj_t *s_menu_back = NULL;              // "back" - montserrat14

// === SENSOR CALIBRATION DISPLAY ELEMENTS ===
static lv_obj_t *s_cal_s1_label = NULL;           // "s1:" - montserrat14
static lv_obj_t *s_cal_s1_reading = NULL;         // "XXX.XX mV" - montserrat14
static lv_obj_t *s_cal_s2_label = NULL;           // "s2:" - montserrat14
static lv_obj_t *s_cal_s2_reading = NULL;         // "XXX.XX mV" - montserrat14
static lv_obj_t *s_cal_gas_air = NULL;            // "Air" - montserrat14
static lv_obj_t *s_cal_gas_o2 = NULL;             // "O2" - montserrat14
static lv_obj_t *s_cal_gas_custom = NULL;         // "Custom" - montserrat14
static lv_obj_t *s_cal_gas_percent = NULL;        // "XX%" (for custom) - montserrat14
static lv_obj_t *s_cal_calibrate_btn = NULL;      // "calibrate" - montserrat14
static lv_obj_t *s_cal_back_btn = NULL;           // "back" - montserrat14

// === RESET CALIBRATION DISPLAY ELEMENTS ===
static lv_obj_t *s_reset_s1_label = NULL;         // "s1:" - montserrat14
static lv_obj_t *s_reset_s1_voltage = NULL;       // "XXX.X mV" - montserrat14
static lv_obj_t *s_reset_s2_label = NULL;         // "s2:" - montserrat14
static lv_obj_t *s_reset_s2_voltage = NULL;       // "XXX.X mV" - montserrat14
static lv_obj_t *s_reset_all_btn = NULL;          // "reset all" - montserrat14
static lv_obj_t *s_reset_s1_btn = NULL;           // "reset S1" - montserrat14
static lv_obj_t *s_reset_s2_btn = NULL;           // "reset S2" - montserrat14
static lv_obj_t *s_reset_back_btn = NULL;         // "back" - montserrat14

// === SYSTEM SETUP DISPLAY ELEMENTS ===
static lv_obj_t *s_setup_atm_pressure = NULL;     // "atmospheric pressure: xxxx mBar" - montserrat14
static lv_obj_t *s_setup_disagreement = NULL;     // "sensor disagreement: x.xx Bar" - montserrat14
static lv_obj_t *s_setup_back_btn = NULL;         // "back" - montserrat14

// === SYSTEM MESSAGE DISPLAY ELEMENTS ===
static lv_obj_t *s_message_text = NULL;           // Multi-line message - montserrat14
static lv_obj_t *s_message_back_btn = NULL;       // "back" - montserrat14



// Forward declarations
static void apply_boot_time_contrast(void);
static esp_err_t init_lcd_panel(const display_config_t *config);
static esp_err_t init_lvgl_display(void);
static void create_ui_elements(void);
// static void update_ui_OLD(const display_data_t *data);  // Commented during refactoring

esp_err_t display_manager_init(const display_config_t *config)
{
    ESP_LOGI(TAG, "=== Initializing Display Manager with LVGL ===");

    if (s_initialized) {
        ESP_LOGW(TAG, "Display manager already initialized");
        return ESP_OK;
    }

    if (!config || !config->i2c_bus) {
        ESP_LOGE(TAG, "Invalid config or I2C bus handle");
        return ESP_ERR_INVALID_ARG;
    }

    // Store I2C bus handle
    s_i2c_bus = config->i2c_bus;

    esp_err_t ret;

    // Step 1: Initialize LCD panel
    ESP_LOGI(TAG, "Step 1: Initializing LCD panel...");
    ret = init_lcd_panel(config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD panel initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Step 2: Initialize LVGL display
    ESP_LOGI(TAG, "Step 2: Initializing LVGL display...");
    ret = init_lvgl_display();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LVGL display initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Step 3: Create UI elements
    ESP_LOGI(TAG, "Step 3: Creating UI elements...");
    create_ui_elements();

    s_initialized = true;

    ESP_LOGI(TAG, "=== Display Manager Initialized Successfully ===");
    return ESP_OK;
}

// Apply contrast setting from NVS during boot-time initialization
static void apply_boot_time_contrast(void)
{ /*
    
    // Get the current configuration from NVS
    const app_config_t *config = app_config_get_current();
    if (!config) {
        ESP_LOGW(TAG, "Could not load config for contrast - using default");
        return;
    }
    
    // Map brightness percentage to contrast value
    uint8_t contrast = (config->display_contrast * 255) / 100;
    
    ESP_LOGI(TAG, "Applying boot-time contrast: %d%% -> contrast %d", config->display_contrast, contrast);
    
    // Send contrast command during initialization (when display is ready to receive commands)
    esp_err_t ret = esp_lcd_panel_io_tx_param(s_io_handle, 0x81, &contrast, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Boot-time contrast applied successfully");
    } else {
        ESP_LOGW(TAG, "Failed to apply boot-time contrast: %s", esp_err_to_name(ret));
        // Try alternative method
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay
        ret = esp_lcd_panel_io_tx_param(s_io_handle, 0x81, &contrast, 1);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Boot-time contrast applied successfully (retry)");
        } else {
            ESP_LOGW(TAG, "Boot-time contrast failed - using hardware default");
        }
    }  */
}

static esp_err_t init_lcd_panel(const display_config_t *config)
{
    // Install panel IO
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = config->i2c_address > 0 ? config->i2c_address : DEFAULT_I2C_ADDR,
        .scl_speed_hz = config->i2c_freq_hz > 0 ? config->i2c_freq_hz : DEFAULT_I2C_FREQ,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_bit_offset = 0,
        .flags = {
            .disable_control_phase = 1,
        }
    };
    
    esp_err_t ret = esp_lcd_new_panel_io_i2c(s_i2c_bus, &io_config, &s_io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Install SH1107 panel driver
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,
    };
    
    ret = esp_lcd_new_panel_sh1107(s_io_handle, &panel_config, &s_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create SH1107 panel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Reset and initialize panel
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel_handle, false)); // Changed to false for black background
    
    // Apply saved contrast setting from NVS during initialization
    apply_boot_time_contrast();
    
    return ESP_OK;
}

static esp_err_t init_lvgl_display(void)
{
    // Initialize LVGL
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    esp_err_t ret = lvgl_port_init(&lvgl_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LVGL port: %s", esp_err_to_name(ret));
        return ret;
    }
    
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = s_io_handle,
        .panel_handle = s_panel_handle,
        .buffer_size = DISPLAY_WIDTH * DISPLAY_HEIGHT,  // Buffer size for monochrome with RGB565 format
        .double_buffer = true,
        .hres = DISPLAY_WIDTH,
        .vres = DISPLAY_HEIGHT,
        .monochrome = true,
#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565,  // Use RGB565 for monochrome as per LVGL port requirements
#endif
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = false,
#endif
            .sw_rotate = false,
        }
    };
    
    s_disp = lvgl_port_add_disp(&disp_cfg);
    if (s_disp == NULL) {
        ESP_LOGE(TAG, "Failed to add LVGL display");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}


// NEW create_ui_elements function - clean architecture
static void create_ui_elements(void)
{
    if (lvgl_port_lock(0)) {
        s_screen = lv_disp_get_scr_act(s_disp);
        lv_obj_set_style_bg_color(s_screen, lv_color_black(), 0);
        
        // === CREATE MAIN DISPLAY ELEMENTS ===
        // Battery symbol (top right corner)
        s_battery = lv_label_create(s_screen);
        lv_label_set_text(s_battery, LV_SYMBOL_BATTERY_2);
        lv_obj_set_style_text_font(s_battery, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_battery, lv_color_white(), 0);
        lv_obj_align(s_battery, LV_ALIGN_TOP_RIGHT, -2, 2);
        lv_obj_add_flag(s_battery, LV_OBJ_FLAG_HIDDEN); // Initially hidden
        
        // Battery voltage text (below battery symbol)
        s_battery_voltage = lv_label_create(s_screen);
        lv_label_set_text(s_battery_voltage, "3.3V");
        lv_obj_set_style_text_font(s_battery_voltage, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_battery_voltage, lv_color_white(), 0);
        lv_obj_align(s_battery_voltage, LV_ALIGN_TOP_RIGHT, -25, 2);  // Below battery symbol
        lv_obj_add_flag(s_battery_voltage, LV_OBJ_FLAG_HIDDEN); // Initially hidden
        
        // Sensor 1 elements
        s_sensor1_cal_status = lv_label_create(s_screen);
        lv_label_set_text(s_sensor1_cal_status, "na");
        lv_obj_set_style_text_font(s_sensor1_cal_status, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_sensor1_cal_status, lv_color_white(), 0);
        lv_obj_align(s_sensor1_cal_status, LV_ALIGN_CENTER, -50, -35);
        lv_obj_add_flag(s_sensor1_cal_status, LV_OBJ_FLAG_HIDDEN);
        
        s_sensor1_name = lv_label_create(s_screen);
        lv_label_set_text(s_sensor1_name, "S1");
        lv_obj_set_style_text_font(s_sensor1_name, &lv_font_montserrat_22, 0);
        lv_obj_set_style_text_color(s_sensor1_name, lv_color_white(), 0);
        lv_obj_align(s_sensor1_name, LV_ALIGN_CENTER, -50, -15);
        lv_obj_add_flag(s_sensor1_name, LV_OBJ_FLAG_HIDDEN);
        
        s_sensor1_ppo2 = lv_label_create(s_screen);
        lv_label_set_text(s_sensor1_ppo2, "1.00");
        lv_obj_set_style_text_font(s_sensor1_ppo2, &lv_font_montserrat_48, 0); // 48pt for PPO2 values
        lv_obj_set_style_text_color(s_sensor1_ppo2, lv_color_white(), 0);
        lv_obj_align(s_sensor1_ppo2, LV_ALIGN_CENTER, 12, -20);
        lv_obj_add_flag(s_sensor1_ppo2, LV_OBJ_FLAG_HIDDEN);
        
        // Sensor 2 elements
        s_sensor2_cal_status = lv_label_create(s_screen);
        lv_label_set_text(s_sensor2_cal_status, "na");
        lv_obj_set_style_text_font(s_sensor2_cal_status, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_sensor2_cal_status, lv_color_white(), 0);
        lv_obj_align(s_sensor2_cal_status,  LV_ALIGN_CENTER, -50, 15);
        lv_obj_add_flag(s_sensor2_cal_status, LV_OBJ_FLAG_HIDDEN);
        
        s_sensor2_name = lv_label_create(s_screen);
        lv_label_set_text(s_sensor2_name, "S2");
        lv_obj_set_style_text_font(s_sensor2_name, &lv_font_montserrat_22, 0);
        lv_obj_set_style_text_color(s_sensor2_name, lv_color_white(), 0);
        lv_obj_align(s_sensor2_name, LV_ALIGN_CENTER, -50, 35);
        lv_obj_add_flag(s_sensor2_name, LV_OBJ_FLAG_HIDDEN);
        
        s_sensor2_ppo2 = lv_label_create(s_screen);
        lv_label_set_text(s_sensor2_ppo2, "1.00");
        lv_obj_set_style_text_font(s_sensor2_ppo2, &lv_font_montserrat_48, 0); // 48pt for PPO2 values
        lv_obj_set_style_text_color(s_sensor2_ppo2, lv_color_white(), 0);
        lv_obj_align(s_sensor2_ppo2, LV_ALIGN_CENTER, 12, 30);
        lv_obj_add_flag(s_sensor2_ppo2, LV_OBJ_FLAG_HIDDEN);
        
        // Warning label (bottom of screen) with scrolling for long messages
        s_warning_label = lv_label_create(s_screen);
        lv_label_set_text(s_warning_label, "");
        lv_obj_set_style_text_font(s_warning_label, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_warning_label, lv_color_white(), 0);
        lv_obj_align(s_warning_label, LV_ALIGN_BOTTOM_LEFT, 2, -2);
        
        // Set width constraint to enable scrolling (use most of display width)
        lv_obj_set_width(s_warning_label, DISPLAY_WIDTH - 10); // Leave 5px margins on each side
        
        // Configure scrolling behavior
        lv_label_set_long_mode(s_warning_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_style_anim_time(s_warning_label, 2000, 0); // Animation time in ms (slower = more readable)
        
        lv_obj_add_flag(s_warning_label, LV_OBJ_FLAG_HIDDEN);
        
        // === CREATE MENU DISPLAY ELEMENTS (7 items) ===
        s_menu_sensor_cal = lv_label_create(s_screen);
        lv_label_set_text(s_menu_sensor_cal, " sensor calibration");
        lv_obj_set_style_text_font(s_menu_sensor_cal, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_menu_sensor_cal, lv_color_white(), 0);
        lv_obj_align(s_menu_sensor_cal, LV_ALIGN_CENTER, 0, -45);
        lv_obj_add_flag(s_menu_sensor_cal, LV_OBJ_FLAG_HIDDEN);
        
        s_menu_reset_cal = lv_label_create(s_screen);
        lv_label_set_text(s_menu_reset_cal, " reset calibration");
        lv_obj_set_style_text_font(s_menu_reset_cal, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_menu_reset_cal, lv_color_white(), 0);
        lv_obj_align(s_menu_reset_cal, LV_ALIGN_CENTER, 0, -30);
        lv_obj_add_flag(s_menu_reset_cal, LV_OBJ_FLAG_HIDDEN);
        
        s_menu_print_cal = lv_label_create(s_screen);
        lv_label_set_text(s_menu_print_cal, " print logs");
        lv_obj_set_style_text_font(s_menu_print_cal, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_menu_print_cal, lv_color_white(), 0);
        lv_obj_align(s_menu_print_cal, LV_ALIGN_CENTER, 0, -15);
        lv_obj_add_flag(s_menu_print_cal, LV_OBJ_FLAG_HIDDEN);
        
        s_menu_sensor_health = lv_label_create(s_screen);
        lv_label_set_text(s_menu_sensor_health, " sensor health");
        lv_obj_set_style_text_font(s_menu_sensor_health, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_menu_sensor_health, lv_color_white(), 0);
        lv_obj_align(s_menu_sensor_health, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_flag(s_menu_sensor_health, LV_OBJ_FLAG_HIDDEN);
        
        s_menu_system_setup = lv_label_create(s_screen);
        lv_label_set_text(s_menu_system_setup, " setup");
        lv_obj_set_style_text_font(s_menu_system_setup, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_menu_system_setup, lv_color_white(), 0);
        lv_obj_align(s_menu_system_setup, LV_ALIGN_CENTER, 0, 15);
        lv_obj_add_flag(s_menu_system_setup, LV_OBJ_FLAG_HIDDEN);
        
        s_menu_power_off = lv_label_create(s_screen);
        lv_label_set_text(s_menu_power_off, " power off");
        lv_obj_set_style_text_font(s_menu_power_off, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_menu_power_off, lv_color_white(), 0);
        lv_obj_align(s_menu_power_off, LV_ALIGN_CENTER, 0, 30);
        lv_obj_add_flag(s_menu_power_off, LV_OBJ_FLAG_HIDDEN);
        
        s_menu_back = lv_label_create(s_screen);
        lv_label_set_text(s_menu_back, " back");
        lv_obj_set_style_text_font(s_menu_back, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_menu_back, lv_color_white(), 0);
        lv_obj_align(s_menu_back, LV_ALIGN_CENTER, 0, 45);
        lv_obj_add_flag(s_menu_back, LV_OBJ_FLAG_HIDDEN);
        
        // === CREATE SENSOR CALIBRATION DISPLAY ELEMENTS ===
        s_cal_s1_label = lv_label_create(s_screen);
        lv_label_set_text(s_cal_s1_label, "s1:");
        lv_obj_set_style_text_font(s_cal_s1_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_cal_s1_label, lv_color_white(), 0);
        lv_obj_align(s_cal_s1_label, LV_ALIGN_TOP_LEFT, 5, 5);
        lv_obj_add_flag(s_cal_s1_label, LV_OBJ_FLAG_HIDDEN);
        
        s_cal_s1_reading = lv_label_create(s_screen);
        lv_label_set_text(s_cal_s1_reading, "0.0mV");
        lv_obj_set_style_text_font(s_cal_s1_reading, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_cal_s1_reading, lv_color_white(), 0);
        lv_obj_align(s_cal_s1_reading, LV_ALIGN_TOP_LEFT, 30, 5);
        lv_obj_add_flag(s_cal_s1_reading, LV_OBJ_FLAG_HIDDEN);
        
        s_cal_s2_label = lv_label_create(s_screen);
        lv_label_set_text(s_cal_s2_label, "s2:");
        lv_obj_set_style_text_font(s_cal_s2_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_cal_s2_label, lv_color_white(), 0);
        lv_obj_align(s_cal_s2_label, LV_ALIGN_TOP_LEFT, 5, 25);
        lv_obj_add_flag(s_cal_s2_label, LV_OBJ_FLAG_HIDDEN);
        
        s_cal_s2_reading = lv_label_create(s_screen);
        lv_label_set_text(s_cal_s2_reading, "0.0mV");
        lv_obj_set_style_text_font(s_cal_s2_reading, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_cal_s2_reading, lv_color_white(), 0);
        lv_obj_align(s_cal_s2_reading, LV_ALIGN_TOP_LEFT, 30, 25);
        lv_obj_add_flag(s_cal_s2_reading, LV_OBJ_FLAG_HIDDEN);
        
        s_cal_gas_air = lv_label_create(s_screen);
        lv_label_set_text(s_cal_gas_air, "Air");
        lv_obj_set_style_text_font(s_cal_gas_air, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_cal_gas_air, lv_color_white(), 0);
        lv_obj_align(s_cal_gas_air, LV_ALIGN_TOP_LEFT, 5, 50);
        lv_obj_add_flag(s_cal_gas_air, LV_OBJ_FLAG_HIDDEN);
        
        s_cal_gas_o2 = lv_label_create(s_screen);
        lv_label_set_text(s_cal_gas_o2, "O2");
        lv_obj_set_style_text_font(s_cal_gas_o2, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_cal_gas_o2, lv_color_white(), 0);
        lv_obj_align(s_cal_gas_o2, LV_ALIGN_TOP_LEFT, 35, 50);  // Positioned next to Air
        lv_obj_add_flag(s_cal_gas_o2, LV_OBJ_FLAG_HIDDEN);
        
        s_cal_gas_custom = lv_label_create(s_screen);
        lv_label_set_text(s_cal_gas_custom, "Custom");
        lv_obj_set_style_text_font(s_cal_gas_custom, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_cal_gas_custom, lv_color_white(), 0);
        lv_obj_align(s_cal_gas_custom, LV_ALIGN_TOP_LEFT, 60, 50);  // Positioned next to O2
        lv_obj_add_flag(s_cal_gas_custom, LV_OBJ_FLAG_HIDDEN);
        
        s_cal_gas_percent = lv_label_create(s_screen);
        lv_label_set_text(s_cal_gas_percent, "21%");
        lv_obj_set_style_text_font(s_cal_gas_percent, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_cal_gas_percent, lv_color_white(), 0);
        lv_obj_align(s_cal_gas_percent, LV_ALIGN_TOP_LEFT, 5, 70);
        lv_obj_add_flag(s_cal_gas_percent, LV_OBJ_FLAG_HIDDEN);
        
        s_cal_calibrate_btn = lv_label_create(s_screen);
        lv_label_set_text(s_cal_calibrate_btn, " calibrate");
        lv_obj_set_style_text_font(s_cal_calibrate_btn, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_cal_calibrate_btn, lv_color_white(), 0);
        lv_obj_align(s_cal_calibrate_btn, LV_ALIGN_TOP_LEFT, 5, 90);
        lv_obj_add_flag(s_cal_calibrate_btn, LV_OBJ_FLAG_HIDDEN);
        
        s_cal_back_btn = lv_label_create(s_screen);
        lv_label_set_text(s_cal_back_btn, " back");
        lv_obj_set_style_text_font(s_cal_back_btn, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_cal_back_btn, lv_color_white(), 0);
        lv_obj_align(s_cal_back_btn, LV_ALIGN_TOP_LEFT, 5, 110);
        lv_obj_add_flag(s_cal_back_btn, LV_OBJ_FLAG_HIDDEN);
        
        // === CREATE RESET CALIBRATION DISPLAY ELEMENTS ===
        s_reset_s1_label = lv_label_create(s_screen);
        lv_label_set_text(s_reset_s1_label, "s1:");
        lv_obj_set_style_text_font(s_reset_s1_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_reset_s1_label, lv_color_white(), 0);
        lv_obj_align(s_reset_s1_label, LV_ALIGN_TOP_LEFT, 5, 5);
        lv_obj_add_flag(s_reset_s1_label, LV_OBJ_FLAG_HIDDEN);
        
        s_reset_s1_voltage = lv_label_create(s_screen);
        lv_label_set_text(s_reset_s1_voltage, "0.0mV");
        lv_obj_set_style_text_font(s_reset_s1_voltage, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_reset_s1_voltage, lv_color_white(), 0);
        lv_obj_align(s_reset_s1_voltage, LV_ALIGN_TOP_LEFT, 30, 5);
        lv_obj_add_flag(s_reset_s1_voltage, LV_OBJ_FLAG_HIDDEN);
        
        s_reset_s2_label = lv_label_create(s_screen);
        lv_label_set_text(s_reset_s2_label, "s2:");
        lv_obj_set_style_text_font(s_reset_s2_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_reset_s2_label, lv_color_white(), 0);
        lv_obj_align(s_reset_s2_label, LV_ALIGN_TOP_LEFT, 5, 25);
        lv_obj_add_flag(s_reset_s2_label, LV_OBJ_FLAG_HIDDEN);
        
        s_reset_s2_voltage = lv_label_create(s_screen);
        lv_label_set_text(s_reset_s2_voltage, "0.0mV");
        lv_obj_set_style_text_font(s_reset_s2_voltage, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_reset_s2_voltage, lv_color_white(), 0);
        lv_obj_align(s_reset_s2_voltage, LV_ALIGN_TOP_LEFT, 30, 25);
        lv_obj_add_flag(s_reset_s2_voltage, LV_OBJ_FLAG_HIDDEN);
        
        s_reset_all_btn = lv_label_create(s_screen);
        lv_label_set_text(s_reset_all_btn, " reset all");
        lv_obj_set_style_text_font(s_reset_all_btn, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_reset_all_btn, lv_color_white(), 0);
        lv_obj_align(s_reset_all_btn, LV_ALIGN_TOP_LEFT, 5, 50);
        lv_obj_add_flag(s_reset_all_btn, LV_OBJ_FLAG_HIDDEN);
        
        s_reset_s1_btn = lv_label_create(s_screen);
        lv_label_set_text(s_reset_s1_btn, " reset S1");
        lv_obj_set_style_text_font(s_reset_s1_btn, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_reset_s1_btn, lv_color_white(), 0);
        lv_obj_align(s_reset_s1_btn, LV_ALIGN_TOP_LEFT, 5, 70);
        lv_obj_add_flag(s_reset_s1_btn, LV_OBJ_FLAG_HIDDEN);
        
        s_reset_s2_btn = lv_label_create(s_screen);
        lv_label_set_text(s_reset_s2_btn, " reset S2");
        lv_obj_set_style_text_font(s_reset_s2_btn, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_reset_s2_btn, lv_color_white(), 0);
        lv_obj_align(s_reset_s2_btn, LV_ALIGN_TOP_LEFT, 5, 90);
        lv_obj_add_flag(s_reset_s2_btn, LV_OBJ_FLAG_HIDDEN);
        
        s_reset_back_btn = lv_label_create(s_screen);
        lv_label_set_text(s_reset_back_btn, " back");
        lv_obj_set_style_text_font(s_reset_back_btn, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_reset_back_btn, lv_color_white(), 0);
        lv_obj_align(s_reset_back_btn, LV_ALIGN_TOP_LEFT, 5, 110);
        lv_obj_add_flag(s_reset_back_btn, LV_OBJ_FLAG_HIDDEN);
        
        // === CREATE SYSTEM SETUP DISPLAY ELEMENTS ===
        s_setup_atm_pressure = lv_label_create(s_screen);
        lv_label_set_text(s_setup_atm_pressure, "atmospheric pressure: 1013 mBar");
        lv_obj_set_style_text_font(s_setup_atm_pressure, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_setup_atm_pressure, lv_color_white(), 0);
        lv_obj_align(s_setup_atm_pressure, LV_ALIGN_TOP_LEFT, 5, 20);
        lv_obj_add_flag(s_setup_atm_pressure, LV_OBJ_FLAG_HIDDEN);
        
        s_setup_disagreement = lv_label_create(s_screen);
        lv_label_set_text(s_setup_disagreement, "sensor disagreement: 0.05 Bar");
        lv_obj_set_style_text_font(s_setup_disagreement, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_setup_disagreement, lv_color_white(), 0);
        lv_obj_align(s_setup_disagreement, LV_ALIGN_TOP_LEFT, 5, 40);
        lv_obj_add_flag(s_setup_disagreement, LV_OBJ_FLAG_HIDDEN);
        
        s_setup_back_btn = lv_label_create(s_screen);
        lv_label_set_text(s_setup_back_btn, " back");
        lv_obj_set_style_text_font(s_setup_back_btn, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_setup_back_btn, lv_color_white(), 0);
        lv_obj_align(s_setup_back_btn, LV_ALIGN_TOP_LEFT, 5, 70);
        lv_obj_add_flag(s_setup_back_btn, LV_OBJ_FLAG_HIDDEN);
        
        // === CREATE SYSTEM MESSAGE DISPLAY ELEMENTS ===
        s_message_text = lv_label_create(s_screen);
        lv_label_set_text(s_message_text, "System message");
        lv_obj_set_style_text_font(s_message_text, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_message_text, lv_color_white(), 0);
        lv_obj_align(s_message_text, LV_ALIGN_CENTER, 0, -10);
        lv_obj_add_flag(s_message_text, LV_OBJ_FLAG_HIDDEN);
        
        s_message_back_btn = lv_label_create(s_screen);
        lv_label_set_text(s_message_back_btn, " back");
        lv_obj_set_style_text_font(s_message_back_btn, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_message_back_btn, lv_color_white(), 0);
        lv_obj_align(s_message_back_btn, LV_ALIGN_CENTER, 0, 40);
        lv_obj_add_flag(s_message_back_btn, LV_OBJ_FLAG_HIDDEN);
        
        ESP_LOGI(TAG, "Created all UI elements for clean architecture");
        lvgl_port_unlock();
    }
}



// HELPER FUNCTIONS for clean display mode switching
static void hide_all_elements(void)
{
    // Hide main display elements
    lv_obj_add_flag(s_battery, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_battery_voltage, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_sensor1_cal_status, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_sensor1_name, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_sensor1_ppo2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_sensor2_cal_status, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_sensor2_name, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_sensor2_ppo2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_warning_label, LV_OBJ_FLAG_HIDDEN);
    
    // Hide menu elements
    lv_obj_add_flag(s_menu_sensor_cal, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_menu_reset_cal, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_menu_print_cal, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_menu_sensor_health, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_menu_system_setup, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_menu_power_off, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_menu_back, LV_OBJ_FLAG_HIDDEN);
    
    // Hide sensor calibration elements
    lv_obj_add_flag(s_cal_s1_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_cal_s1_reading, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_cal_s2_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_cal_s2_reading, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_cal_gas_air, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_cal_gas_o2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_cal_gas_custom, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_cal_gas_percent, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_cal_calibrate_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_cal_back_btn, LV_OBJ_FLAG_HIDDEN);
    
    // Hide reset calibration elements
    lv_obj_add_flag(s_reset_s1_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_reset_s1_voltage, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_reset_s2_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_reset_s2_voltage, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_reset_all_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_reset_s1_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_reset_s2_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_reset_back_btn, LV_OBJ_FLAG_HIDDEN);
    
    // Hide system setup elements
    lv_obj_add_flag(s_setup_atm_pressure, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_setup_disagreement, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_setup_back_btn, LV_OBJ_FLAG_HIDDEN);
    
    // Hide system message elements
    lv_obj_add_flag(s_message_text, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_message_back_btn, LV_OBJ_FLAG_HIDDEN);
}

// TEMPORARY STUB FUNCTIONS - to maintain linking compatibility during refactoring
esp_err_t display_manager_update_main(const sensor_data_t *sensor_data)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Display manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!sensor_data) {
        ESP_LOGE(TAG, "Invalid sensor data pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Implement complete main display using clean architecture per UI specification
    if (lvgl_port_lock(0)) {
        // First hide all elements
        hide_all_elements();
        
        // Show main display elements per UI architecture
        lv_obj_clear_flag(s_battery, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_battery_voltage, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_sensor1_cal_status, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_sensor1_name, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_sensor1_ppo2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_sensor2_cal_status, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_sensor2_name, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_sensor2_ppo2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_warning_label, LV_OBJ_FLAG_HIDDEN);

        int32_t hundredths = (sensor_data->o2_sensor1_ppo2_mbar * 100 + 500) / 1000;

        int32_t whole = hundredths / 100;   // X
        int32_t frac  = hundredths % 100;   // XX
        
        // Update PPO2 values using integer mbar values converted to bar (X.XX format)
        char ppo2_str[16];
        snprintf(ppo2_str, sizeof(ppo2_str), "%ld.%02ld", whole, frac);
        lv_label_set_text(s_sensor1_ppo2, ppo2_str);

        hundredths = (sensor_data->o2_sensor2_ppo2_mbar * 100 + 500) / 1000;

        whole = hundredths / 100;   // X
        frac  = hundredths % 100;   // XX

        snprintf(ppo2_str, sizeof(ppo2_str), "%ld.%02ld", whole, frac);
        lv_label_set_text(s_sensor2_ppo2, ppo2_str);
        
        // Update calibration status labels (1pt/2pt/na)
        const char* cal_status_s1 = sensor_manager_get_calibration_display_status(0);
        const char* cal_status_s2 = sensor_manager_get_calibration_display_status(1);
        lv_label_set_text(s_sensor1_cal_status, cal_status_s1);
        lv_label_set_text(s_sensor2_cal_status, cal_status_s2);
        
        // Update battery display based on voltage and percentage
        const char* battery_symbol;
        if (sensor_data->battery_percentage >= 75) {
            battery_symbol = LV_SYMBOL_BATTERY_FULL;
        } else if (sensor_data->battery_percentage >= 50) {
            battery_symbol = LV_SYMBOL_BATTERY_3;
        } else if (sensor_data->battery_percentage >= 25) {
            battery_symbol = LV_SYMBOL_BATTERY_2;
        } else {
            battery_symbol = LV_SYMBOL_BATTERY_1;
        }
        lv_label_set_text(s_battery, battery_symbol);
        
        // Update battery voltage text using integer formatting (no FPU)
        char battery_str[16];
        format_battery_voltage_display(sensor_data->battery_voltage_mv, battery_str, sizeof(battery_str));
        lv_label_set_text(s_battery_voltage, battery_str);
        
        // Change color to red if battery is low
        /*
        if (sensor_data->battery_low) {
            lv_obj_set_style_text_color(s_battery, lv_color_make(255, 0, 0), 0);  // Red
            lv_obj_set_style_text_color(s_battery_voltage, lv_color_make(255, 0, 0), 0);  // Red
        } else {
            lv_obj_set_style_text_color(s_battery, lv_color_white(), 0);  // White
            lv_obj_set_style_text_color(s_battery_voltage, lv_color_white(), 0);  // White
        }*/
        
        // Update warning/error messages at bottom of screen
        // First priority: Check for warning manager messages (PPO2 warnings, sensor disagreement, etc.)
        const char* warning_msg = warning_manager_get_display_message();
        if (warning_msg) {
            lv_label_set_text(s_warning_label, warning_msg);
        } else if (!sensor_data->valid) {
            // Second priority: Show sensor failure messages if no warning manager message
            switch (sensor_data->failure_type) {
                case SENSOR_FAIL_CALIBRATION_INVALID:
                    lv_label_set_text(s_warning_label, "CALIBRATION INVALID");
                    break;
                case SENSOR_FAIL_O2_COMMUNICATION:
                    lv_label_set_text(s_warning_label, "SENSOR COMM ERROR");
                    break;
                default:
                    lv_label_set_text(s_warning_label, "SENSOR ERROR");
                    break;
            }
        } else {
            // Show raw sensor mV readings when no warnings/errors
            char sensor_mv_str[32];
            snprintf(sensor_mv_str, sizeof(sensor_mv_str), "S1: %ldmV  S2: %ldmV",
                     sensor_data->o2_sensor1_reading_mv, sensor_data->o2_sensor2_reading_mv);
            lv_label_set_text(s_warning_label, sensor_mv_str);
        }
        
        lvgl_port_unlock();
    }
    
    ESP_LOGD(TAG, "Main display updated: S1_PPO2=%.2f (%s), S2_PPO2=%.2f (%s), valid=%s", 
             sensor_data->o2_sensor1_ppo2, sensor_manager_get_calibration_display_status(0),
             sensor_data->o2_sensor2_ppo2, sensor_manager_get_calibration_display_status(1),
             sensor_data->valid ? "yes" : "no");
    return ESP_OK;
}

esp_err_t display_manager_update_menu(uint8_t selected_item)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Display manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // TODO: Implement new menu display using clean architecture
    if (lvgl_port_lock(0)) {
        // First hide all elements
        hide_all_elements();
        
        // Show menu elements with selection indicator
        lv_obj_t* menu_items[] = {
            s_menu_sensor_cal,
            s_menu_reset_cal,
            s_menu_print_cal,
            s_menu_sensor_health,
            s_menu_system_setup,
            s_menu_power_off,
            s_menu_back
        };

        for (int i = 0; i < 7; i++) {
            lv_obj_clear_flag(menu_items[i], LV_OBJ_FLAG_HIDDEN);
            // Set selection indicator for monochrome display
            if (i == selected_item) {
                // Invert colors for selection - black text on white background
                lv_obj_set_style_text_color(menu_items[i], lv_color_black(), 0);
                lv_obj_set_style_bg_color(menu_items[i], lv_color_white(), 0);
                lv_obj_set_style_bg_opa(menu_items[i], LV_OPA_COVER, 0);
            } else {
                // Normal - white text on black background
                lv_obj_set_style_text_color(menu_items[i], lv_color_white(), 0);
                lv_obj_set_style_bg_opa(menu_items[i], LV_OPA_TRANSP, 0);
            }
        }
        
        lvgl_port_unlock();
    }
    
    ESP_LOGD(TAG, "Menu display updated (STUB): selected_item=%d", selected_item);
    return ESP_OK;
}

esp_err_t display_manager_update_dual_calibration(int sensor1_mv, float sensor1_ppo2, 
                                                 int sensor2_mv, float sensor2_ppo2,
                                                 uint8_t selected_item, uint8_t selected_gas,
                                                 float custom_o2_percent, bool custom_editing, bool in_action_mode,
                                                 bool calibration_session_active)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Display manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Implement complete calibration display per UI architecture
    if (lvgl_port_lock(0)) {
        // First hide all elements
        hide_all_elements();
        
        // Show calibration elements per UI architecture:
        // s1: label and readings, s2: label and readings
        lv_obj_clear_flag(s_cal_s1_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cal_s1_reading, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cal_s2_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cal_s2_reading, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cal_gas_air, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cal_gas_o2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cal_gas_custom, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cal_calibrate_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cal_back_btn, LV_OBJ_FLAG_HIDDEN);
        
        // Update sensor readings
        char reading_str[16];
        snprintf(reading_str, sizeof(reading_str), "%3dmV", sensor1_mv);
        lv_label_set_text(s_cal_s1_reading, reading_str);
        
        snprintf(reading_str, sizeof(reading_str), "%3dmV", sensor2_mv);
        lv_label_set_text(s_cal_s2_reading, reading_str);
        
        // Update display based on navigation mode per UI architecture
        if (!in_action_mode) {
            // Gas selection mode: Show gas options with individual selection highlighting
            
            // Apply individual highlighting to each gas option
            lv_obj_t* gas_labels[] = {s_cal_gas_air, s_cal_gas_o2, s_cal_gas_custom};
            
            for (int i = 0; i < 3; i++) {
                if (i == selected_item) {
                    // Invert colors for selection - black text on white background
                    lv_obj_set_style_text_color(gas_labels[i], lv_color_black(), 0);
                    lv_obj_set_style_bg_color(gas_labels[i], lv_color_white(), 0);
                    lv_obj_set_style_bg_opa(gas_labels[i], LV_OPA_COVER, 0);
                } else {
                    // Normal - white text on black background
                    lv_obj_set_style_text_color(gas_labels[i], lv_color_white(), 0);
                    lv_obj_set_style_bg_opa(gas_labels[i], LV_OPA_TRANSP, 0);
                }
            }
            
            // Show percentage for currently selected gas
            lv_obj_clear_flag(s_cal_gas_percent, LV_OBJ_FLAG_HIDDEN);
            char percent_str[16];
            
            if (selected_item == 0) {
                // Air - show 21%
                snprintf(percent_str, sizeof(percent_str), "21%%");
            } else if (selected_item == 1) {
                // O2 - show 100%
                snprintf(percent_str, sizeof(percent_str), "100%%");
            } else if (selected_item == 2) {
                // Custom - show custom percentage, with brackets if editing
                if (custom_editing) {
                    snprintf(percent_str, sizeof(percent_str), "[%.0f%%]", custom_o2_percent);
                } else {
                    snprintf(percent_str, sizeof(percent_str), "%.0f%%", custom_o2_percent);
                }
            }
            lv_label_set_text(s_cal_gas_percent, percent_str);
            
            // Hide action buttons in gas selection mode
            lv_obj_add_flag(s_cal_calibrate_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_cal_back_btn, LV_OBJ_FLAG_HIDDEN);
            
        } else {
            // Action mode: Show selected gas and action options
            const char* gas_names[] = {"", "Air", "O2", "Custom"};
            char gas_display[64];
            if (selected_gas == 3) {
                // Custom gas - show percentage
                snprintf(gas_display, sizeof(gas_display), "Selected: %s %.0f%%", 
                         gas_names[selected_gas], custom_o2_percent);
            } else {
                snprintf(gas_display, sizeof(gas_display), "Selected: %s", gas_names[selected_gas]);
            }
            // Hide individual gas labels in action mode and show selected gas in a different location
            lv_obj_add_flag(s_cal_gas_air, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_cal_gas_o2, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_cal_gas_custom, LV_OBJ_FLAG_HIDDEN);
            
            // Show selected gas percentage using the correct label
            char percent_str[16];
            if (selected_gas == 3) {
                // Custom gas - show custom percentage
                snprintf(percent_str, sizeof(percent_str), "%.0f%%", custom_o2_percent);
            } else if (selected_gas == 2) {
                // O2 - show 100%
                snprintf(percent_str, sizeof(percent_str), "100%%");
            } else {
                // Air - show 21%
                snprintf(percent_str, sizeof(percent_str), "21%%");
            }
            lv_label_set_text(s_cal_gas_percent, percent_str);
            lv_obj_clear_flag(s_cal_gas_percent, LV_OBJ_FLAG_HIDDEN);
            
            // Show action buttons with selection highlighting
            lv_obj_clear_flag(s_cal_calibrate_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(s_cal_back_btn, LV_OBJ_FLAG_HIDDEN);
            
            // Update button text based on calibration session state
            if (calibration_session_active) {
                lv_label_set_text(s_cal_calibrate_btn, "calibrate 2nd gas");
            } else {
                lv_label_set_text(s_cal_calibrate_btn, "calibrate");
            }
            lv_label_set_text(s_cal_back_btn, "back");
            
            // Apply color inversion highlighting based on selected item
            if (selected_item == 0) {
                // Highlight calibrate button
                lv_obj_set_style_text_color(s_cal_calibrate_btn, lv_color_black(), 0);
                lv_obj_set_style_bg_color(s_cal_calibrate_btn, lv_color_white(), 0);
                lv_obj_set_style_bg_opa(s_cal_calibrate_btn, LV_OPA_COVER, 0);
                // Normal back button
                lv_obj_set_style_text_color(s_cal_back_btn, lv_color_white(), 0);
                lv_obj_set_style_bg_opa(s_cal_back_btn, LV_OPA_TRANSP, 0);
            } else {
                // Normal calibrate button
                lv_obj_set_style_text_color(s_cal_calibrate_btn, lv_color_white(), 0);
                lv_obj_set_style_bg_opa(s_cal_calibrate_btn, LV_OPA_TRANSP, 0);
                // Highlight back button
                lv_obj_set_style_text_color(s_cal_back_btn, lv_color_black(), 0);
                lv_obj_set_style_bg_color(s_cal_back_btn, lv_color_white(), 0);
                lv_obj_set_style_bg_opa(s_cal_back_btn, LV_OPA_COVER, 0);
            }
        }
        
        lvgl_port_unlock();
    }
    
    ESP_LOGD(TAG, "Dual calibration display updated: S1=%dmV, S2=%dmV, mode=%s, selected=%d, gas=%d", 
             sensor1_mv, sensor2_mv, in_action_mode ? "action" : "gas", selected_item, selected_gas);
    return ESP_OK;
}

esp_err_t display_manager_update_system_message(const char* message)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Display manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!message) {
        ESP_LOGE(TAG, "Invalid message pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Display system message per UI architecture
    if (lvgl_port_lock(0)) {
        // First hide all elements
        hide_all_elements();
        
        // Show system message elements
        lv_obj_clear_flag(s_message_text, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_message_back_btn, LV_OBJ_FLAG_HIDDEN);
        
        // Update message text (supports multi-line)
        lv_label_set_text(s_message_text, message);
        
        // Always show ">back" as the only menu option
        lv_label_set_text(s_message_back_btn, ">back");
        
        lvgl_port_unlock();
    }
    
    ESP_LOGD(TAG, "System message display updated: %.50s%s", 
             message, strlen(message) > 50 ? "..." : "");
    return ESP_OK;
}

esp_err_t display_manager_update_reset_calibration(float sensor1_mv, float sensor2_mv, uint8_t selected_item)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Display manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Display reset calibration screen per UI architecture
    if (lvgl_port_lock(0)) {
        // First hide all elements
        hide_all_elements();
        
        // Show reset calibration elements
        lv_obj_clear_flag(s_reset_s1_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_reset_s1_voltage, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_reset_s2_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_reset_s2_voltage, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_reset_all_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_reset_s1_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_reset_s2_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_reset_back_btn, LV_OBJ_FLAG_HIDDEN);
        
        // Update sensor voltage readings
        char voltage_str[16];
        snprintf(voltage_str, sizeof(voltage_str), "%.1fmV", sensor1_mv);
        lv_label_set_text(s_reset_s1_voltage, voltage_str);
        
        snprintf(voltage_str, sizeof(voltage_str), "%.1fmV", sensor2_mv);
        lv_label_set_text(s_reset_s2_voltage, voltage_str);
        
        // Update button text without ">" prefix
        lv_label_set_text(s_reset_all_btn, "reset all");
        lv_label_set_text(s_reset_s1_btn, "reset S1");
        lv_label_set_text(s_reset_s2_btn, "reset S2");
        lv_label_set_text(s_reset_back_btn, "back");
        
        // Apply color inversion highlighting based on selected item
        lv_obj_t* reset_menu_items[] = {s_reset_all_btn, s_reset_s1_btn, s_reset_s2_btn, s_reset_back_btn};
        
        for (int i = 0; i < 4; i++) {
            if (i == selected_item) {
                // Invert colors for selection - black text on white background
                lv_obj_set_style_text_color(reset_menu_items[i], lv_color_black(), 0);
                lv_obj_set_style_bg_color(reset_menu_items[i], lv_color_white(), 0);
                lv_obj_set_style_bg_opa(reset_menu_items[i], LV_OPA_COVER, 0);
            } else {
                // Normal - white text on black background
                lv_obj_set_style_text_color(reset_menu_items[i], lv_color_white(), 0);
                lv_obj_set_style_bg_opa(reset_menu_items[i], LV_OPA_TRANSP, 0);
            }
        }
        
        lvgl_port_unlock();
    }
    
    ESP_LOGD(TAG, "Reset calibration display updated: S1=%.1fmV, S2=%.1fmV, selected=%d", 
             sensor1_mv, sensor2_mv, selected_item);
    return ESP_OK;
}

esp_err_t display_manager_set_brightness(uint8_t brightness)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Display contrast setting: %d%% (will take effect on next boot)", brightness);
    ESP_LOGI(TAG, "The SH1107 display driver applies contrast settings during initialization only");
    ESP_LOGI(TAG, "Contrast setting has been saved to NVS and will be applied when the system restarts");
    
    // The contrast setting is already saved to NVS by the calling function (setup_save_config)
    // We just need to inform the user that it will take effect on next boot
    return ESP_OK;
}




void display_manager_get_dimensions(uint8_t *width, uint8_t *height)
{
    if (width) *width = DISPLAY_WIDTH;
    if (height) *height = DISPLAY_HEIGHT;
}

esp_err_t display_manager_sleep(bool sleep)
{
    if (!s_initialized || !s_panel_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return esp_lcd_panel_disp_on_off(s_panel_handle, !sleep);
}

void display_manager_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing display manager");

    if (s_initialized) {
        if (s_panel_handle) {
            esp_lcd_panel_disp_on_off(s_panel_handle, false);
            esp_lcd_panel_del(s_panel_handle);
            s_panel_handle = NULL;
        }
        
        if (s_io_handle) {
            esp_lcd_panel_io_del(s_io_handle);
            s_io_handle = NULL;
        }
        
        // Note: I2C bus is shared and managed by main.c, so we don't delete it
        s_i2c_bus = NULL;
        
        s_initialized = false;
    }

    ESP_LOGI(TAG, "Display manager deinitialized");
}

