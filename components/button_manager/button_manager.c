/**
 * @file button_manager.c
 * @brief Simple button manager implementation
 */

#include "sdkconfig.h"
#include "button_manager.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BTN_MGR";

// GPIO pin configuration (set during initialization)
static gpio_num_t s_mode_gpio = GPIO_NUM_NC;
static gpio_num_t s_select_gpio = GPIO_NUM_NC;

// Button timing constants are now defined in app_types.h

// Button state structure
typedef struct {
    gpio_num_t gpio;
    bool current_state;
    bool previous_state;
    uint32_t press_time;
    uint32_t release_time;
    button_event_t pending_event;
    bool long_press_fired;
} button_state_t;

// Button states
static button_state_t s_buttons[BUTTON_MAX];
static bool s_initialized = false;

// Forward declarations
static void init_button_gpio(button_id_t button_id, gpio_num_t gpio);
static bool read_button_raw(button_id_t button_id);
static void update_button_state(button_id_t button_id);

esp_err_t button_manager_init(const button_config_t *config)
{
    ESP_LOGI(TAG, "Initializing button manager");

    if (s_initialized) {
        ESP_LOGW(TAG, "Button manager already initialized");
        return ESP_OK;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Configuration is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Store GPIO configuration
    s_mode_gpio = config->mode_gpio;
    s_select_gpio = config->select_gpio;

    // Initialize button configurations
    init_button_gpio(BUTTON_MODE, s_mode_gpio);
    init_button_gpio(BUTTON_SELECT, s_select_gpio);

    // Initialize button states
    for (int i = 0; i < BUTTON_MAX; i++) {
        s_buttons[i].current_state = false;
        s_buttons[i].previous_state = false;
        s_buttons[i].press_time = 0;
        s_buttons[i].release_time = 0;
        s_buttons[i].pending_event = BUTTON_EVENT_NONE;
        s_buttons[i].long_press_fired = false;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Button manager initialized");
    return ESP_OK;
}

button_event_t button_manager_get_event(button_id_t button_id)
{
    if (!s_initialized || button_id >= BUTTON_MAX) {
        return BUTTON_EVENT_NONE;
    }

    button_event_t event = s_buttons[button_id].pending_event;
    s_buttons[button_id].pending_event = BUTTON_EVENT_NONE;
    
    return event;
}

esp_err_t button_manager_update(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Update all button states
    for (int i = 0; i < BUTTON_MAX; i++) {
        update_button_state((button_id_t)i);
    }

    return ESP_OK;
}

bool button_manager_is_pressed(button_id_t button_id)
{
    if (!s_initialized || button_id >= BUTTON_MAX) {
        return false;
    }

    return s_buttons[button_id].current_state;
}

void button_manager_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing button manager");
    
    if (s_initialized) {
        // Reset GPIO pins
        gpio_reset_pin(s_mode_gpio);
        gpio_reset_pin(s_select_gpio);
        s_initialized = false;
    }

    ESP_LOGI(TAG, "Button manager deinitialized");
}

// Private functions

static void init_button_gpio(button_id_t button_id, gpio_num_t gpio)
{
    s_buttons[button_id].gpio = gpio;

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << gpio),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG, "Button %d configured on GPIO %d", button_id, gpio);
}

static bool read_button_raw(button_id_t button_id)
{
    // Buttons are active LOW (pressed = 0)
    return (gpio_get_level(s_buttons[button_id].gpio) == 0);
}

static void update_button_state(button_id_t button_id)
{
    button_state_t *btn = &s_buttons[button_id];
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Read current raw state
    bool raw_state = read_button_raw(button_id);
    btn->previous_state = btn->current_state;
    
    // Simple debouncing: require stable state for DEBOUNCE_MS
    static uint32_t last_change_time[BUTTON_MAX] = {0};
    static bool last_raw_state[BUTTON_MAX] = {false};
    
    if (raw_state != last_raw_state[button_id]) {
        last_change_time[button_id] = current_time;
        last_raw_state[button_id] = raw_state;
        return; // Wait for debounce
    }
    
    if ((current_time - last_change_time[button_id]) > BUTTON_DEBOUNCE_MS) {
        btn->current_state = raw_state;
    }
    
    // Detect events
    if (btn->current_state && !btn->previous_state) {
        // Button pressed
        btn->press_time = current_time;
        btn->long_press_fired = false;
        btn->pending_event = BUTTON_EVENT_PRESS;
        ESP_LOGD(TAG, "Button %d pressed", button_id);
    }
    else if (!btn->current_state && btn->previous_state) {
        // Button released
        btn->release_time = current_time;
        uint32_t press_duration = btn->release_time - btn->press_time;
        
        if (!btn->long_press_fired) {
            if (press_duration >= BUTTON_LONG_PRESS_MS) {
                // This was already handled as long press
                btn->pending_event = BUTTON_EVENT_RELEASE;
            } else {
                // Check for double press
                static uint32_t last_release_time[BUTTON_MAX] = {0};
                if ((btn->release_time - last_release_time[button_id]) < BUTTON_DOUBLE_PRESS_MS) {
                    btn->pending_event = BUTTON_EVENT_DOUBLE_PRESS;
                    ESP_LOGD(TAG, "Button %d double pressed", button_id);
                } else {
                    btn->pending_event = BUTTON_EVENT_RELEASE;
                }
                last_release_time[button_id] = btn->release_time;
            }
        }
        ESP_LOGD(TAG, "Button %d released after %lu ms", button_id, press_duration);
    }
    else if (btn->current_state && btn->previous_state) {
        // Button held - check for long press
        uint32_t press_duration = current_time - btn->press_time;
        if (press_duration >= BUTTON_LONG_PRESS_MS && !btn->long_press_fired) {
            btn->pending_event = BUTTON_EVENT_LONG_PRESS;
            btn->long_press_fired = true;
            ESP_LOGD(TAG, "Button %d long pressed", button_id);
        }
    }
}

