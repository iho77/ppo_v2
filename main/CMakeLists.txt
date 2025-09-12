idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES freertos
                               esp_system
                               nvs_flash
                               app_common
                               button_manager
                               display_manager
                               warning_manager
                               esp_timer
                               sensor_manager
                               ppo2_logger
                               esp_driver_i2c)