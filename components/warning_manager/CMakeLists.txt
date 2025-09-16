idf_component_register(
    SRCS "warning_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES driver log freertos app_common led_strip esp_timer sensor_manager
)