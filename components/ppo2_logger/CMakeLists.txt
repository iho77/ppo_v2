idf_component_register(
    SRCS "ppo2_logger.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash esp_timer freertos sensor_manager app_common
)