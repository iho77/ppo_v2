idf_component_register(SRCS "display_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_common freertos
                               app_common sensor_manager warning_manager esp_lvgl_port esp_lcd_sh1107)