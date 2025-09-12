idf_component_register(SRCS "button_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver
                               freertos
                               esp_common
                               app_common)