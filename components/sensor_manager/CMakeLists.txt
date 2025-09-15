idf_component_register(SRCS "sensor_health.c" "sensor_calibration_storage.c" "sensor_calibration_math.c" "sensor_calibration.c" "sensor_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos
                               esp_common
                               app_common
                               esp_adc
                               driver
                        )