idf_component_register(
    SRCS
        "barometer_task.cpp"
        "display_task.cpp"
        "encoder.c"
        "imu_task.cpp"
        "ms5611_spi.c"
        "telemetry.cpp"
        "testf.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_driver_i2c
        esp_driver_gpio
        esp_driver_pcnt
        esp_driver_spi
        esp_lcd
        esp_timer
    PRIV_REQUIRES
        esp32_BNO08x

        
)

set_source_files_properties(testf.c PROPERTIES LANGUAGE CXX)
