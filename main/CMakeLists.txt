idf_component_register(
    SRCS
        "ms5611_spi.c"
        "testf.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_driver_gpio
        esp_driver_spi
        esp_timer
    PRIV_REQUIRES
        esp32_BNO08x
)

set_source_files_properties(testf.c PROPERTIES LANGUAGE CXX)
