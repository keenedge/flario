idf_component_register(
  SRCS "src/ms5611_spi.cpp"
  INCLUDE_DIRS "include"
  PRIV_REQUIRES driver esp_rom freertos esp_timer
)
