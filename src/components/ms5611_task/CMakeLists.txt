idf_component_register(
  SRCS "src/ms5611_task.cpp"
  INCLUDE_DIRS "include"
  REQUIRES freertos driver ms5611_spi
)
