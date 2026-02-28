#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
  float     temperature_celsius;
  float     pressure_mbar;
  int       result_code;
  uint32_t  timestamp_ms;
} ms5611_reading_t;

/**
 * @brief Start the MS5611 background task and initialize the queue.
 *
 * @return esp_err_t ESP_OK if the task was created and the queue is ready.
 */
esp_err_t ms5611_task_start(void);

/**
 * @brief Get the queue that the MS5611 task publishes readings to.
 *
 * @return QueueHandle_t Queue handle, or NULL if the task has not finished setup.
 */
QueueHandle_t ms5611_task_get_queue(void);

#ifdef __cplusplus
}
#endif
