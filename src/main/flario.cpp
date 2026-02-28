#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ms5611_spi.h"
#include "ms5611_task.h"

static constexpr const char *TAG = "flario_app";

extern "C" void app_main(void)
{
  if (ms5611_task_start() != ESP_OK)
  {
    ESP_LOGE(TAG, "failed to start MS5611 background task");
    return;
  }

  QueueHandle_t queue = ms5611_task_get_queue();
  if (queue == nullptr)
  {
    ESP_LOGE(TAG, "sensor queue unavailable");
    return;
  }

  while (true)
  {
    ms5611_reading_t reading{};
    if (xQueueReceive(queue, &reading, pdMS_TO_TICKS(1500)) == pdTRUE)
    {
      if (reading.result_code == MS5611_READ_OK)
      {
        ESP_LOGI(TAG, "Temp %.2f C, Pressure %.2f mbar", reading.temperature_celsius, reading.pressure_mbar);
      }
      else
      {
        ESP_LOGW(TAG, "MS5611 read failed (code %d)", reading.result_code);
      }
    }
    else
    {
      ESP_LOGW(TAG, "timeout waiting for MS5611 data");
    }
  }
}
