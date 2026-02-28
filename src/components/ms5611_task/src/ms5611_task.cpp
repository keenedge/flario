#include "ms5611_task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "ms5611_spi.h"

namespace
{
static constexpr const char *TAG = "ms5611_task";
static QueueHandle_t s_queue = nullptr;
static TaskHandle_t s_task_handle = nullptr;
static SemaphoreHandle_t s_queue_ready = nullptr;

void ms5611_task_entry(void *arg)
{
  const size_t queue_length = 4;
  s_queue = xQueueCreate(queue_length, sizeof(ms5611_reading_t));
  if (s_queue == nullptr)
  {
    ESP_LOGE(TAG, "failed to create queue");
    if (s_queue_ready != nullptr)
    {
      xSemaphoreGive(s_queue_ready);
    }
    vTaskDelete(nullptr);
    return;
  }

  if (s_queue_ready != nullptr)
  {
    xSemaphoreGive(s_queue_ready);
  }

  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = GPIO_NUM_6;
  buscfg.miso_io_num = GPIO_NUM_5;
  buscfg.sclk_io_num = GPIO_NUM_7;
  buscfg.quadwp_io_num = GPIO_NUM_NC;
  buscfg.quadhd_io_num = GPIO_NUM_NC;
  buscfg.max_transfer_sz = 32;

  esp_err_t bus_err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if (bus_err != ESP_OK && bus_err != ESP_ERR_INVALID_STATE)
  {
    ESP_LOGE(TAG, "failed to initialize SPI bus (%s)", esp_err_to_name(bus_err));
    vTaskDelete(nullptr);
    return;
  }

  MS5611_SPI sensor(SPI2_HOST, GPIO_NUM_4);
  if (!sensor.begin())
  {
    ESP_LOGE(TAG, "failed to initialize MS5611");
    vTaskDelete(nullptr);
    return;
  }

  while (true)
  {
    ms5611_reading_t reading{};
    reading.result_code = sensor.read();
    reading.temperature_celsius = sensor.getTemperature();
    reading.pressure_mbar = sensor.getPressure();
    reading.timestamp_ms = sensor.lastRead();

    if (xQueueSend(s_queue, &reading, 0) != pdPASS)
    {
      ESP_LOGW(TAG, "queue full, dropping reading");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
} // namespace

esp_err_t ms5611_task_start(void)
{
  if (s_task_handle != nullptr)
  {
    return ESP_OK;
  }

  if (s_queue_ready == nullptr)
  {
    s_queue_ready = xSemaphoreCreateBinary();
    if (s_queue_ready == nullptr)
    {
      return ESP_ERR_NO_MEM;
    }
  }

  if (xTaskCreate(ms5611_task_entry, "ms5611_task", 4096, nullptr, 5, &s_task_handle) != pdPASS)
  {
    vSemaphoreDelete(s_queue_ready);
    s_queue_ready = nullptr;
    return ESP_ERR_NO_MEM;
  }

  if (xSemaphoreTake(s_queue_ready, pdMS_TO_TICKS(2000)) != pdTRUE)
  {
    ESP_LOGE(TAG, "queue setup timed out");
    return ESP_ERR_TIMEOUT;
  }

  if (s_queue == nullptr)
  {
    return ESP_FAIL;
  }

  return ESP_OK;
}

QueueHandle_t ms5611_task_get_queue(void)
{
  return s_queue;
}
