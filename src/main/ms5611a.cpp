#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "ms5611_spi.h"

#include <cstring>

static constexpr const char *TAG = "ms5611_app";

extern "C" void app_main(void)
{
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = GPIO_NUM_6;
  buscfg.miso_io_num = GPIO_NUM_5;
  buscfg.sclk_io_num = GPIO_NUM_7;
  buscfg.quadwp_io_num = GPIO_NUM_NC;
  buscfg.quadhd_io_num = GPIO_NUM_NC;
  buscfg.max_transfer_sz = 32;

  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

  MS5611_SPI sensor(SPI2_HOST, GPIO_NUM_4);
  if (!sensor.begin())
  {
    ESP_LOGE(TAG, "MS5611 init failed");
    return;
  }

  while (true)
  {
    int result = sensor.read();
    if (result == MS5611_READ_OK)
    {
      ESP_LOGI(TAG, "Temp %.2f C, Pressure %.2f mbar", sensor.getTemperature(), sensor.getPressure());
    }
    else
    {
      ESP_LOGW(TAG, "MS5611 read failed (code %d)", result);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
