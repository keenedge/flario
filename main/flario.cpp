#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "fusion_task.hpp"
#include "ui_task.hpp"
#include "imu_task.hpp"
#include "barometer_task.hpp"

static const char *TAG = "main";

void initGPIO( void );
void initSPI( void );

extern "C" void app_main(void)
{
    initGPIO();
//    initSPI();

    static ImuTask imuTask;
    static BarometerTask barometerTask;
    static FusionTask fusionTask;
    static UiTask uiTask;

    configASSERT(imuTask.start());
    vTaskDelay(pdMS_TO_TICKS(500));        
    configASSERT(barometerTask.start());
    configASSERT(fusionTask.start(imuTask, barometerTask));
    configASSERT(uiTask.start(fusionTask));

    ESP_LOGI(TAG, "Start Main Task Loop");
    int count = 0;
    while (true)
    {
        ESP_LOGI( TAG, "Main Loop: %d", count++);
        vTaskDelay(pdMS_TO_TICKS(5000));        
    }
}

void initGPIO( void ) {
    gpio_config_t boot_gpio = {};
    boot_gpio.pin_bit_mask = (1ULL << GPIO_NUM_3) | (1ULL << GPIO_NUM_4);
    boot_gpio.mode = GPIO_MODE_OUTPUT;
    boot_gpio.pull_up_en = GPIO_PULLUP_DISABLE;
    boot_gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    boot_gpio.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&boot_gpio));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_3, 1));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_4, 1));
}

void initSPI( void ) {
    bno08x_config_t imu_cfg = bno08x_config_t();
  
    spi_bus_config_t bus_cfg = {};
    bus_cfg.sclk_io_num = imu_cfg.io_sclk;
    bus_cfg.mosi_io_num = imu_cfg.io_mosi;
    bus_cfg.miso_io_num = imu_cfg.io_miso;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 0;
    bus_cfg.flags = SPICOMMON_BUSFLAG_MASTER;

    esp_err_t ret = spi_bus_initialize(imu_cfg.spi_peripheral, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
    }

}
