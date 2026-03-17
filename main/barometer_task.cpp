#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "barometer_task.hpp"
#include "ms5611_spi.h"
#include "telemetry.hpp"

static const char *TAG = "BARO_TASK";
static constexpr gpio_num_t PIN_NUM_CS_MS = GPIO_NUM_4;

static spi_device_handle_t ms5611_spi_dev = nullptr;
static ms5611_t ms5611 = {};

void barometer_start()
{
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 1000000; // 1 MHz is a safe starting point
    devcfg.mode = 0;                 // MS5611 uses SPI mode 0
    devcfg.spics_io_num = PIN_NUM_CS_MS;
    devcfg.queue_size = 1;

    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &ms5611_spi_dev));
    ESP_ERROR_CHECK(ms5611_spi_init(&ms5611, ms5611_spi_dev));

    xTaskCreate(
        barometer_task,
        "barometer_task",
        3072,
        nullptr,
        5,
        nullptr);
}

void barometer_task(void *arg)
{
    (void)arg;

    int32_t pressure_pa = 0;
    printf("PROM: ");
    for (int i = 0; i < 7; ++i)
    {
        printf("C%d=0x%04X ", i, ms5611.C[i]);
    }
    printf("\n");

    int counter = 0;
    while (true)
    {
        ESP_LOGI(TAG, "barometer main loop %d", counter++);

        float temp_c = 0.0f;
        if (ms5611_read(&ms5611, MS5611_OSR_4096, &pressure_pa, &temp_c) == ESP_OK)
        {
            float pressure_hpa = pressure_pa / 100.0f;
            telemetry_store_baro(pressure_pa, temp_c);
            printf("MS5611: P=%6.2f hPa, T=%.2f C\n", pressure_hpa, temp_c);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
