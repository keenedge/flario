
#include <string.h>
#include <stdio.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "display_task.hpp"
#include "imu_task.hpp"
#include "barometer_task.hpp"

#define PIN_NUM_VEXT GPIO_NUM_36
#define PIN_NUM_CS_MS GPIO_NUM_4
#define PIN_NUM_CS_BNO GPIO_NUM_33


static const char *TAG = "testf";

void init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_NUM_VEXT | 1ULL << PIN_NUM_CS_MS | 1ULL << PIN_NUM_CS_BNO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    // deselect device (CS HIGH)
    gpio_set_level(PIN_NUM_VEXT, 0); // turn on the VEXT RAIL
    vTaskDelay(pdMS_TO_TICKS(100));  // wiat for the power rail to settle

    gpio_set_level(PIN_NUM_CS_MS, 1); // disable MS5611
    gpio_set_level(PIN_NUM_CS_BNO, 1);
}

extern "C" void app_main(void)
{
    gpio_dump_io_configuration(stdout, (1ULL << 3) | (1ULL << 26) | (1ULL << 47) | (1ULL << 48));

    init();

    imu_start();

    vTaskDelay(pdMS_TO_TICKS(100));

    barometer_start();

    display_start();

    int counter = 0;
    while (1)
    {
        ESP_LOGI(TAG, "app_main loop %d", counter++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
