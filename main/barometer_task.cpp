#include "barometer_task.hpp"

#include <cmath>

#include "esp_log.h"
#include "esp_timer.h"
#include "ms5611_spi.h"

static const char *TAG = "BarometerTask";
static MS5611_SPI sensor(SPI3_HOST, GPIO_NUM_4);

bool BarometerTask::start()
{
    return startTask("Barometer", 4096, 5);
}

void BarometerTask::run()
{
    init_sensor();

    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20); // ~50 Hz

    while (!stopRequested())
    {
        const BarometerSample sample = read_sensor();
        latest_.write(sample);
        if (barometerSampleListener_)
        {
            barometerSampleListener_->onBarometerSampleReady();
        }
        vTaskDelayUntil(&last, period);
    }
}

void BarometerTask::init_sensor()
{
    ESP_LOGI(TAG, "Initializing barometer");

    if (!sensor.begin())
    {
        ESP_LOGE(TAG, "MS5611 init failed");
    }
}

BarometerSample BarometerTask::read_sensor()
{
    BarometerSample sample{};
    sample.t_us = esp_timer_get_time();

    const int result = sensor.read();
    if (result != MS5611_READ_OK)
    {
        sample.pressure = NAN;
        sample.temperature = NAN;
        return sample;
    }

    sample.pressure = sensor.getPressure();
    sample.temperature = sensor.getTemperature();
    return sample;
}
