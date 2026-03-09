#include "imu_task.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include <freertos/task.h>

static const char *TAG = "ImuTask";

bool ImuTask::start()
{
    return startTask("imu", 8192, 5);
}

void ImuTask::onHostInterrupt(void)
{
    BaseType_t hp_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandle(), &hp_task_woken);

    if (hp_task_woken)
    {
        portYIELD_FROM_ISR();
    }
}

void ImuTask::processAllPendingReports()
{
            ESP_LOGI(TAG, "processing");

    // Keep pulling reports until the library says no more are ready
    while (imu_.data_available())
    {
            ESP_LOGI(TAG, "available");
        ImuSample sample = {};

        bool has_any = false;
        sample.t_us = esp_timer_get_time();

        if (imu_.rpt.rv_game.has_new_data())
        {
            ESP_LOGI(TAG, "game");

            sample.has_euler = true;
            sample.euler = imu_.rpt.rv_game.get_euler();
            has_any = true;
        }

        if (imu_.rpt.cal_gyro.has_new_data())
        {
            ESP_LOGI(TAG, "gyro");
            sample.has_gyro = true;
            sample.gyro = imu_.rpt.cal_gyro.get();
            has_any = true;
        }

        if (imu_.rpt.gravity.has_new_data())
        {
            ESP_LOGI(TAG, "gravity");
            sample.has_gravity = true;
            sample.gravity = imu_.rpt.gravity.get();
            has_any = true;
        }

        if (imu_.rpt.accelerometer.has_new_data())
        {
            ESP_LOGI(TAG, "accel");
            sample.has_accel = true;
            sample.accel = imu_.rpt.accelerometer.get();
            has_any = true;
        }

        if (imu_.rpt.linear_accelerometer.has_new_data())
        {
            ESP_LOGI(TAG, "la");
            sample.has_linear_accel = true;
            sample.linear_accel = imu_.rpt.linear_accelerometer.get();
            has_any = true;
        }

        if (has_any)
        {
            latest_.write(sample);
            if (imuSampleListener_)
            {
                imuSampleListener_->onImuSampleReady();
            }
        }
    };
}

void ImuTask::run()
{
    ESP_LOGI(TAG, "Initializing IMU");
    if (!imu_.initialize()) // initialize the sensor
    {
        ESP_LOGE(TAG, "IMU init failure");
        return; // TaskBase entryPoint will vTaskDelete(nullptr) after run() returns
    }
    ESP_LOGI(TAG, "IMU Initialized");

    // Preserve your report enables exactly
    imu_.rpt.rv_game.enable(100000UL);
    imu_.rpt.cal_gyro.enable(100000UL);
    imu_.rpt.gravity.enable(100000UL);
    imu_.rpt.accelerometer.enable(100000UL);
    imu_.rpt.linear_accelerometer.enable(100000UL);

    ESP_LOGI(TAG, "IMU Register Callback");

    // imu_.register_cb( []()
    // {
    //     ESP_LOGI(TAG, "Hi");

    //   //onHostInterrupt();
    // });

    while (1)
    {
        //uint32_t count = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10000));
        //if (count == 0)
        //{
        //    ESP_LOGI(TAG, "ulTaskNotifyTake timeout");
       // }
        //ESP_LOGI(TAG, "Wake in HINT %d", count);

            processAllPendingReports();
            vTaskDelay(pdMS_TO_TICKS(500));    

    }
}
