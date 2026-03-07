#include "imu_task.hpp"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ImuTask";

bool ImuTask::start()
{
    return startTask("imu", 8192, 5);
}

void ImuTask::run()
{
    bno08x_config_t imuCfg_;  /// read config
    BNO08x imu(imuCfg_); // create teh sensor

    ESP_LOGI(TAG, "Initializing IMU");
    if (!imu.initialize()) // initialize the sensor
    {
        ESP_LOGE(TAG, "IMU init failure");
        return; // TaskBase entryPoint will vTaskDelete(nullptr) after run() returns
    }
    ESP_LOGI(TAG, "IMU Initialized");

    // Preserve your report enables exactly
    imu.rpt.rv_game.enable(100000UL);
    imu.rpt.cal_gyro.enable(100000UL);
    imu.rpt.gravity.enable(100000UL);
    imu.rpt.accelerometer.enable(100000UL);
    imu.rpt.linear_accelerometer.enable(100000UL);

    ESP_LOGE(TAG, "IMU Register Callback");

    imu.register_cb(
        [this, &imu]()
        {
            ImuSample sample = {};
            bool has_any = false;
            sample.t_us = esp_timer_get_time();

            if (imu.rpt.rv_game.has_new_data())
            {
                sample.has_euler = true;
                sample.euler = imu.rpt.rv_game.get_euler();
                has_any = true;
            }

            if (imu.rpt.cal_gyro.has_new_data())
            {
                sample.has_gyro = true;
                sample.gyro = imu.rpt.cal_gyro.get();
                has_any = true;
            }

            if (imu.rpt.gravity.has_new_data())
            {
                sample.has_gravity = true;
                sample.gravity = imu.rpt.gravity.get();
                has_any = true;
            }

            if (imu.rpt.accelerometer.has_new_data())
            {
                sample.has_accel = true;
                sample.accel = imu.rpt.accelerometer.get();
                has_any = true;
            }

            if (imu.rpt.linear_accelerometer.has_new_data())
            {
                sample.has_linear_accel = true;
                sample.linear_accel = imu.rpt.linear_accelerometer.get();
                has_any = true;
            }

            if (has_any)
            {
                latest_.write(sample);
                if (imuSampleListener_) {
                    imuSampleListener_->onImuSampleReady();
                }
            }
        });

    while (!stopRequested())
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
