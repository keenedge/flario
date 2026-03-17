#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "imu_task.hpp"
#include "BNO08x.hpp"
#include "telemetry.hpp"

static const char *TAG = "TASK_EXAMPLE";
static BNO08x imu;

void imu_start()
{

    // initialize imu
    if (!imu.initialize())
    {
        ESP_LOGE(TAG, "Init failure, returning from main.");
        return;
    }

    // enable game rotation vector and calibrated gyro reports
    imu.rpt.rv_game.enable(100000UL); // 100,000us == 100ms report interval
    imu.rpt.cal_gyro.enable(100000UL);
    imu.rpt.gravity.enable(100000UL);
    imu.rpt.accelerometer.enable(100000UL);
    imu.rpt.linear_accelerometer.enable(100000UL);

    imu.register_cb(
        []()
        {
            if (imu.rpt.rv_game.has_new_data())
            {
                bno08x_euler_angle_t euler = imu.rpt.rv_game.get_euler();
                telemetry_store_imu_euler(euler.x, euler.y, euler.z);
                ESP_LOGW(TAG, "Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg]", euler.x, euler.y, euler.z);
            }

            if (imu.rpt.cal_gyro.has_new_data())
            {
                bno08x_gyro_t velocity = imu.rpt.cal_gyro.get();
                telemetry_store_imu_gyro(velocity.x, velocity.y, velocity.z);
                ESP_LOGW(TAG, "Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s]", velocity.x, velocity.y, velocity.z);
            }

            if (imu.rpt.gravity.has_new_data())
            {
                bno08x_accel_t grav = imu.rpt.gravity.get();
                ESP_LOGW(TAG, "Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2]", grav.x, grav.y, grav.z);
            }

            if (imu.rpt.accelerometer.has_new_data())
            {
                bno08x_accel_t ang_accel = imu.rpt.accelerometer.get();
                telemetry_store_imu_accel(ang_accel.x, ang_accel.y, ang_accel.z);
                ESP_LOGW(TAG, "Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", ang_accel.x, ang_accel.y, ang_accel.z);
            }

            if (imu.rpt.linear_accelerometer.has_new_data())
            {
                bno08x_accel_t lin_accel = imu.rpt.linear_accelerometer.get();
                telemetry_store_imu_linear_accel(lin_accel.x, lin_accel.y, lin_accel.z);
                ESP_LOGW(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel.x, lin_accel.y, lin_accel.z);
            }
        });

    xTaskCreate(
        imu_task,        // Task function
        "my_task",      // Task name
        2048,           // Stack size in bytes
        NULL,           // Task parameter
        5,              // Priority
        NULL            // Task handle
    );    

}

void imu_task(void *arg)
{
    int counter = 0;

    while (true)
    {
        ESP_LOGI(TAG, "imu main loop %d", counter++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
