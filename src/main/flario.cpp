#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "ms5611_spi.h"

#include <cstring>
#include <stdio.h>



#include "BNO08x.hpp"

static const constexpr char *TAG = "Main";

extern "C" void app_main(void)
{
    gpio_config_t boot_gpio = {};
    boot_gpio.pin_bit_mask = (1ULL << GPIO_NUM_3) | (1ULL << GPIO_NUM_4);
    boot_gpio.mode = GPIO_MODE_OUTPUT;
    boot_gpio.pull_up_en = GPIO_PULLUP_DISABLE;
    boot_gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    boot_gpio.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&boot_gpio));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_3, 1));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_4, 1));

    static BNO08x imu;

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
    // see BNO08x::bno08x_reports_t for all possible reports to enable

    // register callback to execute for all reports, 2 different methods

    // method 1, void input param:
    imu.register_cb(
        []()
        {
            if (imu.rpt.rv_game.has_new_data())
            {
                bno08x_euler_angle_t euler = imu.rpt.rv_game.get_euler();
                ESP_LOGW(TAG, "Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg]", euler.x, euler.y, euler.z);
            }

            if (imu.rpt.cal_gyro.has_new_data())
            {
                bno08x_gyro_t velocity = imu.rpt.cal_gyro.get();
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
                ESP_LOGW(TAG, "Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", ang_accel.x, ang_accel.y, ang_accel.z);
            }

            if (imu.rpt.linear_accelerometer.has_new_data())
            {
                bno08x_accel_t lin_accel = imu.rpt.accelerometer.get();
                ESP_LOGW(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel.x, lin_accel.y, lin_accel.z);
            }
        });

    // method 2, report ID param (comment method 1 out before commenting this in):
    /*
    imu.register_cb(
            [](uint8_t rpt_ID)
            {
                static bno08x_euler_angle_t euler;
                static bno08x_gyro_t velocity;
                static bno08x_accel_t grav;
                static bno08x_accel_t ang_accel;
                static bno08x_accel_t lin_accel;

                switch (rpt_ID)
                {
                    case SH2_GAME_ROTATION_VECTOR:
                        euler = imu.rpt.rv_game.get_euler();
                        ESP_LOGW(TAG, "Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg]", euler.x, euler.y,
                                euler.z);
                        break;

                    case SH2_CAL_GYRO:
                        velocity = imu.rpt.cal_gyro.get();
                        ESP_LOGW(TAG, "Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s]", velocity.x, velocity.y, velocity.z);
                        break;

                    case SH2_GRAVITY:
                        grav = imu.rpt.gravity.get();
                        ESP_LOGW(TAG, "Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2]", grav.x, grav.y, grav.z);
                        break;

                    case SH2_ACCELEROMETER:
                        ang_accel = imu.rpt.accelerometer.get();
                        ESP_LOGW(TAG, "Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", ang_accel.x, ang_accel.y, ang_accel.z);
                        break;

                    case SH2_LINEAR_ACCELERATION:
                        lin_accel = imu.rpt.accelerometer.get();
                        ESP_LOGW(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel.x, lin_accel.y, lin_accel.z);
                        break;

                    default:

                        break;
                }
            });

    */

    MS5611_SPI sensor(SPI3_HOST, GPIO_NUM_4);
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