
#include <string.h>
#include <stdio.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "ms5611_spi.h"
#include "BNO08x.hpp"

#define PIN_NUM_VEXT GPIO_NUM_36
#define PIN_NUM_CS_MS GPIO_NUM_4
#define PIN_NUM_CS_BNO GPIO_NUM_3
#define PIN_NUM_SCLK GPIO_NUM_5
#define PIN_NUM_MISO GPIO_NUM_6
#define PIN_NUM_MOSI GPIO_NUM_7

static const char *TAG = "testf";

static spi_device_handle_t ms5611_spi_dev __attribute__((unused));
static ms5611_t ms5611 __attribute__((unused));


void init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_NUM_VEXT | 1ULL << PIN_NUM_CS_MS | 1ULL << PIN_NUM_CS_BNO ),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    // deselect device (CS HIGH)
    gpio_set_level(PIN_NUM_VEXT, 0); // turn on the VEXT RAIL
    vTaskDelay(pdMS_TO_TICKS(100)); // wiat for the power rail to settle

    gpio_set_level(PIN_NUM_CS_MS, 1); // disable MS5611
    gpio_set_level(PIN_NUM_CS_BNO, 1);
}

extern "C" void app_main(void)
{
    init();
    
    if (false)
    {
        static BNO08x imu;

        // initialize imu
        if (!imu.initialize())
        {
            ESP_LOGE(TAG, "Init failure, returning from main.");
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(100));

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

        while (1)
        {
            // delay time is irrelevant, we just don't want to trip WDT
            vTaskDelay(10000UL / portTICK_PERIOD_MS);
        }
    }
    else
    {

        static BNO08x imu;

        // initialize imu
        if (!imu.initialize())
        {
            ESP_LOGE(TAG, "Init failure, returning from main.");
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        // spi_bus_config_t buscfg = {};
        // buscfg.mosi_io_num = PIN_NUM_MOSI;
        // buscfg.miso_io_num = PIN_NUM_MISO;
        // buscfg.sclk_io_num = PIN_NUM_SCLK;
        // buscfg.quadwp_io_num = -1;
        // buscfg.quadhd_io_num = -1;
        // buscfg.max_transfer_sz = 8;

        // ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

        spi_device_interface_config_t devcfg = {};
        devcfg.clock_speed_hz = 1000000; // 1 MHz is a safe starting point
        devcfg.mode = 0;                 // MS5611 uses SPI mode 0
        devcfg.spics_io_num = PIN_NUM_CS_MS;
        devcfg.queue_size = 1;
        (void)devcfg;

        ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &ms5611_spi_dev));
        ESP_ERROR_CHECK(ms5611_spi_init(&ms5611, ms5611_spi_dev));

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

        int32_t pressure_pa = 0;
        printf("PROM: ");
        for (int i = 0; i < 7; ++i)
        {
            printf("C%d=0x%04X ", i, ms5611.C[i]);
        }
        printf("\n");
        while (1)
        {

            float temp_c = 0.0f;

            if (ms5611_read(&ms5611, MS5611_OSR_4096, &pressure_pa, &temp_c) == ESP_OK)
            {
                float pressure_hpa = pressure_pa / 100.0f;

                printf("MS5611: P=%6.2f Pa, T=%.2f C\n", pressure_hpa, temp_c);
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
