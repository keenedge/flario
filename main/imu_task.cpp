#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>

#include "imu_task.hpp"

static const char *TAG = "ImuTask";

static Latest<RvGameSample> latest_rv_game;
static Latest<CalGyroSample> latest_cal_gyro;
static Latest<GravitySample> latest_gravity;
static Latest<AccelSample> latest_accel;
static Latest<LinearAccelSample> latest_linear_accel;

static ImuSample readLatestImuSample(void)
{
    ImuSample sample{};

    const RvGameSample rv = latest_rv_game.read();
    if (rv.has)
    {
        sample.has_euler = true;
        sample.euler = rv.value;
        sample.t_us = rv.t_us;
    }

    const CalGyroSample gyro = latest_cal_gyro.read();
    if (gyro.has)
    {
        sample.has_gyro = true;
        sample.gyro = gyro.value;
        if (gyro.t_us > sample.t_us)
            sample.t_us = gyro.t_us;
    }

    const GravitySample gravity = latest_gravity.read();
    if (gravity.has)
    {
        sample.has_gravity = true;
        sample.gravity = gravity.value;
        if (gravity.t_us > sample.t_us)
            sample.t_us = gravity.t_us;
    }

    const AccelSample accel = latest_accel.read();
    if (accel.has)
    {
        sample.has_accel = true;
        sample.accel = accel.value;
        if (accel.t_us > sample.t_us)
            sample.t_us = accel.t_us;
    }

    const LinearAccelSample lin = latest_linear_accel.read();
    if (lin.has)
    {
        sample.has_linear_accel = true;
        sample.linear_accel = lin.value;
        if (lin.t_us > sample.t_us)
            sample.t_us = lin.t_us;
    }

    return sample;
}

void ImuTask::printStatusPanel(uint32_t report_bits, bool timed_out, const ImuSample &sample)
{
    static bool panel_drawn = false;
    static constexpr int PANEL_LINES = 3;

    if (panel_drawn)
    {
        printf("[%dA", PANEL_LINES);
    }

    printf("[Main] bits=0x%08lx timeout=%1d",
           static_cast<unsigned long>(report_bits),
           timed_out ? 1 : 0);

    char sample_line[320];
    int pos = 0;
    pos += snprintf(sample_line + pos, sizeof(sample_line) - static_cast<size_t>(pos), "[Sample]");

    if (sample.has_euler)
    {
        pos += snprintf(sample_line + pos, sizeof(sample_line) - static_cast<size_t>(pos),
                        " euler=(%8.2f,%8.2f,%8.2f)", sample.euler.x, sample.euler.y, sample.euler.z);
    }
    if (sample.has_gyro)
    {
        pos += snprintf(sample_line + pos, sizeof(sample_line) - static_cast<size_t>(pos),
                        " gyro=(%8.2f,%8.2f,%8.2f)", sample.gyro.x, sample.gyro.y, sample.gyro.z);
    }
    if (sample.has_accel)
    {
        pos += snprintf(sample_line + pos, sizeof(sample_line) - static_cast<size_t>(pos),
                        " accel=(%8.2f,%8.2f,%8.2f)", sample.accel.x, sample.accel.y, sample.accel.z);
    }
    if (sample.has_linear_accel)
    {
        pos += snprintf(sample_line + pos, sizeof(sample_line) - static_cast<size_t>(pos),
                        " lin=(%8.2f,%8.2f,%8.2f)", sample.linear_accel.x, sample.linear_accel.y, sample.linear_accel.z);
    }
    if (sample.has_gravity)
    {
        pos += snprintf(sample_line + pos, sizeof(sample_line) - static_cast<size_t>(pos),
                        " grav=(%8.2f,%8.2f,%8.2f)", sample.gravity.x, sample.gravity.y, sample.gravity.z);
    }
    if (pos == static_cast<int>(sizeof("[Sample]") - 1))
    {
        pos += snprintf(sample_line + pos, sizeof(sample_line) - static_cast<size_t>(pos), " none");
    }

    printf("%s\n", sample_line);

    panel_drawn = true;
}

void ImuTask::onHostInterrupt_rv_game(TaskHandle_t handle)
{
    if (handle != nullptr)
        xTaskNotify(handle, NOTIFY_RV_GAME, eSetBits);
}

void ImuTask::onHostInterrupt_cal_gyro(TaskHandle_t handle)
{
    if (handle != nullptr)
        xTaskNotify(handle, NOTIFY_CAL_GYRO, eSetBits);
}

void ImuTask::onHostInterrupt_gravity(TaskHandle_t handle)
{
    if (handle != nullptr)
        xTaskNotify(handle, NOTIFY_GRAVITY, eSetBits);
}

void ImuTask::onHostInterrupt_acceleration(TaskHandle_t handle)
{
    if (handle != nullptr)
        xTaskNotify(handle, NOTIFY_ACCEL, eSetBits);
}

void ImuTask::onHostInterrupt_linear_acceleration(TaskHandle_t handle)
{
    if (handle != nullptr)
        xTaskNotify(handle, NOTIFY_LIN_ACCEL, eSetBits);
}

size_t ImuTask::drainSamples(uint32_t notify_bits)
{
    size_t emitted = 0;
    const TickType_t start = xTaskGetTickCount();
    TickType_t budget = pdMS_TO_TICKS(2); // prevent monopolizing CPU
    if (budget == 0)
        budget = 1; // at 100 Hz tick, 2 ms -> 0 ticks

    while ((xTaskGetTickCount() - start) < budget)
    {
        bool any = false;

        if ((notify_bits & NOTIFY_RV_GAME) && imu_.rpt.rv_game.has_new_data())
        {
            RvGameSample sample{};
            sample.t_us = esp_timer_get_time();
            sample.value = imu_.rpt.rv_game.get_euler();
            sample.has = true;
            latest_rv_game.write(sample);
            any = true;
            // sampleCount++;
            // totalCount++;
            emitted++;
        }

        if ((notify_bits & NOTIFY_CAL_GYRO) && imu_.rpt.cal_gyro.has_new_data())
        {
            CalGyroSample sample{};
            sample.t_us = esp_timer_get_time();
            sample.value = imu_.rpt.cal_gyro.get();
            sample.has = true;
            latest_cal_gyro.write(sample);
            any = true;
            emitted++;
        }

        if ((notify_bits & NOTIFY_GRAVITY) && imu_.rpt.gravity.has_new_data())
        {
            GravitySample sample{};
            sample.t_us = esp_timer_get_time();
            sample.value = imu_.rpt.gravity.get();
            sample.has = true;
            latest_gravity.write(sample);
            any = true;
            emitted++;
        }

        if ((notify_bits & NOTIFY_ACCEL) && imu_.rpt.accelerometer.has_new_data())
        {
            AccelSample sample{};
            sample.t_us = esp_timer_get_time();
            sample.value = imu_.rpt.accelerometer.get();
            sample.has = true;
            latest_accel.write(sample);
            any = true;
            emitted++;
        }

        if ((notify_bits & NOTIFY_LIN_ACCEL) && imu_.rpt.linear_accelerometer.has_new_data())
        {
            LinearAccelSample sample{};
            sample.t_us = esp_timer_get_time();
            sample.value = imu_.rpt.linear_accelerometer.get();
            sample.has = true;
            latest_linear_accel.write(sample);
            any = true;
            emitted++;
        }
        if (!any)
        {
            break;
        }
    }

    return emitted;
}

bool ImuTask::start()
{
    return startTask("imu", 8192, 5);
}

void ImuTask::run()
{
    ESP_LOGI(TAG, "tiny_task started");
    ESP_LOGI(TAG, "Starting imu_.initialize()");
    if (!imu_.initialize())
    {
        ESP_LOGE(TAG, "Init failure, returning from main.");
        return;
    }
    ESP_LOGI(TAG, "imu_.initialize() complete");

    imu_.rpt.rv_game.enable(RV_GAME_INTERVAL_US); // 100,000us == 100ms report interval
    imu_.rpt.cal_gyro.enable(CAL_GYRO_INTERVAL_US);
    imu_.rpt.gravity.enable(GRAVITY_INTERVAL_US);
    imu_.rpt.accelerometer.enable(ACCEL_INTERVAL_US);
    imu_.rpt.linear_accelerometer.enable(LIN_ACCEL_INTERVAL_US);

    bool cb_ok = true;
    cb_ok &= imu_.rpt.rv_game.register_cb([this]()
                                          { onHostInterrupt_rv_game(taskHandle()); });
    cb_ok &= imu_.rpt.cal_gyro.register_cb([this]()
                                           { onHostInterrupt_cal_gyro(taskHandle()); });
    cb_ok &= imu_.rpt.gravity.register_cb([this]()
                                          { onHostInterrupt_gravity(taskHandle()); });
    cb_ok &= imu_.rpt.accelerometer.register_cb([this]()
                                                { onHostInterrupt_acceleration(taskHandle()); });
    cb_ok &= imu_.rpt.linear_accelerometer.register_cb([this]()
                                                       { onHostInterrupt_linear_acceleration(taskHandle()); });

    if (!cb_ok)
    {
        ESP_LOGE(TAG, "Frequency diag: failed to register one or more report callbacks.");
    }

    static const int64_t PANEL_REFRESH_US = 200000LL; // 5Hz panel update
    int64_t last_panel_us = 0;

    for (;;)
    {
        uint32_t report_bits = 0U;
        BaseType_t woke = xTaskNotifyWait(0UL, 0xFFFFFFFFUL, &report_bits, pdMS_TO_TICKS(100));

        // Try to drain pending report flags a few times, then yield.
        for (int pass = 0; pass < 4; ++pass)
        {
            size_t drained = drainSamples(report_bits);
            if (drained == 0)
            {
                break; // nothing pending now
            }
        }

        ImuSample sample = readLatestImuSample();
        const int64_t now_us = esp_timer_get_time();
        if ((last_panel_us == 0) || ((now_us - last_panel_us) >= PANEL_REFRESH_US))
        {
            printStatusPanel(report_bits, (woke != pdTRUE), sample);
            last_panel_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
