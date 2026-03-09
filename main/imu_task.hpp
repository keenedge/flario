#pragma once

#include <stdint.h>

#include "task_base.hpp"
#include "latest.hpp"

#include "BNO08x.hpp"
#include "samples.hpp"

#include "imu_sample_listener.hpp"

class ImuTask : public TaskBase
{
public:
    bool start();

    void registerListener(ImuSampleListener *listener)
    {
        imuSampleListener_ = listener;
    }

    ImuSample latestSample() const
    {
        return latest_.read();
    }

protected:
    void run() override;

private:
    BNO08x imu_;
    // void processAllPendingReports();
    // void onHostInterrupt(void);
    Latest<ImuSample> latest_;
    ImuSampleListener *imuSampleListener_ = nullptr;
    enum ReportNotifyBits : uint32_t
    {
        NOTIFY_RV_GAME = (1UL << 0),
        NOTIFY_CAL_GYRO = (1UL << 1),
        NOTIFY_GRAVITY = (1UL << 2),
        NOTIFY_ACCEL = (1UL << 3),
        NOTIFY_LIN_ACCEL = (1UL << 4),
    };

    static void onHostInterrupt_rv_game(TaskHandle_t handle);
    static void onHostInterrupt_cal_gyro(TaskHandle_t handle);
    static void onHostInterrupt_acceleration(TaskHandle_t handle);
    static void onHostInterrupt_linear_acceleration(TaskHandle_t handle);
    static void onHostInterrupt_gravity(TaskHandle_t handle);

    size_t drainSamples(uint32_t notify_bits);
    void printStatusPanel(uint32_t report_bits, bool timed_out, const ImuSample &sample);
    static constexpr uint32_t RV_GAME_INTERVAL_US = 10000UL;
    static constexpr uint32_t CAL_GYRO_INTERVAL_US = 10000UL;
    static constexpr uint32_t GRAVITY_INTERVAL_US = 10000UL;
    static constexpr uint32_t ACCEL_INTERVAL_US = 10000UL;
    static constexpr uint32_t LIN_ACCEL_INTERVAL_US = 10000UL;
};
