#pragma once

#include "task_base.hpp"
#include "samples.hpp"
#include "latest.hpp"
#include "imu_task.hpp"
#include "barometer_task.hpp"

#include "imu_sample_listener.hpp"
#include "barometer_sample_listener.hpp"
#include "fusion_sample_listener.hpp"

class FusionTask : 
    public TaskBase, 
    public ImuSampleListener,
    public BarometerSampleListener
{
public:
    bool start(ImuTask& imu, BarometerTask& barometer);

    FusionSample latestOutput() const
    {
        return latestFusionSample_.read();
    }

    void registerListener(FusionSampleListener* listener)
    {
        fusionSampleListener_ = listener;
    }

    void onBarometerSampleReady() override;
    void onImuSampleReady() override;

protected:
    void run() override;

private:
    FusionSample fuse(const ImuSample& imu, const BarometerSample& barometer);

private:
    static constexpr uint32_t EVT_IMU_SAMPLE_READY = 1u << 0;
    static constexpr uint32_t EVT_BAROMETER_SAMPLE_READY = 1u << 0;

    FusionSampleListener* fusionSampleListener_ = nullptr;

    ImuTask* imu_ = nullptr;
    BarometerTask* barometer_ = nullptr;
    Latest<FusionSample> latestFusionSample_;

    bool have_alt_ = false;
    //float last_alt_m_ = 0.0f;
    //int64_t last_t_us_ = 0;
};
