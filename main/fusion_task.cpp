#include "fusion_task.hpp"

#include <cstdint>
#include <cmath>

#include "esp_log.h"

static const char *TAG = "FusionTask";

// This function converts air pressure to altitude using the International Standard Atmosphere (ISA) barometric formula.
// It estimates the altitude above a reference pressure (usually sea level).
static float pressureBarToAltitudeMeters(float p, float p0)
{
    if (p <= 0.0f || p0 <= 0.0f)
    {
        return NAN;
    }

    const float ratio = p / p0;

    return 44330.0f * (1.0f - powf(ratio, 0.1903f));;

}

bool FusionTask::start(ImuTask &imu, BarometerTask &barometer)
{
    imu_ = &imu;
    barometer_ = &barometer;

    imu_->registerListener(this);
    barometer_->registerListener(this);

    return startTask("fusion", 8192, 7);
}

void FusionTask::onBarometerSampleReady()
{
    TaskHandle_t h = taskHandle();
    if (h)
    {
        xTaskNotify(h, EVT_BAROMETER_SAMPLE_READY, eSetBits);
    }
}

void FusionTask::onImuSampleReady()
{
    TaskHandle_t h = taskHandle();
    if (h)
    {
        xTaskNotify(h, EVT_IMU_SAMPLE_READY, eSetBits);
    }
}

void FusionTask::run()
{
    configASSERT(imu_ != nullptr);
    configASSERT(barometer_ != nullptr);
    ESP_LOGI(TAG, "Fusion task starting");

    while (!stopRequested())
    {

        uint32_t events = 0;
        xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY);

        const ImuSample imu = imu_->latestSample();
        const BarometerSample barometer = barometer_->latestSample();

        const FusionSample out = fuse(imu, barometer);
        
        latestFusionSample_.write(out);

        if (events & EVT_BAROMETER_SAMPLE_READY)
        {
            ImuSample imu = imu_->latestSample();
            BarometerSample barometer = barometer_->latestSample();

            FusionSample out = fuse(imu, barometer);
            latestFusionSample_.write(out);
        }

        if (events & EVT_IMU_SAMPLE_READY)
        {
            ImuSample imu = imu_->latestSample();
            BarometerSample barometer = barometer_->latestSample();

            FusionSample out = fuse(imu, barometer);
            latestFusionSample_.write(out);

            if (fusionSampleListener_)
            {
                fusionSampleListener_->onFusionSampleReady();
            }
        }

        // vTaskDelay(pdMS_TO_TICKS(50));
    }
}

FusionSample FusionTask::fuse(const ImuSample &imu, const BarometerSample &barometer)
{
    FusionSample out{};
    out.imu = imu;
    out.barometer = barometer;
    const float altitude_m = pressureBarToAltitudeMeters(barometer.pressure, 1013.25f);

    out.t_us = (imu.t_us > barometer.t_us) ? imu.t_us : barometer.t_us;
    out.vertical_speed_mps = 9.8f;
    out.altitude_m = altitude_m;


    // if (!std::isfinite(Barometer.pressure_pa) || Barometer.pressure_pa <= 0.0f)
    // {
    //     return out;
    // }

    // //out.altitude_m = std::isfinite(altitude_m) ? altitude_m : 0.0f;
    // out.altitude_m = 1000.0f;
    // out.vertical_speed_mps = 9.0f;

    // if (!have_alt_)
    // {
    //     have_alt_ = true;
//        last_alt_m_ = out.altitude_m;
//        last_t_us_ = out.t_us;
        // out.vertical_speed_mps = 0.0f;
        // return out;
//    }

  //  const int64_t dt_us = out.t_us - last_t_us_;
  //  if (dt_us <= 0)
  //  {
  //      out.vertical_speed_mps = 0.0f;
  //      return out;
  //  }

  //  const float dt_s = static_cast<float>(dt_us) * 1e-6f;
  //  out.vertical_speed_mps = (out.altitude_m - last_alt_m_) / dt_s;

//    last_alt_m_ = out.altitude_m;
//    last_t_us_ = out.t_us;
    return out;
}
