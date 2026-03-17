#include <math.h>

#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

#include "telemetry.hpp"

namespace {

portMUX_TYPE s_telemetry_lock = portMUX_INITIALIZER_UNLOCKED;
telemetry_snapshot_t s_snapshot = {};

void telemetry_touch_imu(int64_t now_us)
{
    s_snapshot.imu.valid = true;
    s_snapshot.imu.updated_us = now_us;
    s_snapshot.imu.sample_count++;
    s_snapshot.revision++;
}

void telemetry_touch_baro(int64_t now_us)
{
    s_snapshot.baro.valid = true;
    s_snapshot.baro.updated_us = now_us;
    s_snapshot.baro.sample_count++;
    s_snapshot.revision++;
}

} // namespace

void telemetry_store_imu_euler(float roll_deg, float pitch_deg, float yaw_deg)
{
    const int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL(&s_telemetry_lock);
    telemetry_touch_imu(now_us);
    s_snapshot.imu.roll_deg = roll_deg;
    s_snapshot.imu.pitch_deg = pitch_deg;
    s_snapshot.imu.yaw_deg = yaw_deg;
    portEXIT_CRITICAL(&s_telemetry_lock);
}

void telemetry_store_imu_gyro(float x_rads, float y_rads, float z_rads)
{
    const int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL(&s_telemetry_lock);
    telemetry_touch_imu(now_us);
    s_snapshot.imu.gyro_x_rads = x_rads;
    s_snapshot.imu.gyro_y_rads = y_rads;
    s_snapshot.imu.gyro_z_rads = z_rads;
    portEXIT_CRITICAL(&s_telemetry_lock);
}

void telemetry_store_imu_accel(float x_ms2, float y_ms2, float z_ms2)
{
    const int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL(&s_telemetry_lock);
    telemetry_touch_imu(now_us);
    s_snapshot.imu.accel_x_ms2 = x_ms2;
    s_snapshot.imu.accel_y_ms2 = y_ms2;
    s_snapshot.imu.accel_z_ms2 = z_ms2;
    portEXIT_CRITICAL(&s_telemetry_lock);
}

void telemetry_store_imu_linear_accel(float x_ms2, float y_ms2, float z_ms2)
{
    const int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL(&s_telemetry_lock);
    telemetry_touch_imu(now_us);
    s_snapshot.imu.lin_accel_x_ms2 = x_ms2;
    s_snapshot.imu.lin_accel_y_ms2 = y_ms2;
    s_snapshot.imu.lin_accel_z_ms2 = z_ms2;
    portEXIT_CRITICAL(&s_telemetry_lock);
}

void telemetry_store_baro(int32_t pressure_pa, float temp_c)
{
    const int64_t now_us = esp_timer_get_time();
    const float pressure_hpa = pressure_pa / 100.0f;
    float altitude_m = 0.0f;

    if (pressure_hpa > 0.0f) {
        altitude_m = 44330.0f * (1.0f - powf(pressure_hpa / 1013.25f, 0.19029495f));
    }

    portENTER_CRITICAL(&s_telemetry_lock);
    telemetry_touch_baro(now_us);
    s_snapshot.baro.pressure_pa = pressure_pa;
    s_snapshot.baro.pressure_hpa = pressure_hpa;
    s_snapshot.baro.temp_c = temp_c;
    s_snapshot.baro.altitude_m = altitude_m;
    portEXIT_CRITICAL(&s_telemetry_lock);
}

void telemetry_load_snapshot(telemetry_snapshot_t *snapshot)
{
    if (snapshot == nullptr) {
        return;
    }

    portENTER_CRITICAL(&s_telemetry_lock);
    *snapshot = s_snapshot;
    portEXIT_CRITICAL(&s_telemetry_lock);
}
