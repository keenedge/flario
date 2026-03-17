#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    uint32_t sample_count;
    int64_t updated_us;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    float gyro_x_rads;
    float gyro_y_rads;
    float gyro_z_rads;

    float accel_x_ms2;
    float accel_y_ms2;
    float accel_z_ms2;

    float lin_accel_x_ms2;
    float lin_accel_y_ms2;
    float lin_accel_z_ms2;
} telemetry_imu_t;

typedef struct {
    bool valid;
    uint32_t sample_count;
    int64_t updated_us;

    int32_t pressure_pa;
    float pressure_hpa;
    float temp_c;
    float altitude_m;
} telemetry_baro_t;

typedef struct {
    uint32_t revision;
    telemetry_imu_t imu;
    telemetry_baro_t baro;
} telemetry_snapshot_t;

void telemetry_store_imu_euler(float roll_deg, float pitch_deg, float yaw_deg);
void telemetry_store_imu_gyro(float x_rads, float y_rads, float z_rads);
void telemetry_store_imu_accel(float x_ms2, float y_ms2, float z_ms2);
void telemetry_store_imu_linear_accel(float x_ms2, float y_ms2, float z_ms2);
void telemetry_store_baro(int32_t pressure_pa, float temp_c);
void telemetry_load_snapshot(telemetry_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif
