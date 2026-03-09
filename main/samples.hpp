#pragma once

#include <stdint.h>
#include "BNO08xGlobalTypes.hpp"

template <typename T>
struct TimestampedSample
{
    int64_t t_us = 0;
    bool has = false;
    T value{};
};

using RvGameSample = TimestampedSample<bno08x_euler_angle_t>;
using CalGyroSample = TimestampedSample<bno08x_gyro_t>;
using GravitySample = TimestampedSample<bno08x_accel_t>;
using AccelSample = TimestampedSample<bno08x_accel_t>;
using LinearAccelSample = TimestampedSample<bno08x_accel_t>;

struct ImuSample
{
    int64_t t_us = 0;
    bool has_euler = false;
    bool has_gyro = false;
    bool has_gravity = false;
    bool has_accel = false;
    bool has_linear_accel = false;
    bno08x_euler_angle_t euler{};
    bno08x_gyro_t gyro{};
    bno08x_accel_t gravity{};
    bno08x_accel_t accel{};
    bno08x_accel_t linear_accel{};
};

struct BarometerSample
{
    int64_t t_us = 0;
    float pressure = 0.0f;
    float temperature = 0.0f;
};

struct FusionSample
{
    float altitude_m = 0.0f;
    float vertical_speed_mps = 0.0f;
    ImuSample imu{};
    BarometerSample barometer{};
    int64_t t_us = 0;
};
