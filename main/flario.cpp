#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
//#include "report_frequency_diag.hpp"
#include "latest.hpp"
#include "samples.hpp"

#include <stdio.h>
#include <array>
#include <atomic>
#include "BNO08x.hpp"

// #include "fusion_task.hpp"
// #include "ui_task.hpp"
#include "imu_task.hpp"
// #include "barometer_task.hpp"

// #define PIEZO_A GPIO_NUM_47
// #define PIEZO_B GPIO_NUM_48

// #define LEDC_TIMER      LEDC_TIMER_0
// #define LEDC_MODE       LEDC_LOW_SPEED_MODE
// #define LEDC_CHANNEL_A  LEDC_CHANNEL_0
// #define LEDC_CHANNEL_B  LEDC_CHANNEL_1

// #define DUTY_RES        LEDC_TIMER_10_BIT
// #define DUTY_MAX        (1 << DUTY_RES)

void initGPIO(void);
//  void initSPI( void );
//  void piezo_init( void );


enum ReportNotifyBits : uint32_t
{
    NOTIFY_RV_GAME = (1UL << 0),
    NOTIFY_CAL_GYRO = (1UL << 1),
    NOTIFY_GRAVITY = (1UL << 2),
    NOTIFY_ACCEL = (1UL << 3),
    NOTIFY_LIN_ACCEL = (1UL << 4),
};


static const constexpr char *TAG = "Main";
static TaskHandle_t handle;
static BNO08x imu;

static Latest<RvGameSample> latest_rv_game;
static Latest<CalGyroSample> latest_cal_gyro;
static Latest<GravitySample> latest_gravity;
static Latest<AccelSample> latest_accel;
static Latest<LinearAccelSample> latest_linear_accel;

// static int sampleCount;
// static int totalCount;

// static std::atomic<uint32_t> cb_count_rv_game{0};
// static std::atomic<uint32_t> cb_count_cal_gyro{0};
// static std::atomic<uint32_t> cb_count_gravity{0};
// static std::atomic<uint32_t> cb_count_accel{0};
// static std::atomic<uint32_t> cb_count_lin_accel{0};

// static constexpr size_t REPORT_COUNT = 5;

static constexpr uint32_t RV_GAME_INTERVAL_US =  10000UL;
static constexpr uint32_t CAL_GYRO_INTERVAL_US = 10000UL;
static constexpr uint32_t GRAVITY_INTERVAL_US =  10000UL;
static constexpr uint32_t ACCEL_INTERVAL_US =    10000UL;
static constexpr uint32_t LIN_ACCEL_INTERVAL_US =10000UL;

// static constexpr float hz_from_interval_us(uint32_t interval_us)
// {
//     return 1000000.0f / static_cast<float>(interval_us);
// }

// using ReportFreqDiag = ReportFrequencyDiag<REPORT_COUNT>;
// using ReportFreqResult = ReportFreqDiag::Result;

// static const std::array<ReportFreqDiag::Config, REPORT_COUNT> report_freq_configs = {
//     ReportFreqDiag::Config{"rv_game", &cb_count_rv_game, hz_from_interval_us(RV_GAME_INTERVAL_US), 8.0f},
//     ReportFreqDiag::Config{"cal_gyro", &cb_count_cal_gyro, hz_from_interval_us(CAL_GYRO_INTERVAL_US), 8.0f},
//     ReportFreqDiag::Config{"gravity", &cb_count_gravity, hz_from_interval_us(GRAVITY_INTERVAL_US), 8.0f},
//     ReportFreqDiag::Config{"accel", &cb_count_accel, hz_from_interval_us(ACCEL_INTERVAL_US), 8.0f},
//     ReportFreqDiag::Config{"linear_accel", &cb_count_lin_accel, hz_from_interval_us(LIN_ACCEL_INTERVAL_US), 8.0f},
// };
// static ReportFreqDiag report_freq_diag(report_freq_configs);

// enum ReportDiagIndex : size_t
// {
//     RV_GAME_IDX = 0,
//     CAL_GYRO_IDX = 1,
//     GRAVITY_IDX = 2,
//     ACCEL_IDX = 3,
//     LIN_ACCEL_IDX = 4
// };

// void initReportFrequencyDiag(void)
// {
//     report_freq_diag.reset();
// }

// ReportFreqResult checkReportFrequencyDiag(void)
// {
//     return report_freq_diag.update();
// }

ImuSample readLatestImuSample(void)
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

// void printStatusPanel(uint32_t report_bits, bool timed_out, const ImuSample &sample, const ReportFreqResult &freq)
// {
//     static bool panel_drawn = false;
//     static constexpr int PANEL_LINES = 3;

//     if (panel_drawn)
//     {
//         printf("\x1b[%dA", PANEL_LINES);
//     }

//     const char rv_mark = freq.below_threshold[RV_GAME_IDX] ? '!' : ' ';
//     const char gyro_mark = freq.below_threshold[CAL_GYRO_IDX] ? '!' : ' ';
//     const char gravity_mark = freq.below_threshold[GRAVITY_IDX] ? '!' : ' ';
//     const char accel_mark = freq.below_threshold[ACCEL_IDX] ? '!' : ' ';
//     const char lin_accel_mark = freq.below_threshold[LIN_ACCEL_IDX] ? '!' : ' ';

//     printf("\r\x1b[2K[Main] bits=0x%08lx timeout=%1d sample=%6d total=%8d\n",
//            static_cast<unsigned long>(report_bits),
//            timed_out ? 1 : 0,
//            sampleCount,
//            totalCount);

//     printf("\r\x1b[2K[Freq5s] rv=%7.2f/%7.2f%c gyro=%7.2f/%7.2f%c gravity=%7.2f/%7.2f%c accel=%7.2f/%7.2f%c lin=%7.2f/%7.2f%c\n",
//            freq.measured_hz[RV_GAME_IDX], report_freq_diag.expected_hz(RV_GAME_IDX), rv_mark,
//            freq.measured_hz[CAL_GYRO_IDX], report_freq_diag.expected_hz(CAL_GYRO_IDX), gyro_mark,
//            freq.measured_hz[GRAVITY_IDX], report_freq_diag.expected_hz(GRAVITY_IDX), gravity_mark,
//            freq.measured_hz[ACCEL_IDX], report_freq_diag.expected_hz(ACCEL_IDX), accel_mark,
//            freq.measured_hz[LIN_ACCEL_IDX], report_freq_diag.expected_hz(LIN_ACCEL_IDX), lin_accel_mark);

//     char sample_line[320];
//     int pos = 0;
//     pos += snprintf(sample_line + pos, sizeof(sample_line) - static_cast<size_t>(pos), "[Sample]");

//     if (sample.has_euler)
//     {
//         pos += snprintf(sample_line + pos, sizeof(sample_line) - static_cast<size_t>(pos),
//                         " euler=(%8.2f,%8.2f,%8.2f)", sample.euler.x, sample.euler.y, sample.euler.z);
//     }
//     if (sample.has_gyro)
//     {
//         pos += snprintf(sample_line + pos, sizeof(sample_line) - static_cast<size_t>(pos),
//                         " gyro=(%8.2f,%8.2f,%8.2f)", sample.gyro.x, sample.gyro.y, sample.gyro.z);
//     }
//     if (sample.has_accel)
//     {
//         pos += snprintf(sample_line + pos, sizeof(sample_line) - static_cast<size_t>(pos),
//                         " accel=(%8.2f,%8.2f,%8.2f)", sample.accel.x, sample.accel.y, sample.accel.z);
//     }
//     if (sample.has_linear_accel)
//     {
//         pos += snprintf(sample_line + pos, sizeof(sample_line) - static_cast<size_t>(pos),
//                         " lin=(%8.2f,%8.2f,%8.2f)", sample.linear_accel.x, sample.linear_accel.y, sample.linear_accel.z);
//     }
//     if (sample.has_gravity)
//     {
//         pos += snprintf(sample_line + pos, sizeof(sample_line) - static_cast<size_t>(pos),
//                         " grav=(%8.2f,%8.2f,%8.2f)", sample.gravity.x, sample.gravity.y, sample.gravity.z);
//     }
//     if (pos == static_cast<int>(sizeof("[Sample]") - 1))
//     {
//         pos += snprintf(sample_line + pos, sizeof(sample_line) - static_cast<size_t>(pos), " none");
//     }

//     printf("\r\x1b[2K%s\n", sample_line);

//     fflush(stdout);
//     panel_drawn = true;
// }

void printStatusPanel(uint32_t report_bits, bool timed_out, const ImuSample &sample )
{
    static bool panel_drawn = false;
    static constexpr int PANEL_LINES = 3;

    if (panel_drawn)
    {
        printf("\x1b[%dA", PANEL_LINES);
    }

    printf("\r\x1b[2K[Main] bits=0x%08lx timeout=%1d",
           static_cast<unsigned long>(report_bits),
           timed_out ? 1 : 0
           );

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

    printf("\r\x1b[2K%s\n", sample_line);

    fflush(stdout);
    panel_drawn = true;
}

void initGPIO(void)
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
}

void onHostInterrupt_rv_game(void)
{
    //cb_count_rv_game.fetch_add(1, std::memory_order_relaxed);
    if (handle != nullptr)
        xTaskNotify(handle, NOTIFY_RV_GAME, eSetBits);
}

void onHostInterrupt_cal_gyro(void)
{
    //cb_count_cal_gyro.fetch_add(1, std::memory_order_relaxed);
    if (handle != nullptr)
        xTaskNotify(handle, NOTIFY_CAL_GYRO, eSetBits);
}

void onHostInterrupt_gravity(void)
{
    //cb_count_gravity.fetch_add(1, std::memory_order_relaxed);
    if (handle != nullptr)
        xTaskNotify(handle, NOTIFY_GRAVITY, eSetBits);
}

void onHostInterrupt_acceleration(void)
{
    //cb_count_accel.fetch_add(1, std::memory_order_relaxed);
    if (handle != nullptr)
        xTaskNotify(handle, NOTIFY_ACCEL, eSetBits);
}

void onHostInterrupt_linear_acceleration(void)
{
    //cb_count_lin_accel.fetch_add(1, std::memory_order_relaxed);
    if (handle != nullptr)
        xTaskNotify(handle, NOTIFY_LIN_ACCEL, eSetBits);
}

size_t drainSamples(uint32_t notify_bits)
{
    //sampleCount = 0;
    size_t emitted = 0;
    const TickType_t start = xTaskGetTickCount();
    TickType_t budget = pdMS_TO_TICKS(2); // prevent monopolizing CPU
    if (budget == 0) budget = 1;   // at 100 Hz tick, 2 ms -> 0 ticks

    while ((xTaskGetTickCount() - start) < budget)
    {
        bool any = false;

        if ((notify_bits & NOTIFY_RV_GAME) && imu.rpt.rv_game.has_new_data())
        {
            RvGameSample sample{};
            sample.t_us = esp_timer_get_time();
            sample.value = imu.rpt.rv_game.get_euler();
            sample.has = true;
            latest_rv_game.write(sample);
            any = true;
            // sampleCount++;
            // totalCount++;
            emitted++;
        }

        if ((notify_bits & NOTIFY_CAL_GYRO) && imu.rpt.cal_gyro.has_new_data())
        {
            CalGyroSample sample{};
            sample.t_us = esp_timer_get_time();
            sample.value = imu.rpt.cal_gyro.get();
            sample.has = true;
            latest_cal_gyro.write(sample);
            any = true;
            // sampleCount++;
            // totalCount++;
            emitted++;
        }

        if ((notify_bits & NOTIFY_GRAVITY) && imu.rpt.gravity.has_new_data())
        {
            GravitySample sample{};
            sample.t_us = esp_timer_get_time();
            sample.value = imu.rpt.gravity.get();
            sample.has = true;
            latest_gravity.write(sample);
            any = true;
            // sampleCount++;
            // totalCount++;
            emitted++;
        }

        if ((notify_bits & NOTIFY_ACCEL) && imu.rpt.accelerometer.has_new_data())
        {
            AccelSample sample{};
            sample.t_us = esp_timer_get_time();
            sample.value = imu.rpt.accelerometer.get();
            sample.has = true;
            latest_accel.write(sample);
            any = true;
            // sampleCount++;
            // totalCount++;
            emitted++;
        }

        if ((notify_bits & NOTIFY_LIN_ACCEL) && imu.rpt.linear_accelerometer.has_new_data())
        {
            LinearAccelSample sample{};
            sample.t_us = esp_timer_get_time();
            sample.value = imu.rpt.linear_accelerometer.get();
            sample.has = true;
            latest_linear_accel.write(sample);
            any = true;
            // sampleCount++;
            // totalCount++;
            emitted++;
        }
        if (!any)
        {
            break;
        }
    }

    return emitted;
}

// void tiny_task(void *arg)
// {
//     ESP_LOGI(TAG, "tiny_task started");
//     ESP_LOGI(TAG, "Starting imu.initialize()");
//     if (!imu.initialize())
//     {
//         ESP_LOGE(TAG, "Init failure, returning from main.");
//         return;
//     }
//     ESP_LOGI(TAG, "imu.initialize() complete");

//     imu.rpt.rv_game.enable(RV_GAME_INTERVAL_US); // 100,000us == 100ms report interval
//     imu.rpt.cal_gyro.enable(CAL_GYRO_INTERVAL_US);
//     imu.rpt.gravity.enable(GRAVITY_INTERVAL_US);
//     imu.rpt.accelerometer.enable(ACCEL_INTERVAL_US);
//     imu.rpt.linear_accelerometer.enable(LIN_ACCEL_INTERVAL_US);

//     bool cb_ok = true;
//     cb_ok &= imu.rpt.rv_game.register_cb(onHostInterrupt_rv_game);
//     cb_ok &= imu.rpt.cal_gyro.register_cb(onHostInterrupt_cal_gyro);
//     cb_ok &= imu.rpt.gravity.register_cb(onHostInterrupt_gravity);
//     cb_ok &= imu.rpt.accelerometer.register_cb(onHostInterrupt_acceleration);
//     cb_ok &= imu.rpt.linear_accelerometer.register_cb(onHostInterrupt_linear_acceleration);

//     if (!cb_ok)
//     {
//         ESP_LOGE(TAG, "Frequency diag: failed to register one or more report callbacks.");
//     }

// //    initReportFrequencyDiag();
// //    static const int64_t PANEL_REFRESH_US = 200000LL; // 5Hz panel update
// //    int64_t last_panel_us = 0;

//     for (;;)
//     {
//         uint32_t report_bits = 0U;
//         BaseType_t woke = xTaskNotifyWait(0UL, 0xFFFFFFFFUL, &report_bits, pdMS_TO_TICKS(100));

//         // Try to drain pending report flags a few times, then yield.
//         for (int pass = 0; pass < 4; ++pass)
//         {
//             size_t drained = drainSamples(report_bits);
//             if (drained == 0)
//             {
//                 break; // nothing pending now
//             }
//         }

//         ImuSample sample = readLatestImuSample();
//   //      ReportFreqResult freq = checkReportFrequencyDiag();
//   //      const int64_t now_us = esp_timer_get_time();
//   //      if ((last_panel_us == 0) || ((now_us - last_panel_us) >= PANEL_REFRESH_US))
//         {
//             printStatusPanel(report_bits, (woke != pdTRUE), sample );
//   //          last_panel_us = now_us;
//         }

//         vTaskDelay(pdMS_TO_TICKS(1));
//     }
// }

extern "C" void app_main(void)
{
    initGPIO();
    
    ESP_LOGI(TAG, "app_main creating tiny_task");

    // const BaseType_t created = xTaskCreate(
    //     imu_task, // task function
    //     "tiny",    // name
    //     8192,      // stack size (bytes in ESP-IDF)
    //     NULL,      // parameter
    //     1,         // priority
    //     &handle    // task handle
    // );

    // if (created != pdPASS)
    // {
    //     ESP_LOGE(TAG, "xTaskCreate failed, heap=%lu", static_cast<unsigned long>(esp_get_free_heap_size()));
    //     return;
    // }



// //    initSPI();
//     piezo_init();

     static ImuTask imuTask;
//     static BarometerTask barometerTask;
//     static FusionTask fusionTask;
//     static UiTask uiTask;

     configASSERT(imuTask.start());
     vTaskDelay(pdMS_TO_TICKS(500));
//     configASSERT(barometerTask.start());
//     configASSERT(fusionTask.start(imuTask, barometerTask));
//     configASSERT(uiTask.start(fusionTask));

//     ESP_LOGI(TAG, "Start Main Task Loop");
//     int count = 0;
     while (true)
     {
//         // for (int freq = 1000; freq <= 5000; freq += 50) {

//         //    ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq);

//         //     vTaskDelay(pdMS_TO_TICKS(40));
//         // }
//         ESP_LOGI( TAG, "Main Loop: %d", count++);
         vTaskDelay(pdMS_TO_TICKS(5000));

     }
}

// void initGPIO( void ) {
//     gpio_config_t boot_gpio = {};
//     boot_gpio.pin_bit_mask = (1ULL << GPIO_NUM_3) | (1ULL << GPIO_NUM_4);
//     boot_gpio.mode = GPIO_MODE_OUTPUT;
//     boot_gpio.pull_up_en = GPIO_PULLUP_DISABLE;
//     boot_gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
//     boot_gpio.intr_type = GPIO_INTR_DISABLE;
//     ESP_ERROR_CHECK(gpio_config(&boot_gpio));
//     ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_3, 1));
//     ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_4, 1));
// }

// void initSPI( void ) {
//     bno08x_config_t imu_cfg = bno08x_config_t();

//     spi_bus_config_t bus_cfg = {};
//     bus_cfg.sclk_io_num = imu_cfg.io_sclk;
//     bus_cfg.mosi_io_num = imu_cfg.io_mosi;
//     bus_cfg.miso_io_num = imu_cfg.io_miso;
//     bus_cfg.quadwp_io_num = -1;
//     bus_cfg.quadhd_io_num = -1;
//     bus_cfg.max_transfer_sz = 0;
//     bus_cfg.flags = SPICOMMON_BUSFLAG_MASTER;

//     esp_err_t ret = spi_bus_initialize(imu_cfg.spi_peripheral, &bus_cfg, SPI_DMA_CH_AUTO);
//     if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
//     {
//         ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
//     }

// }

// void piezo_init(void)
// {
//     ledc_timer_config_t timer = {
//         .speed_mode = LEDC_MODE,
//         .duty_resolution = DUTY_RES,
//         .timer_num = LEDC_TIMER,
//         .freq_hz = 2000,
//         .clk_cfg = LEDC_AUTO_CLK,
//                                             };

//     ledc_timer_config(&timer);

//     // Channel A
//     ledc_channel_config_t chA = {
//         .gpio_num = PIEZO_A,
//         .speed_mode = LEDC_MODE,
//         .channel = LEDC_CHANNEL_A,
//         .timer_sel = LEDC_TIMER,
//         .duty = DUTY_MAX / 2,   // 50%
//         .hpoint = 0
//     };
//     ledc_channel_config(&chA);

//     // Channel B (phase shifted 180°)
//     ledc_channel_config_t chB = {
//         .gpio_num = PIEZO_B,
//         .speed_mode = LEDC_MODE,
//         .channel = LEDC_CHANNEL_B,
//         .timer_sel = LEDC_TIMER,
//         .duty = DUTY_MAX / 2,
//         .hpoint = DUTY_MAX / 2
//     };
//     ledc_channel_config(&chB);
//}
