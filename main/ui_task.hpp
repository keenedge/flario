#pragma once

#include "task_base.hpp"
#include "fusion_task.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "ssd1306.h"

#include "fusion_sample_listener.hpp"

class UiTask : 
    public TaskBase,
    public FusionSampleListener
{
public:
    bool start(FusionTask &fusion);
    void onFusionSampleReady() override;
    
protected:
    void run() override;

private:
    static void DrawText(ssd1306_handle_t oled, uint8_t line, const char *fmt, ...);

    void render(const FusionSample &output);

    // hardware handles
    i2c_master_bus_handle_t bus_ = nullptr;
    ssd1306_handle_t oled_ = nullptr;
    FusionTask *fusion_ = nullptr;

    static constexpr uint32_t EVT_FUSION_SAMPLE_READY = 1u << 0;

    FusionSampleListener* fusionSampleListener = nullptr;

    // UI update timing
    TickType_t last_io_tick_ = 0;

    void init_display();
};
