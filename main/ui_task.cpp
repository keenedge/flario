// UiTask.cpp
#include "ui_task.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "ssd1306.h"

static const char* TAG = "UiTask";

#define OLED_I2C_PORT I2C_NUM_0
#define OLED_SDA_GPIO 17
#define OLED_SCL_GPIO 18
#define OLED_RST_GPIO 21
#define VEXT_CTRL_GPIO 36

#define VEXT_ON_LEVEL 0

void UiTask::DrawText(ssd1306_handle_t oled, uint8_t line, const char* fmt, ...)
{
    char buffer[24];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    const esp_err_t err = ssd1306_display_text(oled, line, buffer, false);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "display_text failed: %s", esp_err_to_name(err));
    }
}

static void vext_on()
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << VEXT_CTRL_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(static_cast<gpio_num_t>(VEXT_CTRL_GPIO), VEXT_ON_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void oled_reset()
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << OLED_RST_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    gpio_set_level(static_cast<gpio_num_t>(OLED_RST_GPIO), 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(static_cast<gpio_num_t>(OLED_RST_GPIO), 1);
    vTaskDelay(pdMS_TO_TICKS(80));
}

static void i2c_bus_clear_gpio()
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << OLED_SDA_GPIO) | (1ULL << OLED_SCL_GPIO),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    gpio_set_level(static_cast<gpio_num_t>(OLED_SDA_GPIO), 1);
    gpio_set_level(static_cast<gpio_num_t>(OLED_SCL_GPIO), 1);
    esp_rom_delay_us(5);

    for (int i = 0; i < 9; i++) {
        gpio_set_level(static_cast<gpio_num_t>(OLED_SCL_GPIO), 0);
        esp_rom_delay_us(5);
        gpio_set_level(static_cast<gpio_num_t>(OLED_SCL_GPIO), 1);
        esp_rom_delay_us(5);
        if (gpio_get_level(static_cast<gpio_num_t>(OLED_SDA_GPIO)) == 1) {
            break;
        }
    }

    gpio_set_level(static_cast<gpio_num_t>(OLED_SDA_GPIO), 0);
    esp_rom_delay_us(5);
    gpio_set_level(static_cast<gpio_num_t>(OLED_SCL_GPIO), 1);
    esp_rom_delay_us(5);
    gpio_set_level(static_cast<gpio_num_t>(OLED_SDA_GPIO), 1);
    esp_rom_delay_us(5);
}

static void i2c_init(i2c_master_bus_handle_t* out_bus)
{
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = static_cast<i2c_port_t>(OLED_I2C_PORT);
    bus_cfg.sda_io_num = static_cast<gpio_num_t>(OLED_SDA_GPIO);
    bus_cfg.scl_io_num = static_cast<gpio_num_t>(OLED_SCL_GPIO);
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.intr_priority = 0;
    bus_cfg.trans_queue_depth = 0;
    bus_cfg.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, out_bus));
}

bool UiTask::start(FusionTask& fusion)
{
    fusion_ = &fusion;

    fusion_->registerListener(this);

    return startTask("ui", 4096, 4);
}

void UiTask::onFusionSampleReady()
{
    TaskHandle_t h = taskHandle();
    if (h)
    {
        xTaskNotify(h, EVT_FUSION_SAMPLE_READY, eSetBits);
    }
}


void UiTask::init_display()
{
    vext_on();
    oled_reset();
    i2c_bus_clear_gpio();
    i2c_init(&bus_);

    const uint8_t address = 0x3C;
    const esp_err_t probe = i2c_master_probe(bus_, address, pdMS_TO_TICKS(200));
    if (probe == ESP_OK) {
        ESP_LOGI(TAG, "I2C ACK at 0x%02X", address);
    } else {
        ESP_LOGW(TAG, "I2C probe 0x%02X -> %s", address, esp_err_to_name(probe));
    }

    ssd1306_config_t cfg = I2C_SSD1306_128x64_CONFIG_DEFAULT;
    ssd1306_init(bus_, &cfg, &oled_);

    if (!oled_) {
        ESP_LOGE(TAG, "ssd1306_init failed (handle NULL)");
        abort();
    }

    ssd1306_clear_display(oled_, false);
    ssd1306_set_contrast(oled_, 0xFF);
    last_io_tick_ = xTaskGetTickCount();
}

void UiTask::run()
{
    configASSERT(fusion_ != nullptr);
    init_display();

    while (!stopRequested()) {
        uint32_t events = 0;
        xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY);

        if (events & EVT_FUSION_SAMPLE_READY) 
        {
            const FusionSample out = fusion_->latestOutput();
            render(out);
        }


        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void UiTask::render(const FusionSample& fusionSample)
{
    const TickType_t now = xTaskGetTickCount();
    if ((now - last_io_tick_) < pdMS_TO_TICKS(200)) {
        return;
    }
    last_io_tick_ = now;

    DrawText(oled_, 0, "Alt :%7.1f m", fusionSample.altitude_m);
    DrawText(oled_, 1, "VSpd:%6.2f m/s", fusionSample.vertical_speed_mps);
    DrawText(oled_, 3, "P   :%7.3f bar", fusionSample.barometer.pressure);
    DrawText(oled_, 2, "Temp:%7.3f c", fusionSample.barometer.temperature);
    DrawText(oled_, 4, "time:%u", last_io_tick_);

    // char line0[24];
    // char line1[24];
    // char line2[24];
    // char line3[24];

    // snprintf(line0, sizeof(line0), "Alt: %7.1f m", fusionSample.altitude_m);
    // snprintf(line1, sizeof(line1), "VSpd:%6.2f m/s", fusionSample.vertical_speed_mps);
    // snprintf(line2, sizeof(line2), "Pressure:  %7.3f", fusionSample.barometer.pressure_pa);
    // snprintf(line3, sizeof(line3), "Tempe:  %7.3f", fusionSample.barometer.temperature_c);

    // const esp_err_t e0 = ssd1306_display_text(oled_, 0, line0, false);
    // const esp_err_t e1 = ssd1306_display_text(oled_, 1, line1, false);
    // const esp_err_t e2 = ssd1306_display_text(oled_, 2, line2, false);
    // const esp_err_t e3 = ssd1306_display_text(oled_, 3, line3, false);

    // if (e0 != ESP_OK || e1 != ESP_OK || e2 != ESP_OK) {
    //     ESP_LOGE(TAG, "display_text failed: %s %s %s",
    //              esp_err_to_name(e0), esp_err_to_name(e1), esp_err_to_name(e2), esp_err_to_name(e3));
    // }
}
