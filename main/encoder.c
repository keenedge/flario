#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "encoder.h"
#include "telemetry.hpp"

#define ENC_A_GPIO GPIO_NUM_41
#define ENC_B_GPIO GPIO_NUM_42
#define BTN_GPIO   GPIO_NUM_40

#define COUNTS_PER_DETENT 4
#define ENCODER_TASK_PERIOD_MS 10
#define BUTTON_DEBOUNCE_US 20000
#define DISPLAY_PAGE_COUNT 3
#define BARO_PAGE_INDEX 0
#define QNH_STEP_HPA 0.1f

static const char *TAG = "ENCODER";

static pcnt_unit_handle_t s_unit = NULL;
static pcnt_channel_handle_t s_chan_a = NULL;
static pcnt_channel_handle_t s_chan_b = NULL;
static portMUX_TYPE s_encoder_lock = portMUX_INITIALIZER_UNLOCKED;
static int32_t s_position = 0;
static bool s_button_pressed = false;
static uint32_t s_button_press_count = 0;
static bool s_started = false;

static void encoder_task(void *arg)
{
    (void)arg;

    bool last_btn_level = gpio_get_level(BTN_GPIO);
    int64_t last_btn_change_us = esp_timer_get_time();

    while (true) {
        int raw_count = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(s_unit, &raw_count));

        int detent_delta = raw_count / COUNTS_PER_DETENT;
        if (detent_delta != 0) {
            int32_t position = 0;
            uint32_t button_press_count = 0;

            portENTER_CRITICAL(&s_encoder_lock);
            button_press_count = s_button_press_count;
            if ((button_press_count % DISPLAY_PAGE_COUNT) == BARO_PAGE_INDEX) {
                s_position += detent_delta;
            }
            position = s_position;
            portEXIT_CRITICAL(&s_encoder_lock);

            ESP_ERROR_CHECK(pcnt_unit_clear_count(s_unit));
            if ((button_press_count % DISPLAY_PAGE_COUNT) == BARO_PAGE_INDEX) {
                telemetry_adjust_qnh_hpa(detent_delta * QNH_STEP_HPA);
                ESP_LOGI(TAG, "qnh step=%d pos=%ld", detent_delta, (long) position);
            }
        }

        bool btn_level = gpio_get_level(BTN_GPIO);
        int64_t now_us = esp_timer_get_time();
        if ((btn_level != last_btn_level) && ((now_us - last_btn_change_us) >= BUTTON_DEBOUNCE_US)) {
            last_btn_level = btn_level;
            last_btn_change_us = now_us;

            portENTER_CRITICAL(&s_encoder_lock);
            s_button_pressed = !btn_level;
            if (!btn_level) {
                s_button_press_count++;
            }
            portEXIT_CRITICAL(&s_encoder_lock);

            ESP_LOGI(TAG, "button %s", btn_level ? "released" : "pressed");
        }

        vTaskDelay(pdMS_TO_TICKS(ENCODER_TASK_PERIOD_MS));
    }
}

esp_err_t encoder_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    const gpio_config_t encoder_pin_config = {
        .pin_bit_mask = (1ULL << ENC_A_GPIO) | (1ULL << ENC_B_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&encoder_pin_config), TAG, "configure encoder pins failed");

    const pcnt_unit_config_t unit_config = {
        .low_limit = -32768,
        .high_limit = 32767,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, &s_unit), TAG, "pcnt_new_unit failed");

    const pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENC_A_GPIO,
        .level_gpio_num = ENC_B_GPIO,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_unit, &chan_a_config, &s_chan_a), TAG, "pcnt_new_channel A failed");

    const pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENC_B_GPIO,
        .level_gpio_num = ENC_A_GPIO,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_unit, &chan_b_config, &s_chan_b), TAG, "pcnt_new_channel B failed");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            s_chan_a,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE),
        TAG,
        "set edge action A failed");
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            s_chan_a,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG,
        "set level action A failed");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            s_chan_b,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        TAG,
        "set edge action B failed");
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            s_chan_b,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG,
        "set level action B failed");

    const pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(s_unit, &filter_config), TAG, "set glitch filter failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_unit), TAG, "pcnt_unit_enable failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_unit), TAG, "pcnt_unit_clear_count failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(s_unit), TAG, "pcnt_unit_start failed");

    const gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&button_config), TAG, "configure button pin failed");

    const BaseType_t task_ok = xTaskCreate(
        encoder_task,
        "encoder_task",
        3072,
        NULL,
        4,
        NULL);

    if (task_ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    ESP_LOGI(TAG, "started A=%d B=%d BTN=%d", ENC_A_GPIO, ENC_B_GPIO, BTN_GPIO);
    return ESP_OK;
}

int32_t encoder_get_position(void)
{
    int32_t position = 0;

    portENTER_CRITICAL(&s_encoder_lock);
    position = s_position;
    portEXIT_CRITICAL(&s_encoder_lock);

    return position;
}

uint8_t encoder_get_page(uint8_t page_count)
{
    if (page_count == 0) {
        return 0;
    }

    int32_t page = encoder_get_position() % page_count;
    if (page < 0) {
        page += page_count;
    }

    return (uint8_t) page;
}

bool encoder_is_button_pressed(void)
{
    bool button_pressed = false;

    portENTER_CRITICAL(&s_encoder_lock);
    button_pressed = s_button_pressed;
    portEXIT_CRITICAL(&s_encoder_lock);

    return button_pressed;
}

uint32_t encoder_get_button_press_count(void)
{
    uint32_t press_count = 0;

    portENTER_CRITICAL(&s_encoder_lock);
    press_count = s_button_press_count;
    portEXIT_CRITICAL(&s_encoder_lock);

    return press_count;
}
