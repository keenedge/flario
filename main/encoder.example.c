#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#define ENC_A_GPIO 41
#define ENC_B_GPIO 42
#define BTN_GPIO   40

static const char *TAG = "ENCODER";

// From your earlier measurements, 4 raw quadrature counts per detent
#define COUNTS_PER_DETENT 4

void app_main(void)
{
    pcnt_unit_handle_t unit = NULL;
    pcnt_channel_handle_t chan_a = NULL;
    pcnt_channel_handle_t chan_b = NULL;

    // ---------------- PCNT UNIT ----------------

    pcnt_unit_config_t unit_config = {
        .low_limit = -32768,
        .high_limit = 32767,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENC_A_GPIO,
        .level_gpio_num = ENC_B_GPIO,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_a_config, &chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENC_B_GPIO,
        .level_gpio_num = ENC_A_GPIO,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_b_config, &chan_b));

    // Quadrature decode
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE));

    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));

    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Glitch filter must be configured before enable/start.
    // Keep this small; PCNT glitch filter is for short glitches, not full button-style debounce.
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_config));

    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));

    // ---------------- BUTTON ----------------

    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn_config));

    ESP_LOGI(TAG, "PCNT encoder started");
    ESP_LOGI(TAG, "A=%d B=%d BTN=%d", ENC_A_GPIO, ENC_B_GPIO, BTN_GPIO);

    int last_raw = 0;
    int last_detent = 0;

    bool last_btn_state = true;   // pull-up: true = released
    int64_t last_btn_change_us = 0;

    while (1) {
        // -------- ENCODER --------

        int raw = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &raw));

        int detent = raw / COUNTS_PER_DETENT;

        // Only print when the detent changes
        if (detent != last_detent) {
            ESP_LOGI(TAG, "raw=%d detent=%d", raw, detent);
            last_detent = detent;
        }

        last_raw = raw;

        // -------- BUTTON --------

        bool btn_state = gpio_get_level(BTN_GPIO);   // HIGH=released, LOW=pressed
        int64_t now_us = esp_timer_get_time();

        if (btn_state != last_btn_state) {
            // 20 ms debounce
            if ((now_us - last_btn_change_us) >= 20000) {
                last_btn_change_us = now_us;
                last_btn_state = btn_state;

                if (!btn_state) {
                    ESP_LOGI(TAG, "BUTTON pressed");
                } else {
                    ESP_LOGI(TAG, "BUTTON released");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}