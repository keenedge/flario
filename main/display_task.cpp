#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "display_task.hpp"
#include "telemetry.hpp"

namespace {

constexpr const char *TAG = "DISPLAY";

constexpr i2c_port_num_t I2C_PORT = I2C_NUM_0;
constexpr gpio_num_t PIN_NUM_SDA = GPIO_NUM_17;
constexpr gpio_num_t PIN_NUM_SCL = GPIO_NUM_18;
constexpr gpio_num_t PIN_NUM_RST = GPIO_NUM_21;

constexpr uint32_t OLED_PIXEL_CLOCK_HZ = 400000;
constexpr uint8_t OLED_I2C_ADDR = 0x3C;
constexpr int OLED_WIDTH = 128;
constexpr int OLED_HEIGHT = 64;
constexpr int OLED_PAGE_COUNT = OLED_HEIGHT / 8;
constexpr size_t OLED_BUFFER_SIZE = OLED_WIDTH * OLED_PAGE_COUNT;
constexpr TickType_t DISPLAY_REFRESH_TICKS = pdMS_TO_TICKS(250);
constexpr int DISPLAY_PAGE_PERIOD_S = 4;
constexpr int TEXT_LINE_HEIGHT = 8;
constexpr int TEXT_CHAR_WIDTH = 6;
constexpr int TEXT_MAX_LINES = OLED_HEIGHT / TEXT_LINE_HEIGHT;
constexpr size_t TEXT_LINE_LEN = 22;

typedef struct {
    char ch;
    uint8_t columns[5];
} glyph_t;

static const glyph_t FONT[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x7A}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'W', {0x7F, 0x20, 0x18, 0x20, 0x7F}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x03, 0x04, 0x78, 0x04, 0x03}},
    {'Z', {0x61, 0x51, 0x49, 0x45, 0x43}},
};

static i2c_master_bus_handle_t s_i2c_bus = nullptr;
static esp_lcd_panel_io_handle_t s_io_handle = nullptr;
static esp_lcd_panel_handle_t s_panel = nullptr;
static uint8_t s_oled_buffer[OLED_BUFFER_SIZE];

const uint8_t *lookup_glyph(char ch)
{
    const char upper = static_cast<char>(toupper(static_cast<unsigned char>(ch)));

    for (size_t i = 0; i < sizeof(FONT) / sizeof(FONT[0]); ++i) {
        if (FONT[i].ch == upper) {
            return FONT[i].columns;
        }
    }

    return FONT[0].columns;
}

void clear_buffer()
{
    memset(s_oled_buffer, 0, sizeof(s_oled_buffer));
}

void draw_pixel(int x, int y)
{
    if ((x < 0) || (x >= OLED_WIDTH) || (y < 0) || (y >= OLED_HEIGHT)) {
        return;
    }

    s_oled_buffer[(y / 8) * OLED_WIDTH + x] |= static_cast<uint8_t>(1U << (y % 8));
}

void draw_char(int x, int y, char ch)
{
    const uint8_t *glyph = lookup_glyph(ch);

    for (int col = 0; col < 5; ++col) {
        for (int row = 0; row < 7; ++row) {
            if ((glyph[col] >> row) & 0x01U) {
                draw_pixel(x + col, y + row);
            }
        }
    }
}

void draw_text(int x, int y, const char *text)
{
    if (text == nullptr) {
        return;
    }

    while ((*text != '\0') && (x <= (OLED_WIDTH - 5))) {
        draw_char(x, y, *text);
        x += TEXT_CHAR_WIDTH;
        ++text;
    }
}

int age_ms_from_us(bool valid, int64_t updated_us)
{
    if (!valid) {
        return -1;
    }

    const int64_t age_us = esp_timer_get_time() - updated_us;
    if (age_us <= 0) {
        return 0;
    }

    const int64_t age_ms = age_us / 1000;
    return (age_ms > 9999) ? 9999 : static_cast<int>(age_ms);
}

void render_wait_line(char *line, size_t line_len, const char *label)
{
    snprintf(line, line_len, "%s WAIT", label);
}

void render_pose_page(char lines[TEXT_MAX_LINES][TEXT_LINE_LEN], const telemetry_snapshot_t *snapshot)
{
    snprintf(lines[0], TEXT_LINE_LEN, "POSE BARO");

    if (snapshot->imu.valid) {
        snprintf(lines[1], TEXT_LINE_LEN, "YAW %6.1f", snapshot->imu.yaw_deg);
        snprintf(lines[2], TEXT_LINE_LEN, "PIT %6.1f", snapshot->imu.pitch_deg);
        snprintf(lines[3], TEXT_LINE_LEN, "ROL %6.1f", snapshot->imu.roll_deg);
    } else {
        render_wait_line(lines[1], TEXT_LINE_LEN, "YAW");
        render_wait_line(lines[2], TEXT_LINE_LEN, "PIT");
        render_wait_line(lines[3], TEXT_LINE_LEN, "ROL");
    }

    if (snapshot->baro.valid) {
        snprintf(lines[4], TEXT_LINE_LEN, "P %7.2f HPA", snapshot->baro.pressure_hpa);
        snprintf(lines[5], TEXT_LINE_LEN, "T %7.2f C", snapshot->baro.temp_c);
        snprintf(lines[7], TEXT_LINE_LEN, "BAR %4dMS", age_ms_from_us(true, snapshot->baro.updated_us));
    } else {
        render_wait_line(lines[4], TEXT_LINE_LEN, "BAR");
        render_wait_line(lines[5], TEXT_LINE_LEN, "TEMP");
        render_wait_line(lines[7], TEXT_LINE_LEN, "BAR");
    }

    if (snapshot->imu.valid) {
        snprintf(lines[6], TEXT_LINE_LEN, "IMU %4dMS", age_ms_from_us(true, snapshot->imu.updated_us));
    } else {
        render_wait_line(lines[6], TEXT_LINE_LEN, "IMU");
    }
}

void render_motion_page(char lines[TEXT_MAX_LINES][TEXT_LINE_LEN], const telemetry_snapshot_t *snapshot)
{
    snprintf(lines[0], TEXT_LINE_LEN, "MOTION");

    if (snapshot->imu.valid) {
        snprintf(lines[1], TEXT_LINE_LEN, "GX %7.2f", snapshot->imu.gyro_x_rads);
        snprintf(lines[2], TEXT_LINE_LEN, "GY %7.2f", snapshot->imu.gyro_y_rads);
        snprintf(lines[3], TEXT_LINE_LEN, "GZ %7.2f", snapshot->imu.gyro_z_rads);
        snprintf(lines[4], TEXT_LINE_LEN, "LX %7.2f", snapshot->imu.lin_accel_x_ms2);
        snprintf(lines[5], TEXT_LINE_LEN, "LY %7.2f", snapshot->imu.lin_accel_y_ms2);
        snprintf(lines[6], TEXT_LINE_LEN, "LZ %7.2f", snapshot->imu.lin_accel_z_ms2);
    } else {
        render_wait_line(lines[1], TEXT_LINE_LEN, "GX");
        render_wait_line(lines[2], TEXT_LINE_LEN, "GY");
        render_wait_line(lines[3], TEXT_LINE_LEN, "GZ");
        render_wait_line(lines[4], TEXT_LINE_LEN, "LX");
        render_wait_line(lines[5], TEXT_LINE_LEN, "LY");
        render_wait_line(lines[6], TEXT_LINE_LEN, "LZ");
    }

    if (snapshot->baro.valid) {
        snprintf(lines[7], TEXT_LINE_LEN, "P %7.2f HPA", snapshot->baro.pressure_hpa);
    } else {
        render_wait_line(lines[7], TEXT_LINE_LEN, "BAR");
    }
}

void render_snapshot(const telemetry_snapshot_t *snapshot)
{
    char lines[TEXT_MAX_LINES][TEXT_LINE_LEN] = {};
    const bool show_motion_page = ((esp_timer_get_time() / 1000000) / DISPLAY_PAGE_PERIOD_S) % 2;

    if (show_motion_page) {
        render_motion_page(lines, snapshot);
    } else {
        render_pose_page(lines, snapshot);
    }

    clear_buffer();
    for (int line = 0; line < TEXT_MAX_LINES; ++line) {
        draw_text(0, line * TEXT_LINE_HEIGHT, lines[line]);
    }
}

esp_err_t display_init_panel()
{
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = I2C_PORT;
    bus_config.sda_io_num = PIN_NUM_SDA;
    bus_config.scl_io_num = PIN_NUM_SCL;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = 1;

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_i2c_bus), TAG, "i2c_new_master_bus failed");

    esp_lcd_panel_io_i2c_config_t io_config = {};
    io_config.dev_addr = OLED_I2C_ADDR;
    io_config.control_phase_bytes = 1;
    io_config.dc_bit_offset = 6;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.scl_speed_hz = OLED_PIXEL_CLOCK_HZ;

    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(s_i2c_bus, &io_config, &s_io_handle), TAG, "esp_lcd_new_panel_io_i2c failed");

    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = OLED_HEIGHT,
    };
    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = PIN_NUM_RST;
    panel_config.bits_per_pixel = 1;
    panel_config.vendor_config = &ssd1306_config;

    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ssd1306(s_io_handle, &panel_config, &s_panel), TAG, "esp_lcd_new_panel_ssd1306 failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "esp_lcd_panel_reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "esp_lcd_panel_init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "esp_lcd_panel_disp_on_off failed");

    clear_buffer();
    ESP_RETURN_ON_ERROR(esp_lcd_panel_draw_bitmap(s_panel, 0, 0, OLED_WIDTH, OLED_HEIGHT, s_oled_buffer), TAG, "initial clear failed");

    return ESP_OK;
}

} // namespace

void display_start(void)
{
    const esp_err_t err = display_init_panel();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "display init failed: %s", esp_err_to_name(err));
        return;
    }

    xTaskCreate(
        display_task,
        "display_task",
        4096,
        nullptr,
        4,
        nullptr);
}

void display_task(void *arg)
{
    (void)arg;

    telemetry_snapshot_t snapshot = {};

    while (true) {
        telemetry_load_snapshot(&snapshot);
        render_snapshot(&snapshot);
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(s_panel, 0, 0, OLED_WIDTH, OLED_HEIGHT, s_oled_buffer));
        vTaskDelay(DISPLAY_REFRESH_TICKS);
    }
}
