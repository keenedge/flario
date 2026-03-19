#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "settings.h"

#define SETTINGS_NAMESPACE "settings"
#define SETTINGS_KEY_QNH "qnh"
#define DEFAULT_QNH_CENTI_HPA 101325
#define QNH_SAVE_DELAY_US 1000000

static const char *TAG = "SETTINGS";

static portMUX_TYPE s_settings_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_qnh_dirty = false;
static int32_t s_pending_qnh_centi_hpa = DEFAULT_QNH_CENTI_HPA;
static int64_t s_last_qnh_change_us = 0;

static int32_t qnh_hpa_to_centi(float qnh_hpa)
{
    if (qnh_hpa >= 0.0f) {
        return (int32_t) (qnh_hpa * 100.0f + 0.5f);
    }
    return (int32_t) (qnh_hpa * 100.0f - 0.5f);
}

static float qnh_centi_to_hpa(int32_t qnh_centi_hpa)
{
    return qnh_centi_hpa / 100.0f;
}

static esp_err_t settings_save_qnh_centi_hpa(int32_t qnh_centi_hpa)
{
    nvs_handle_t nvs = 0;
    esp_err_t err = ESP_OK;

    ESP_RETURN_ON_ERROR(nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "nvs_open failed");
    err = nvs_set_i32(nvs, SETTINGS_KEY_QNH, qnh_centi_hpa);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_i32 failed: %s", esp_err_to_name(err));
        nvs_close(nvs);
        return err;
    }
    err = nvs_commit(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
        nvs_close(nvs);
        return err;
    }
    nvs_close(nvs);
    return ESP_OK;
}

esp_err_t settings_init(void)
{
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs_flash_erase failed");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs_flash_init failed");
    return ESP_OK;
}

esp_err_t settings_load_qnh_hpa(float *qnh_hpa)
{
    nvs_handle_t nvs = 0;
    int32_t qnh_centi_hpa = DEFAULT_QNH_CENTI_HPA;
    esp_err_t err = ESP_OK;

    if (qnh_hpa == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    err = nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *qnh_hpa = qnh_centi_to_hpa(qnh_centi_hpa);
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs_open failed");

    err = nvs_get_i32(nvs, SETTINGS_KEY_QNH, &qnh_centi_hpa);
    nvs_close(nvs);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        qnh_centi_hpa = DEFAULT_QNH_CENTI_HPA;
        err = ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs_get_i32 failed");

    *qnh_hpa = qnh_centi_to_hpa(qnh_centi_hpa);
    return ESP_OK;
}

void settings_schedule_qnh_save(float qnh_hpa)
{
    portENTER_CRITICAL(&s_settings_lock);
    s_pending_qnh_centi_hpa = qnh_hpa_to_centi(qnh_hpa);
    s_last_qnh_change_us = esp_timer_get_time();
    s_qnh_dirty = true;
    portEXIT_CRITICAL(&s_settings_lock);
}

void settings_service(void)
{
    const int64_t now_us = esp_timer_get_time();
    bool should_save = false;
    int32_t qnh_centi_hpa = DEFAULT_QNH_CENTI_HPA;

    portENTER_CRITICAL(&s_settings_lock);
    if (s_qnh_dirty && ((now_us - s_last_qnh_change_us) >= QNH_SAVE_DELAY_US)) {
        should_save = true;
        qnh_centi_hpa = s_pending_qnh_centi_hpa;
        s_qnh_dirty = false;
    }
    portEXIT_CRITICAL(&s_settings_lock);

    if (!should_save) {
        return;
    }

    const esp_err_t err = settings_save_qnh_centi_hpa(qnh_centi_hpa);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "saved QNH %.2f hPa", qnh_centi_to_hpa(qnh_centi_hpa));
        return;
    }

    portENTER_CRITICAL(&s_settings_lock);
    s_pending_qnh_centi_hpa = qnh_centi_hpa;
    s_last_qnh_change_us = now_us;
    s_qnh_dirty = true;
    portEXIT_CRITICAL(&s_settings_lock);
}
