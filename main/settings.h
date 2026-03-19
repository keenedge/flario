#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t settings_init(void);
esp_err_t settings_load_qnh_hpa(float *qnh_hpa);
void settings_schedule_qnh_save(float qnh_hpa);
void settings_service(void);

#ifdef __cplusplus
}
#endif
