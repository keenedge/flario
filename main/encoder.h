#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t encoder_start(void);
int32_t encoder_get_position(void);
uint8_t encoder_get_page(uint8_t page_count);
bool encoder_is_button_pressed(void);
uint32_t encoder_get_button_press_count(void);

#ifdef __cplusplus
}
#endif
