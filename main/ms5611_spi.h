// ms5611_spi.h
#pragma once

#include <stdint.h>
#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MS5611_OSR_256  = 0x00,
    MS5611_OSR_512  = 0x02,
    MS5611_OSR_1024 = 0x04,
    MS5611_OSR_2048 = 0x06,
    MS5611_OSR_4096 = 0x08,
} ms5611_osr_t;

typedef struct {
    spi_device_handle_t spi;
    uint16_t C[7];   // C1..C6 valid, index 0 unused
} ms5611_t;

esp_err_t ms5611_spi_init(ms5611_t *dev, spi_device_handle_t spi);
esp_err_t ms5611_reset(ms5611_t *dev);
esp_err_t ms5611_read_prom(ms5611_t *dev);
esp_err_t ms5611_read_raw_pressure(ms5611_t *dev, ms5611_osr_t osr, uint32_t *D1);
esp_err_t ms5611_read_raw_temperature(ms5611_t *dev, ms5611_osr_t osr, uint32_t *D2);
esp_err_t ms5611_read(ms5611_t *dev, ms5611_osr_t osr, int32_t *pressure_pa, float *temp_c);

#ifdef __cplusplus
}
#endif