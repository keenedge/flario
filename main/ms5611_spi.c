// ms5611_spi.c
#include "ms5611_spi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define CMD_RESET        0x1E
#define CMD_ADC_READ     0x00
#define CMD_CONV_D1      0x40
#define CMD_CONV_D2      0x50
#define CMD_PROM_BASE    0xA0

static esp_err_t ms5611_cmd(ms5611_t *dev, uint8_t cmd)
{
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    return spi_device_transmit(dev->spi, &t);
}

static esp_err_t ms5611_read_bytes(ms5611_t *dev, uint8_t cmd, uint8_t *rx, size_t len)
{
    uint8_t tx[4] = {cmd, 0, 0, 0};
    uint8_t rxb[4] = {0};

    if (len > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rxb,
    };

    esp_err_t err = spi_device_transmit(dev->spi, &t);
    if (err != ESP_OK) {
        return err;
    }

    memcpy(rx, &rxb[1], len);
    return ESP_OK;
}

static TickType_t ms5611_conversion_delay_ticks(ms5611_osr_t osr)
{
    // Conservative delays from datasheet class, rounded up a bit
    switch (osr) {
        case MS5611_OSR_256:  return pdMS_TO_TICKS(1);
        case MS5611_OSR_512:  return pdMS_TO_TICKS(2);
        case MS5611_OSR_1024: return pdMS_TO_TICKS(3);
        case MS5611_OSR_2048: return pdMS_TO_TICKS(5);
        case MS5611_OSR_4096: return pdMS_TO_TICKS(10);
        default:              return pdMS_TO_TICKS(10);
    }
}

esp_err_t ms5611_spi_init(ms5611_t *dev, spi_device_handle_t spi)
{
    if (!dev || !spi) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->spi = spi;

    esp_err_t err = ms5611_reset(dev);
    if (err != ESP_OK) {
        return err;
    }

    return ms5611_read_prom(dev);
}

esp_err_t ms5611_reset(ms5611_t *dev)
{
    esp_err_t err = ms5611_cmd(dev, CMD_RESET);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t ms5611_read_prom(ms5611_t *dev)
{
    for (int i = 0; i < 7; ++i) {
        uint8_t buf[2];
        esp_err_t err = ms5611_read_bytes(dev, CMD_PROM_BASE + (i * 2), buf, 2);
        if (err != ESP_OK) {
            return err;
        }
        dev->C[i] = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return ESP_OK;
}

static esp_err_t ms5611_start_and_read_adc(ms5611_t *dev, uint8_t cmd, uint32_t *value)
{
    esp_err_t err = ms5611_cmd(dev, cmd);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(ms5611_conversion_delay_ticks((ms5611_osr_t)(cmd & 0x0E)));

    uint8_t buf[3];
    err = ms5611_read_bytes(dev, CMD_ADC_READ, buf, 3);
    if (err != ESP_OK) {
        return err;
    }

    *value = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    return ESP_OK;
}

esp_err_t ms5611_read_raw_pressure(ms5611_t *dev, ms5611_osr_t osr, uint32_t *D1)
{
    if (!D1) {
        return ESP_ERR_INVALID_ARG;
    }
    return ms5611_start_and_read_adc(dev, CMD_CONV_D1 | osr, D1);
}

esp_err_t ms5611_read_raw_temperature(ms5611_t *dev, ms5611_osr_t osr, uint32_t *D2)
{
    if (!D2) {
        return ESP_ERR_INVALID_ARG;
    }
    return ms5611_start_and_read_adc(dev, CMD_CONV_D2 | osr, D2);
}

esp_err_t ms5611_read(ms5611_t *dev, ms5611_osr_t osr, int32_t *pressure_pa, float *temp_c)
{
    if (!pressure_pa || !temp_c) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t D1 = 0, D2 = 0;
    esp_err_t err;

    err = ms5611_read_raw_pressure(dev, osr, &D1);
    if (err != ESP_OK) {
        return err;
    }

    err = ms5611_read_raw_temperature(dev, osr, &D2);
    if (err != ESP_OK) {
        return err;
    }

    // First-order compensation from datasheet
    int32_t dT   = (int32_t)D2 - ((int32_t)dev->C[5] << 8);
    int64_t OFF  = ((int64_t)dev->C[2] << 16) + (((int64_t)dev->C[4] * dT) >> 7);
    int64_t SENS = ((int64_t)dev->C[1] << 15) + (((int64_t)dev->C[3] * dT) >> 8);
    int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * dev->C[6]) >> 23);

    // Second-order compensation
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000) {
        T2 = ((int64_t)dT * dT) >> 31;
        OFF2 = 5LL * (TEMP - 2000) * (TEMP - 2000) / 2;
        SENS2 = 5LL * (TEMP - 2000) * (TEMP - 2000) / 4;

        if (TEMP < -1500) {
            OFF2 += 7LL * (TEMP + 1500) * (TEMP + 1500);
            SENS2 += 11LL * (TEMP + 1500) * (TEMP + 1500) / 2;
        }
    }

    TEMP -= (int32_t)T2;
    OFF  -= OFF2;
    SENS -= SENS2;

    int32_t P = (int32_t)(((((int64_t)D1 * SENS) >> 21) - OFF) >> 15);

    *pressure_pa = P;
    *temp_c = TEMP / 100.0f;
    return ESP_OK;
}