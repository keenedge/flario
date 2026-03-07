#pragma once

#include <cstdint>
#include <cstddef>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define MS5611_READ_OK  0
#define MS5611_ERROR_2  2
#define MS5611_NOT_READ -999

enum osr_t
{
  OSR_ULTRA_HIGH = 12,
  OSR_HIGH       = 11,
  OSR_STANDARD   = 10,
  OSR_LOW        = 9,
  OSR_ULTRA_LOW  = 8
};

class MS5611_SPI
{
public:
  MS5611_SPI(spi_host_device_t host, gpio_num_t cs_pin, uint32_t clock_speed_hz = 1000000);

  bool     begin();
  bool     isConnected();
  bool     reset(uint8_t mathMode = 0);

  int      read(uint8_t bits);
  inline int read() { return read(static_cast<uint8_t>(_samplingRate)); }

  void     setOversampling(osr_t samplingRate);
  osr_t    getOversampling() const;

  float    getTemperature() const;
  float    getPressure() const;

  void     setPressureOffset(float offset = 0);
  float    getPressureOffset();
  void     setTemperatureOffset(float offset = 0);
  float    getTemperatureOffset();

  float    getAltitude(float airPressure = 1013.25);
  float    getAltitudeFeet(float airPressure = 1013.25);
  float    getSeaLevelPressure(float pressure, float altitude);

  int      getLastResult() const;
  uint32_t lastRead() const;
  uint32_t getDeviceID() const;

  void     setCompensation(bool flag = true);
  bool     getCompensation();

  uint16_t getManufacturer();
  uint16_t getSerialCode();
  uint16_t getProm(uint8_t index);
  uint16_t getCRC();

  void     setSPIspeed(uint32_t speed);
  uint32_t getSPIspeed() const;

protected:
  void     convert(const uint8_t addr, uint8_t bits);
  uint32_t readADC();
  uint16_t readProm(uint8_t reg);
  int      command(const uint8_t command);
  void     initConstants(uint8_t mathMode);
  bool     verifyPromCRC() const;

private:
  esp_err_t sendCommand(uint8_t command);
  esp_err_t read(uint8_t command, uint8_t *buffer, size_t size);

  uint8_t  _address = 0;
  uint8_t  _samplingRate = OSR_ULTRA_LOW;
  int32_t  _temperature = MS5611_NOT_READ;
  int32_t  _pressure = MS5611_NOT_READ;
  float    _pressureOffset = 0;
  float    _temperatureOffset = 0;
  int      _result = MS5611_NOT_READ;
  float    C[7] = {0};
  uint32_t _lastRead = 0;
  uint32_t _deviceID = 0;
  uint16_t _prom[8] = {0};
  spi_device_handle_t _spi = nullptr;
  spi_host_device_t _spi_host = SPI2_HOST;
  gpio_num_t _cs_pin = GPIO_NUM_NC;
  uint32_t _clock_speed_hz = 1000000;
  bool _auto_add_device = false;
  gpio_num_t _select = GPIO_NUM_NC;
  gpio_num_t _dataOut = GPIO_NUM_NC;
  gpio_num_t _dataIn = GPIO_NUM_NC;
  gpio_num_t _clock = GPIO_NUM_NC;
  bool     _compensation = true;
  uint32_t _SPIspeed = 1000000;
};
