#include "ms5611_spi.h"

#include <array>
#include <cstring>
#include <cmath>

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MS5611_CMD_READ_ADC       0x00
#define MS5611_CMD_READ_PROM      0xA0
#define MS5611_CMD_RESET          0x1E
#define MS5611_CMD_CONVERT_D1     0x40
#define MS5611_CMD_CONVERT_D2     0x50

MS5611_SPI::MS5611_SPI(spi_host_device_t host, gpio_num_t cs_pin, uint32_t clock_speed_hz)
  : _spi(nullptr)
  , _spi_host(host)
  , _cs_pin(cs_pin)
  , _clock_speed_hz(clock_speed_hz)
  , _auto_add_device(true)
  , _compensation(true)
{
  _samplingRate = OSR_ULTRA_LOW;
  _temperature = MS5611_NOT_READ;
  _pressure = MS5611_NOT_READ;
  _result = MS5611_NOT_READ;
  _deviceID = 0;
}

bool MS5611_SPI::begin()
{
  if (_spi == nullptr && _auto_add_device)
  {
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = _clock_speed_hz;
    devcfg.mode = 0;
    devcfg.spics_io_num = _cs_pin;
    devcfg.queue_size = 1;
    esp_err_t err = spi_bus_add_device(_spi_host, &devcfg, &_spi);
    if (err != ESP_OK)
    {
      _result = MS5611_ERROR_2;
      return false;
    }
  }

  if (_spi == nullptr) return false;

  return reset(0);
}

bool MS5611_SPI::isConnected()
{
  int rv = read();
  return (rv == MS5611_READ_OK);
}

bool MS5611_SPI::reset(uint8_t mathMode)
{
  command(MS5611_CMD_RESET);
  uint64_t start = esp_timer_get_time();
  while (esp_timer_get_time() - start < 10000)
  {
    vTaskDelay(0);
    esp_rom_delay_us(10);
  }

  initConstants(mathMode);

  bool ROM_OK = true;
  for (uint8_t reg = 0; reg < 8; reg++)
  {
    uint16_t tmp = readProm(reg);
    _prom[reg] = tmp;
    if (reg < 7)
    {
      C[reg] *= tmp;
      _deviceID <<= 4;
      _deviceID ^= tmp;
      if (reg > 0)
      {
        ROM_OK = ROM_OK && (tmp != 0);
      }
    }
  }
  bool crc_ok = verifyPromCRC();
  if (!crc_ok) _result = MS5611_ERROR_2;
  return ROM_OK && crc_ok;
}

int MS5611_SPI::read(uint8_t bits)
{
  _result = MS5611_READ_OK;

  convert(MS5611_CMD_CONVERT_D1, bits);
  if (_result) return _result;
  uint32_t _D1 = readADC();
  if (_result) return _result;

  convert(MS5611_CMD_CONVERT_D2, bits);
  if (_result) return _result;
  uint32_t _D2 = readADC();
  if (_result) return _result;

  float dT = _D2 - C[5];
  _temperature = 2000 + dT * C[6];

  float offset =  C[2] + dT * C[4];
  float sens = C[1] + dT * C[3];

  if (_compensation)
  {
    if (_temperature < 2000)
    {
      float T2 = dT * dT * 4.6566128731E-10;
      float t = (_temperature - 2000) * (_temperature - 2000);
      float offset2 = 2.5 * t;
      float sens2 = 1.25 * t;
      if (_temperature < -1500)
      {
        t = (_temperature + 1500) * (_temperature + 1500);
        offset2 += 7 * t;
        sens2 += 5.5 * t;
      }
      _temperature -= T2;
      offset -= offset2;
      sens -= sens2;
    }
  }

  _pressure = (_D1 * sens * 4.76837158205E-7 - offset) * 3.051757813E-5;
  _lastRead = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
  return MS5611_READ_OK;
}

void MS5611_SPI::setOversampling(osr_t samplingRate)
{
  _samplingRate = static_cast<uint8_t>(samplingRate);
}

osr_t MS5611_SPI::getOversampling() const
{
  return static_cast<osr_t>(_samplingRate);
}

float MS5611_SPI::getTemperature() const
{
  if (_temperatureOffset == 0) return _temperature * 0.01f;
  return _temperature * 0.01f + _temperatureOffset;
}

float MS5611_SPI::getPressure() const
{
  if (_pressureOffset == 0) return _pressure * 0.01f;
  return _pressure * 0.01f + _pressureOffset;
}

void MS5611_SPI::setPressureOffset(float offset)
{
  _pressureOffset = offset;
}

float MS5611_SPI::getPressureOffset()
{
  return _pressureOffset;
}

void MS5611_SPI::setTemperatureOffset(float offset)
{
  _temperatureOffset = offset;
}

float MS5611_SPI::getTemperatureOffset()
{
  return _temperatureOffset;
}

float MS5611_SPI::getAltitude(float airPressure)
{
  float ratio = _pressure * 0.01f / airPressure;
  return 44307.694f * (1 - std::pow(ratio, 0.190284f));
}

float MS5611_SPI::getAltitudeFeet(float airPressure)
{
  float ratio = _pressure * 0.01f / airPressure;
  return 145366.45f * (1 - std::pow(ratio, 0.190284f));
}

float MS5611_SPI::getSeaLevelPressure(float pressure, float altitude)
{
  float x = 1 - altitude * 2.256944358E-5f;
  float ratio = std::pow(x, 5.2553026f);
  return pressure / ratio;
}

int MS5611_SPI::getLastResult() const
{
  return _result;
}

uint32_t MS5611_SPI::lastRead() const
{
  return _lastRead;
}

uint32_t MS5611_SPI::getDeviceID() const
{
  return _deviceID;
}

void MS5611_SPI::setCompensation(bool flag)
{
  _compensation = flag;
}

bool MS5611_SPI::getCompensation()
{
  return _compensation;
}

uint16_t MS5611_SPI::getManufacturer()
{
  return readProm(0);
}

uint16_t MS5611_SPI::getSerialCode()
{
  return readProm(7) >> 4;
}

uint16_t MS5611_SPI::getProm(uint8_t index)
{
  return readProm(index);
}

uint16_t MS5611_SPI::getCRC()
{
  return readProm(7) & 0x0F;
}

void MS5611_SPI::convert(const uint8_t addr, uint8_t bits)
{
  uint8_t index = bits;
  if (index < 8) index = 8;
  else if (index > 12) index = 12;
  index -= 8;
  uint8_t offset = index * 2;
  command(addr + offset);

  uint16_t del[5] = {600, 1200, 2300, 4600, 9100};
  uint16_t waitTime = del[index];
  uint64_t start = esp_timer_get_time();
  while (esp_timer_get_time() - start < waitTime)
  {
    vTaskDelay(0);
    esp_rom_delay_us(10);
  }
}

uint16_t MS5611_SPI::readProm(uint8_t reg)
{
  uint8_t promCRCRegister = 7;
  if (reg > promCRCRegister) return 0;

  uint16_t value = 0;
  uint8_t buffer[2] = {};
  if (read(MS5611_CMD_READ_PROM + reg * 2, buffer, 2) != ESP_OK)
  {
    _result = MS5611_ERROR_2;
    return 0;
  }
  value = (static_cast<uint16_t>(buffer[0]) << 8) | buffer[1];
  return value;
}

uint32_t MS5611_SPI::readADC()
{
  uint32_t value = 0;
  uint8_t buffer[3] = {};
  if (read(MS5611_CMD_READ_ADC, buffer, 3) != ESP_OK)
  {
    _result = MS5611_ERROR_2;
    return 0;
  }
  value |= static_cast<uint32_t>(buffer[0]) << 16;
  value |= static_cast<uint32_t>(buffer[1]) << 8;
  value |= buffer[2];
  return value;
}

int MS5611_SPI::command(const uint8_t command)
{
  vTaskDelay(0);
  if (sendCommand(command) != ESP_OK)
  {
    _result = MS5611_ERROR_2;
    return _result;
  }
  return 0;
}

void MS5611_SPI::initConstants(uint8_t mathMode)
{
  C[0] = 1;
  C[1] = 32768L;
  C[2] = 65536L;
  C[3] = 3.90625E-3f;
  C[4] = 7.8125E-3f;
  C[5] = 256;
  C[6] = 1.1920928955E-7f;

  if (mathMode == 1)
  {
    C[1] = 65536L;
    C[2] = 131072L;
    C[3] = 7.8125E-3f;
    C[4] = 1.5625E-2f;
  }
}

bool MS5611_SPI::verifyPromCRC() const
{
  uint16_t prom_copy[8];
  memcpy(prom_copy, _prom, sizeof(prom_copy));
  prom_copy[7] &= 0xFF00;

  uint16_t n_rem = 0;
  for (int cnt = 0; cnt < 16; ++cnt)
  {
    uint16_t value = (cnt & 1) ? (prom_copy[cnt >> 1] & 0xFF) : (prom_copy[cnt >> 1] >> 8);
    n_rem ^= value;
    for (int bit = 8; bit > 0; --bit)
    {
      if (n_rem & 0x8000)
      {
        n_rem = (n_rem << 1) ^ 0x3000;
      }
      else
      {
        n_rem <<= 1;
      }
    }
  }
  uint8_t crc_calc = static_cast<uint8_t>((n_rem >> 12) & 0x0F);
  return crc_calc == static_cast<uint8_t>(_prom[7] & 0x0F);
}

esp_err_t MS5611_SPI::sendCommand(uint8_t command)
{
  if (_spi == nullptr) return ESP_ERR_INVALID_STATE;
  spi_transaction_t transaction = {};
  transaction.length = 8;
  transaction.tx_buffer = &command;
  return spi_device_transmit(_spi, &transaction);
}

esp_err_t MS5611_SPI::read(uint8_t command, uint8_t *buffer, size_t size)
{
  if (_spi == nullptr || size == 0 || size > 3) return ESP_ERR_INVALID_ARG;
  std::array<uint8_t, 4> tx = {};
  std::array<uint8_t, 4> rx = {};
  tx[0] = command;
  spi_transaction_t transaction = {};
  transaction.tx_buffer = tx.data();
  transaction.rx_buffer = rx.data();
  transaction.length = static_cast<int>((1 + size) * 8);
  esp_err_t err = spi_device_transmit(_spi, &transaction);
  if (err == ESP_OK && buffer)
  {
    std::memcpy(buffer, rx.data() + 1, size);
  }
  return err;
}
