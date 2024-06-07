#include "Arduino.h"
#include "SNP_Sensor.h"

#include <math.h>

SNP_Sensor::SNP_Sensor(uint8_t rxPin, uint8_t txPin, uint8_t dePin)
{
  _rxPin = rxPin;
  _txPin = txPin;
  _dePin = dePin;
}

void SNP_Sensor::begin(void)
{
  if (_dePin != NO_DE_PIN)
  {
    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW);
  }

#if defined(__AVR__) || defined(ESP8266)
  _softwareSerial = new SoftwareSerial(_rxPin, _txPin);
  _serial = _softwareSerial;
  _softwareSerial->begin(9600);

#elif defined(ESP32)
  Serial1.begin(9600, SERIAL_8N1, _rxPin, _txPin);
  _serial = &Serial1;

#endif // #if defined(__AVR__) || defined(__ESP8266__)

  _charTimeout = (10 * 2500000) / 9600;
  _commTimeout = 50000;
}

float SNP_Sensor::readLight(void)
{
  uint8_t buf[8] = {0x02, 0x04, 0x00, 0x00, 0x00, 0x02, 0x71, 0xf8};
  while (_serial->read() != -1)
  {
  }
  if (_dePin != NO_DE_PIN)
    digitalWrite(_dePin, HIGH);
  _serial->write(buf, 8);
  _serial->flush();
  if (_dePin != NO_DE_PIN)
    digitalWrite(_dePin, LOW);

  uint8_t readBuf[16];
  uint16_t numBytes = 0;
  uint32_t startCharTime = 0;
  uint32_t startTime = micros();
  bool isReceived = false;

  do
  {
    if (_serial->available())
    {
      // received response from slave, won't timeout with communication timeout
      isReceived = true;
      startCharTime = micros();
      readBuf[numBytes] = _serial->read();
      numBytes++;
    }

    if (isReceived)
    {
      // char timeout (slave response too late) or got response (format below)
      if (micros() - startCharTime >= _charTimeout || numBytes >= 9)
      {
        break;
      }
    }
    else
    {
      // communication timeout (no response from slave)
      if (micros() - startTime >= _commTimeout)
      {
        break;
      }
    }

  } while (true);

  if (numBytes != 9)
  {
    // unexpected message
    return NAN;
  }

  uint16_t crc = ((uint16_t)readBuf[8] << 8) | ((uint16_t)readBuf[7]);
  if (_crc16(readBuf, 7) != crc)
  {
    // wrong crc
    return NAN;
  }

  // ID | FUNC | SIZE | MID-LOW | LOW | HIGH | MID-HIGH | CRC-LOW | CRC-HIGH
  // --> uint32_t lux : HIGH | MID-HIGH | MID-LOW | LOW
  uint32_t lux_raw = ((uint32_t)readBuf[5] << 24) | ((uint32_t)readBuf[6] << 16) |
                     ((uint32_t)readBuf[3] << 8) | ((uint32_t)readBuf[4]);

  while (_serial->available())
  {
    _serial->read();
  }

  // lux_raw has 2 decimal points as the last 2 digit
  return lux_raw / 100.0f;
}

float SNP_Sensor::readTemperature(void)
{
  uint8_t buf[8] = {0x01, 0x04, 0x00, 0x01, 0x00, 0x02, 0x20, 0x0b};
  if (_dePin != NO_DE_PIN)
    digitalWrite(_dePin, HIGH);
  _serial->write(buf, 8);
  _serial->flush();
  if (_dePin != NO_DE_PIN)
    digitalWrite(_dePin, LOW);

  uint8_t readBuf[16];
  uint16_t numBytes = 0;
  uint32_t startCharTime = 0;
  uint32_t startTime = micros();
  bool isReceived = false;

  do
  {
    if (_serial->available())
    {
      // received response from slave, won't timeout with communication timeout
      isReceived = true;
      startCharTime = micros();
      readBuf[numBytes] = _serial->read();
      numBytes++;
    }

    if (isReceived)
    {
      // char timeout (slave response too late) or got response (format below)
      if (micros() - startCharTime >= _charTimeout || numBytes >= 9)
      {
        break;
      }
    }
    else
    {
      // communication timeout (no response from slave)
      if (micros() - startTime >= _commTimeout)
      {
        break;
      }
    }

  } while (true);

  if (numBytes != 9)
  {
    // unexpected message
    return NAN;
  }

  uint16_t crc = ((uint16_t)readBuf[8] << 8) | ((uint16_t)readBuf[7]);
  if (_crc16(readBuf, 7) != crc)
  {
    // wrong crc
    return NAN;
  }

  // ID | FUNC | SIZE | TEMP-HIGH | TEMP-LOW | HUMID-HIGH | HUMID-LOW | CRC-LOW | CRC-HIGH
  // --> int16_t temp_raw : TEMP-HIGH | TEMP-LOW
  int16_t temp_raw = ((int16_t)readBuf[3] << 8) | ((int16_t)readBuf[4]);

  while (_serial->available())
  {
    _serial->read();
  }

  // temp_raw has a decimal point as the last digit
  return temp_raw / 10.0f;
}

float SNP_Sensor::readHumidity(void)
{
  uint8_t buf[8] = {0x01, 0x04, 0x00, 0x01, 0x00, 0x02, 0x20, 0x0b};
  while (_serial->read() != -1)
  {
  }
  if (_dePin != NO_DE_PIN)
    digitalWrite(_dePin, HIGH);
  _serial->write(buf, 8);
  _serial->flush();
  if (_dePin != NO_DE_PIN)
    digitalWrite(_dePin, LOW);

  uint8_t readBuf[16];
  uint16_t numBytes = 0;
  uint32_t startCharTime = 0;
  uint32_t startTime = micros();
  bool isReceived = false;
  do
  {
    if (_serial->available())
    {
      // received response from slave, won't timeout with communication timeout
      isReceived = true;
      startCharTime = micros();
      readBuf[numBytes] = _serial->read();
      numBytes++;
    }

    if (isReceived)
    {
      // char timeout (slave response too late) or got response (format below)
      if (micros() - startCharTime >= _charTimeout || numBytes >= 9)
      {
        break;
      }
    }
    else
    {
      // communication timeout (no response from slave)
      if (micros() - startTime >= _commTimeout)
      {
        break;
      }
    }

  } while (true);

  if (numBytes != 9)
  {
    // unexpected message
    return NAN;
  }

  uint16_t crc = ((uint16_t)readBuf[8] << 8) | ((uint16_t)readBuf[7]);
  if (_crc16(readBuf, 7) != crc)
  {
    // wrong crc
    return NAN;
  }

  // ID | FUNC | SIZE | TEMP-HIGH | TEMP-LOW | HUMID-HIGH | HUMID-LOW | CRC-LOW | CRC-HIGH
  // --> uint16_t humidity_raw : HUMID-HIGH | HUMID-LOW
  uint16_t humidity_raw = ((uint16_t)readBuf[5] << 8) | ((uint16_t)readBuf[6]);

  while (_serial->available())
  {
    _serial->read();
  }

  // humidity_raw has a decimal point as the last digit
  return humidity_raw / 10.0f;
}

void SNP_Sensor::readTemperatureAndHumidity(float &temperature, float &humidity)
{
  uint8_t buf[8] = {0x01, 0x04, 0x00, 0x01, 0x00, 0x02, 0x20, 0x0b};
  if (_dePin != NO_DE_PIN)
    digitalWrite(_dePin, HIGH);
  _serial->write(buf, 8);
  _serial->flush();
  if (_dePin != NO_DE_PIN)
    digitalWrite(_dePin, LOW);

  uint8_t readBuf[16];
  uint16_t numBytes = 0;
  uint32_t startCharTime = 0;
  uint32_t startTime = micros();
  bool isReceived = false;
  do
  {
    if (_serial->available())
    {
      // received response from slave, won't timeout with communication timeout
      isReceived = true;
      startCharTime = micros();
      readBuf[numBytes] = _serial->read();
      numBytes++;
    }

    if (isReceived)
    {
      // char timeout (slave response too late) or got response (format below)
      if (micros() - startCharTime >= _charTimeout || numBytes >= 9)
      {
        break;
      }
    }
    else
    {
      // communication timeout (no response from slave)
      if (micros() - startTime >= _commTimeout)
      {
        break;
      }
    }

  } while (true);

  if (numBytes != 9)
  {
    humidity = NAN;
    temperature = NAN;
    return;
  }

  uint16_t crc = ((uint16_t)readBuf[8] << 8) | ((uint16_t)readBuf[7]);
  if (_crc16(readBuf, 7) != crc)
  {
    humidity = NAN;
    temperature = NAN;
    return;
  }

  // ID | FUNC | SIZE | TEMP-HIGH | TEMP-LOW | HUMID-HIGH | HUMID-LOW | CRC-LOW | CRC-HIGH
  // --> uint16_t humidity_raw : HUMID-HIGH | HUMID-LOW
  // --> int16_t temp_raw : TEMP-HIGH | TEMP-LOW
  uint16_t humidity_raw = ((uint16_t)readBuf[5] << 8) | ((uint16_t)readBuf[6]);
  int16_t temp_raw = ((int16_t)readBuf[3] << 8) | ((int16_t)readBuf[4]);

  while (_serial->available())
  {
    _serial->read();
  }

  humidity = humidity_raw / 10.0f;
  temperature = temp_raw / 10.0f;
}

uint16_t SNP_Sensor::_crc16(uint8_t *input, uint8_t len)
{
  uint16_t value = 0xFFFF;
  for (uint8_t i = 0; i < len; i++)
  {
    value ^= (uint16_t)input[i];
    for (uint8_t j = 0; j < 8; j++)
    {
      bool lsb = value & 1;
      value >>= 1;
      if (lsb)
        value ^= 0xA001;
    }
  }
  return value;
}