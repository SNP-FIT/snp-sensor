#ifndef SNP_SENSOR_H
#define SNP_SENSOR_H

#include "Arduino.h"
#if defined(__AVR__) || defined(__ESP8266__)
#include <SoftwareSerial.h>
#endif // #if defined(__AVR__) || defined(__ESP8266__)

#define NO_DE_PIN 255

class SNP_Sensor
{
public:
    SNP_Sensor(uint8_t rxPin, uint8_t txPin, uint8_t dePin = NO_DE_PIN);
    void begin(void);
    uint32_t readLight(void);
    float readTemperature(void);
    float readHumidity(void);
    void readTemperatureAndHumidity(float &temperature, float &humidity);

private:
    uint8_t _rxPin;
    uint8_t _txPin;
    uint8_t _dePin;
#if defined(__AVR__) || defined(__ESP8266__)
    SoftwareSerial *_softwareSerial;
#endif // #if defined(__AVR__) || defined(__ESP8266__)
    Stream *_serial;
    uint32_t _charTimeout;
    uint32_t _commTimeout;
    uint16_t _crc16(uint8_t *input, uint8_t len);
};

#endif // SNP_SENSOR_H
