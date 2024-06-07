#ifndef SNP_SENSOR_H
#define SNP_SENSOR_H

#include "Arduino.h"
#if defined(__AVR__) || defined(ESP8266)
#include <SoftwareSerial.h>
#endif // #if defined(__AVR__) || defined(ESP8266)

#define NO_DE_PIN 255

class SNP_Sensor
{
public:
    SNP_Sensor(uint8_t rxPin, uint8_t txPin, uint8_t dePin = NO_DE_PIN);
    void begin(void);
    float readLight(void);
    float readTemperature(void);
    float readHumidity(void);
    void readTemperatureAndHumidity(float &temperature, float &humidity);

private:
    uint8_t _rxPin;
    uint8_t _txPin;
    uint8_t _dePin;
#if defined(__AVR__) || defined(ESP8266)
    SoftwareSerial *_softwareSerial;
#endif // #if defined(__AVR__) || defined(ESP8266)
    Stream *_serial;
    uint32_t _charTimeout;
    uint32_t _commTimeout;
    uint16_t _crc16(uint8_t *input, uint8_t len);
};

#endif // SNP_SENSOR_H
