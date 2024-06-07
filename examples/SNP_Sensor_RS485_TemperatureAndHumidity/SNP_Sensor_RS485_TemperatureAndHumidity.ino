/*
  SNP Sensor temperature and humidity reading with RS485

  This example demonstrates how to use SNP_Sensor library to read temperature and humidity value at the same time
  from SNP Light sensor with RS485 interface.

  Circuit for controller:
  - Digital pin 10 to RS485 RO
  - Digital pin 11 to RS485 DI
  - Digital pin 12 to RS485 DE and RE

  If ESP32 or ESP8266 is used as a controller, It is recommended to use a voltage divider or a level shifter
  between a digital pin and RS485 RO.

  The polling time between reads should be higher than 100 milliseconds to ensure that the sensor is working correctly.

*/

#include <SNP_Sensor.h>

/* User can change controller digital pins here */
#define RO_PIN 10
#define DI_PIN 11
#define DE_PIN 12

SNP_Sensor snpSensor(RO_PIN, DI_PIN, DE_PIN);

float temperature = 0;
float humidity = 0;

void setup()
{
  Serial.begin(9600);
  snpSensor.begin();
}

void loop()
{
  delay(1000);

  snpSensor.readTemperatureAndHumidity(temperature, humidity);
  Serial.print("temperature = ");
  Serial.println(temperature);
  Serial.print("humidity = ");
  Serial.println(humidity);
}
