/*
  SNP Sensor humidity reading with serial communication

  This example demonstrates how to use SNP_Sensor library to read humidity value from SNP Light sensor
  with serial communication interface.

  Circuit for controller:
  - Digital pin 10 to Sensor Tx
  - Digital pin 11 to Sensor Rx

  If ESP32 or ESP8266 is used as a controller, It is recommended to use a voltage divider or a level shifter
  between digital pins and sensor pins.

  The polling time between reads should be higher than 100 milliseconds to ensure that the sensor is working correctly.

*/

#include <SNP_Sensor.h>

/* User can change controller digital pins here */
#define RX_PIN 10
#define TX_PIN 11

SNP_Sensor snpSensor(RX_PIN, TX_PIN, NO_DE_PIN);

float humidity = 0;

void setup()
{
  Serial.begin(9600);
  snpSensor.begin();
}

void loop()
{
  delay(1000);

  humidity = snpSensor.readHumidity();
  Serial.print("humidity = ");
  Serial.println(humidity);
}
