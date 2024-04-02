#include <SNP_Sensor.h>

#define RX_PIN 10
#define TX_PIN 11

SNP_Sensor snpSensor(RX_PIN, TX_PIN, NO_DE_PIN);

uint32_t lux = 0;
float temperature = 0;
float humidity = 0;
float temperature2 = 0;
float humidity2 = 0;

void setup()
{
  Serial.begin(9600);

  snpSensor.begin();
}

void loop()
{
  delay(2000);

  lux = snpSensor.readLight();
  Serial.print("lux = ");
  Serial.println(lux);

  temperature = snpSensor.readTemperature();
  Serial.print("temperature = ");
  Serial.println(temperature);

  humidity = snpSensor.readHumidity();
  Serial.print("humidity = ");
  Serial.println(humidity);

  snpSensor.readTemperatureAndHumidity(temperature2, humidity2);
  Serial.print("temperature2 = ");
  Serial.println(temperature2);
  Serial.print("humidity2 = ");
  Serial.println(humidity2);
}
