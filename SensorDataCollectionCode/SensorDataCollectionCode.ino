//Libraries need for sensors
#include <Wire.h> //Needed for I2C communication
#include <Adafruit_Sensor.h> //Unified sensor library
#include <Adafruit_INA219.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2561_U.h>
#include <pgmspace.h> //Additional library required for light?

//Set up global sensor variables
Adafruit_INA219 ina; //Current
Adafruit_BME280 bme; //Pressure/altitude/temp/humidity
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345); //Light intensity
float current, voltage, temperature, humidity, pressure, altitude, lightIntensity = 0;
float SEALEVELPRESSURE_HPA = 1013.25;

void setup()
{
  //Initialize sensors
  ina.begin();
  if (!bme.begin())
    while (1) {
      delay(1000);
      Serial.println("Current sensor not detected.");
    }
  if (!tsl.begin())
    while (1) {
      delay(1000);
      Serial.println("Current sensor not detected.");
    }
  tsl.enableAutoRange(true); //Configures tsl
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS); //Sets tsl's sampling time
}
void loop()
{
  //Read current and voltage data
  current = ina.getCurrent_mA();
  voltage = ina.getBusVoltage_V() + (ina.getShuntVoltage_mV() / 1000);
  Serial.print("Current: "); Serial.print(current); Serial.println(" mA");
  Serial.print("Voltage: "); Serial.print(voltage); Serial.println(" V");

  //Read pressure, altitude, temp, and humidity
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" *C");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
  Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa");
  Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m");

  //Read light intensity data
  sensors_event_t event;
  tsl.getEvent(&event);
  lightIntensity = event.light;
  Serial.print("Light Intensity: "); Serial.print(lightIntensity); Serial.println(" lux");

  delay(1000);
}
