//Libraries need for sensors
#include <Wire.h> //Needed for I2C communication
#include <Adafruit_Sensor.h> //Unified sensor library
#include <Adafruit_BME280.h> // for barometer/temp/humidity sensor
#include <Adafruit_INA219.h> // for current sensor
#include <Adafruit_TSL2561_U.h>//light sensor
#include <CurieIMU.h> //accelerometer

#define SEALEVELPRESSURE_HPA 1013.25 //sea level pressure
#define launchThreshold 0.0;
#define landingThreshold 0.0;
#define XBEEaddress 0x0000;

//Set up global state variable
byte state = 0; //0 is not launched, 1 is in flight, 2 is landed

//Set up global sensor variables
const Adafruit_INA219 ina; //Current
const Adafruit_BME280 bme; //Pressure/altitude/temp/humidity
const Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345); //Light intensity

//Set up global data variables
float current, voltage, temperature = 0; //Voltage in Volts, Current in Amps, Temperature in Kelvin
float humidity, pressure = 0; //Humidity in Percent Humidity, Pressure in Kilopascals
float altitude, launchAltitude, lightIntensity = 0; //Altitudes in Meters, lightIntensity in lux
float accelerationx, accelerationy, accelerationz = 0; //Accelerations in m/s^2
String data = ""; //String to be sent

//Set up global transmission variables
//Byte data transmission arrays go here!
//XBEE packets go here!

void setup() {
  Serial.begin(115200);
  
  bme.begin()
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,  // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X1,  // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );
 
  ina.begin();
  ina219.setCalibration_16V_400mA()
  
  tsl.begin()
  tsl.enableAutoRange(true);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);

  CurieIMU.begin();
  CurieIMU.setAccelerometerRange(8);

  altitude = bme280.readAltitude(SEALEVELPRESSURE_HPA);
}

void loop()
{
  if(state == 0)
  {
    altitude = bme280.readAltitude(SEALEVELPRESSURE_HPA);
    pressure = bme280.readPressure()/1000.0;
    data = "Waiting to launch! " + String(altitude) + " " + String(pressure); //Transmit a waiting pulse to ground station
    CurieIMU.readAccelerometerScaled(accelerationx, accelerationy, accelerationz);
    if(sqrt(pow(accelerationx, 2) + pow(accelerationy, 2) + pow(accelerationz, 2)) > launchThreshold);
    { 
      delay(250);
      if(altitude - 3 > launchAltitude)
      {
        state = 1;
      }
    }
    data = "";
  }
  else if(state == 1)
  {
    altitude = bme280.readAltitude(SEALEVELPRESSURE_HPA);
    pressure = bme280.readPressure() / 1000.0;
    current = ina219.getCurrent_mA() / 1000.0;
    voltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV()/1000);
    data = "In Flight! " + String(altitude) + " " + String(pressure) + " " + String(current) + " " + String(voltage); //Transmit a informational pulse to ground station
    CurieIMU.readAccelerometerScaled(accelerationx, accelerationy, accelerationz);
    if(sqrt(pow(accelerationx, 2) + pow(accelerationy, 2) + pow(accelerationz, 2)) > landingThreshold);
    {
      delay(250);
      if(altitude - 3 < launchAltitude)
      {
        state = 2;
      }
    }
    data = "";
  }
  else if(state == 2)
  {
    temperature = bme.readTemperature() + 273.15;
    humidity = bme.readHumidity();
    altitude = bme280.readAltitude(SEALEVELPRESSURE_HPA);
    pressure = bme280.readPressure() / 1000.0;
    current = ina219.getCurrent_mA() / 1000.0;
    voltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV()/1000);
    sensors_event_t event;
    tsl.getEvent(&event);
    lightIntensity = event.light;
    data = "Landed! " + String(altitude) + " " + String(pressure) + " " + String(current) + " " + String(voltage) + " " + String(temperature) + " " + String(humidity) + " " + String(lightIntensity); //Transmit a informational pulse to ground station
    data = "";
  }
  else
  {
    while(1);
  }
}
