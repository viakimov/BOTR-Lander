//Libraries need for sensors
#include <Wire.h> //Needed for I2C communication
#include <Adafruit_Sensor.h> //Unified sensor library
#include <Adafruit_BME280.h> // for barometer/temp/humidity sensor
#include <Adafruit_INA219.h> // for current sensor
#include <Adafruit_TSL2561_U.h>//light sensor
#include <CurieIMU.h> //accelerometer
#include <SPI.h> //SD Card
#include <Adafruit_GPS.h> //GPS
#include <SoftwareSerial.h> //SD Card
#include <SD.h> //SD Card

#define SEALEVELPRESSURE_HPA 1013.25 //sea level pressure
#define launchThreshold 0.0;
#define landingThreshold 0.0;
#define XBEEaddress 0x0000;

//Set up global file variable
File dataFile  = SD.open("data.txt", FILE_WRITE);

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
float longitude, latitude = 0; //Location in degrees
String data = ""; //String to be sent

//Set up GPS data variables
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false

//Set up global transmission variables
uint8_t payload[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
}

void loop()
{
  if(state == 0)
  {
    altitude = bme280.readAltitude(SEALEVELPRESSURE_HPA);
    pressure = bme280.readPressure()/1000.0;
    writeFloat(altitude, payload, 0, 3);
    writeFloat(pressure, payload, 4, 7);
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
    dataFile.println(data);
    data = "";
  }
  else if(state == 1)
  {
    GPS.read();
    altitude = bme280.readAltitude(SEALEVELPRESSURE_HPA);
    pressure = bme280.readPressure() / 1000.0;
    current = ina219.getCurrent_mA() / 1000.0;
    voltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV()/1000);
    latitude = GPS.latitudeDegrees;
    longitude = GPS.longitudeDegrees;
    writeFloat(altitude, payload, 0, 3);
    writeFloat(pressure, payload, 4, 7);
    writeFloat(current, payload, 8, 11);
    writeFloat(voltage, payload, 12, 15);
    writeFloat(latitude, payload, 28, 31);
    writeFloat(longitude, payload, 32, 35);
    data = "In Flight! " + String(altitude) + " " + String(pressure) + " " + String(current) + " " + String(voltage) + " " + String(latitude) + " " + String(longitude); //Transmit a informational pulse to ground station
    CurieIMU.readAccelerometerScaled(accelerationx, accelerationy, accelerationz);
    if(sqrt(pow(accelerationx, 2) + pow(accelerationy, 2) + pow(accelerationz, 2)) > landingThreshold);
    {
      delay(250);
      if(altitude - 3 < launchAltitude)
      {
        state = 2;
      }
    }
    dataFile.println(data);
    data = "Debug! " + String(GPS.satellites) + " " + String(GPS.fix) + " " + String(GPS.fixquality);
    dataFile.println(data);
    data = "";
  }
  else if(state == 2)
  {
    GPS.read();
    altitude = bme280.readAltitude(SEALEVELPRESSURE_HPA);
    pressure = bme280.readPressure() / 1000.0;
    current = ina219.getCurrent_mA() / 1000.0;
    voltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV()/1000);
    temperature = bme.readTemperature() + 273.15;
    humidity = bme.readHumidity();
    sensors_event_t event;
    tsl.getEvent(&event);
    lightIntensity = event.light;
    latitude = GPS.latitudeDegrees;
    longitude = GPS.longitudeDegrees;
    writeFloat(altitude, payload, 0, 3);
    writeFloat(pressure, payload, 4, 7);
    writeFloat(current, payload, 8, 11);
    writeFloat(voltage, payload, 12, 15);
    writeFloat(temperature, payload, 16, 19);
    writeFloat(humidity, payload, 20, 23);
    writeFloat(lightIntensity, payload, 24, 27);
    writeFloat(latitude, payload, 28, 31);
    writeFloat(longitude, payload, 32, 35)du;
    data = "Landed! " + String(altitude) + " " + String(pressure) + " " + String(current) + " " + String(voltage) + " " + String(temperature) + " " + String(humidity) + " " + String(lightIntensity)  + " " + String(latitude) + " " + String(longitude); //Transmit a informational pulse to ground station
    dataFile.println(data);
    data = "Debug! " + String(GPS.satellites) + " " + String(GPS.fix) + " " + String(GPS.fixquality);
    dataFile.println(data);
    data = "";
    //GPS.satellites, GPS.fix, GPS.fixquality for debugging
  }
  else
  {
    while(1);
  }
}

void writeFloat(float f, uint8_t* &payload, start, end) //writes float to data packet
{
  unsigned int asInt = *((int*)&f);
  for (int i = start; i <= end; i++)
  {
    payload[i] = (asInt >> 8 * i) & 0xFF;
  }
}
