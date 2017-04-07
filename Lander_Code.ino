//Libraries need for sensors
#include <Wire.h> //Needed for I2C communication
#include <Adafruit_Sensor.h> //Unified sensor library
#include <Adafruit_BME280.h> // for barometer/temp/humidity sensor
#include <Adafruit_INA219.h> // for current sensor
#include <Adafruit_TSL2561_U.h> //light sensor
#include <CurieIMU.h> //accelerometer
#include <Adafruit_GPS.h> //GPS
#include <SoftwareSerial.h> //SD Card
#include <SD.h> //SD Card

#define SEALEVELPRESSURE_HPA 1013.25 //sea level pressure
#define launchThreshold 3.0
#define landingThreshold 2.0
#define XBEEaddress 0x0000

//Set up global file variable
File dataFile  = SD.open("data.txt", FILE_WRITE);

//Set up global state variable
byte state = 0; //0 is not launched, 1 is in flight, 2 is landed

//Set up global sensor variables
Adafruit_INA219 ina; //Current
Adafruit_BME280 bme; //Pressure/altitude/temp/humidity
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345); //Light intensity

//Set up global data variables
float current, voltage, temperature = 0; //Voltage in Volts, Current in Amps, Temperature in Kelvin
float humidity, pressure = 0; //Humidity in Percent Humidity, Pressure in Kilopascals
float altitude, launchAltitude, lightIntensity = 0; //Altitudes in Meters, lightIntensity in lux
float accelerationx, accelerationy, accelerationz, netAcceleration = 0; //Accelerations in m/s^2
float longitude, latitude = 0; //Location in degrees
String data = ""; //String to be sent

//Set up GPS data variables
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO  false

//Set up global transmission variables
uint8_t payload[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//XBEE packets go here!

void setup() {
  Serial.begin(115200);

  bme.begin();
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,   // temperature
                  Adafruit_BME280::SAMPLING_NONE, // pressure
                  Adafruit_BME280::SAMPLING_X1,   // humidity
                  Adafruit_BME280::FILTER_OFF );

  ina.begin();
  ina.setCalibration_16V_400mA();

  tsl.begin();
  tsl.enableAutoRange(true);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);

  CurieIMU.begin();
  CurieIMU.setAccelerometerRange(8);

  //  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
}

void loop()
{
  if (state == 0)
  {
    //    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    //    pressure = bme.readPressure() / 1000.0;
    //    writeFloat(altitude, 0, 3);
    //    writeFloat(pressure, 4, 7);
    CurieIMU.readAccelerometerScaled(accelerationx, accelerationy, accelerationz);
    netAcceleration = sqrt(pow(accelerationx, 2) + pow(accelerationy, 2) + pow(accelerationz, 2));
    //    writeFloat(netAcceleration, 36, 39);
    byte checksum = 0;
    for(byte i = 0; i <= 23; i++)
    {
      checksum + payload[i];
    }
    payload[24] = checksum;
    data = "Waiting to launch! "; //Transmit a waiting pulse to ground station
    if (netAcceleration > launchThreshold);
    {
      delay(250);
      if (altitude - 3 > launchAltitude)
      {
        state = 1;
      }
    }
    dataFile.println(data);
    data = "";
  }
  else if (state == 1)
  {
    GPS.read();
    //    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    //    pressure = bme.readPressure() / 1000.0;
    //    current = ina.getCurrent_mA() / 1000.0;
    voltage = ina.getBusVoltage_V() + (ina.getShuntVoltage_mV() / 1000);
    latitude = GPS.latitudeDegrees;
    longitude = GPS.longitudeDegrees;
    //    writeFloat(altitude, 0, 3);
    //    writeFloat(pressure, 4, 7);
    //    writeFloat(current, 8, 11);
    writeFloat(voltage, 0, 3);
    writeFloat(latitude, 4, 7);
    writeFloat(longitude, 8, 11);
    CurieIMU.readAccelerometerScaled(accelerationx, accelerationy, accelerationz);
    netAcceleration = sqrt(pow(accelerationx, 2) + pow(accelerationy, 2) + pow(accelerationz, 2));
    data = "In Flight! " + String(voltage) + " " + String(latitude) + " " + String(longitude); //Transmit a informational pulse to ground station
    //    writeFloat(netAcceleration, 36, 39);
    if (netAcceleration > landingThreshold);
    {
      delay(250);
      if (altitude - 3 < launchAltitude)
      {
        state = 2;
      }
    }
    byte checksum = 0;
    for(byte i = 0; i <= 23; i++)
    {
      checksum + payload[i];
    }
    payload[24] = checksum;
    dataFile.println(data);
    data = "Debug! " + String(GPS.satellites) + " " + String(GPS.fix) + " " + String(GPS.fixquality);
    dataFile.println(data);
    data = "";
  }
  else if (state == 2)
  {
    GPS.read();
    //    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    //    pressure = bme.readPressure() / 1000.0;
    //    current = ina.getCurrent_mA() / 1000.0;
    voltage = ina.getBusVoltage_V() + (ina.getShuntVoltage_mV() / 1000);
    temperature = bme.readTemperature() + 273.15;
    humidity = bme.readHumidity();
    sensors_event_t event;
    tsl.getEvent(&event);
    lightIntensity = event.light;
    latitude = GPS.latitudeDegrees;
    longitude = GPS.longitudeDegrees;
    //    writeFloat(altitude, 0, 3);
    //    writeFloat(pressure, 4, 7);
    //    writeFloat(current, 8, 11);
    writeFloat(voltage, 0, 3);
    writeFloat(latitude, 4, 7);
    writeFloat(longitude, 8, 11);
    writeFloat(temperature, 12, 15);
    writeFloat(humidity, 16, 19);
    writeFloat(lightIntensity,  20, 23);
    byte checksum = 0;
    for(byte i = 0; i <= 23; i++)
    {
      checksum + payload[i];
    }
    payload[24] = checksum;
    data = "Landed! " + String(voltage) + " " + String(temperature) + " " + String(humidity) + " " + String(lightIntensity)  + " " + String(latitude) + " " + String(longitude); //Transmit a informational pulse to ground station
    dataFile.println(data);
    data = "Debug! " + String(GPS.satellites) + " " + String(GPS.fix) + " " + String(GPS.fixquality);
    dataFile.println(data);
    data = "";
  }
  else
  {
    while (1);
  }
}

void writeFloat(float f, int start, int finish) //writes float to data packet
{
  unsigned int asInt = *((int*)&f);
  for (int i = start; i <= finish; i++)
  {
    payload[i] = (unsigned char)((asInt >> 8 * i) & 0xFF);
  }
}
