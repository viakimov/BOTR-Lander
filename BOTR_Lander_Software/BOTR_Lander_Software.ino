#include <Adafruit_TSL2561_U.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_INA219.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_VC0706.h>
#include <SPI.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SLP (1013.25)

// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
SoftwareSerial XBee(2, 3); // RX, TX

Adafruit_INA219 ina;
Adafruit_BME280 bme;
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
Adafruit_BNO055 bno = Adafruit_BNO055();

byte state = 0;
float groundLevelBaro;

SoftwareSerial GPSSerial(8, 7);
Adafruit_GPS GPS(&GPSSerial);

boolean usingInterrupt = false;

#define chipSelect 10

void setup()
{
  Serial.begin(9600);
  GPSSerial.listen();
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(10000);

  pinMode(4, OUTPUT);

  XBee.begin(9600);

  bme.begin();
  delay(1000);
  groundLevelBaro = bme.readAltitude(SLP);

  tsl.begin();
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
  tsl.setGain(TSL2561_GAIN_1X);

  ina.begin();

  bno.begin();
}

uint32_t timer = millis();
void loop()                     // run over and over again
{
  char c = GPS.read();
  if (GPS.newNMEAreceived())
  {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())
  {
    timer = millis();
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
    sensors_event_t event;
    tsl.getEvent(&event);
    transmit(event.light);

    transmit(bme.readAltitude(SLP) - groundLevelBaro);
    transmit(bme.readTemperature());
    transmit(bme.readHumidity());

    transmit(ina.getBusVoltage_V() + (ina.getShuntVoltage_mV() / 1000));
    transmit(ina.getCurrent_mA());

    imu::Vector<3> acc_vec = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    float acc = sqrt(acc_vec.x() * acc_vec.x() + acc_vec.y() * acc_vec.y() + acc_vec.z() * acc_vec.z());

    transmit(acc);
    transmit(state);
    transmit(GPS.fix);
    if (GPS.fix)
    {
      transmit(GPS.latitudeDegrees);
      transmit(GPS.longitudeDegrees);
    }
    newLine();

    if (state == 0)
    {
      if (bme.readAltitude(SLP) - groundLevelBaro > 10.0 && acc / 9.81 > 2)
      {
        state = 1;
      }
    }
    if (state == 1)
    {
      if (bme.readAltitude(SLP) - groundLevelBaro < 10.0 && acc / 9.81 < 1.3)
      {
        transmit(999.999);
        newLine();
        delay(5000);
        digitalWrite(4, HIGH);
        delay(5000);
        digitalWrite(4, LOW);
        state = 2;
      }
    }
  }
}
void transmit(double f)
{
  Serial.print(f, 6);
  XBee.listen();
  XBee.print(f, 6);
  Serial.print(" ");
  XBee.print(" ");
  GPSSerial.listen();
}
void newLine()
{
  Serial.println();
  XBee.println();
}
