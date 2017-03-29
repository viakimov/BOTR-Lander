#include <Adafruit_Sensor.h>  //general
#include <Wire.h>
#include <Adafruit_BME280.h> // for barometer/temp/humidity sensor
#include <Adafruit_INA219.h> // for current sensor
#include <Adafruit_TSL2561_U.h>//light sensor
#include <SoftwareSerial.h> //xbee
#include <XBee.h>
#include <CurieIMU.h> //accelerometer

//Need to configure
Xbee xbee; //Using API mode
Adafruit_BME280 bme;  //barometer/temp/alt sensor
Adafruit_INA219 ina;    //current sensor
Adafruit_TSL2561_Unified tsl;

float SEALEVELPRESSURE_HPA = 1013.25;
float pressure, altitude, temperature, humidity, voltage, lightIntensity; //telemetry
Tx16Request tx;
uint8_t payload[] = uint8_t[6*4]; //data packet to send

unsigned long start = millis();
unsigned long time;

TxStatusResponse txStatus = TxStatusResponse();
Tx16Request tx = Tx16Request(0x1874, payload, sizeof(payload));

void setup()
{
    Serial.begin(9600);
    xbee.setSerial(9600);
    if(!bme.begin())
    {
        Serial.print("BME not detected");
        while(1);
    }
    if(!tsl.begin())
    {
        Serial.print("TSL not detected");
        while(1);
    }
    ina.begin();
    ina219.setCalibration_16V_400mA()
        
    tsl.enableAutoRange(true);
	tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS); 
	
	CurieIMU.begin();
	CurieIMU.setAccelerometerRange(4);
}

void loop() 
{
//High level outline of lander operations, edit the methods below
powerUp();
integration();
launch();
deployment();
landing();
groundOperation();
}

void powerUp()
{
  //Put all components not utilized during launch into low power modes
  //Once the XBee has established a link with the GS XBee,
  //Repeatedly transmit a basic packet of confirmation data until you receive a reply from the GS
  //The GS will use this transmission to display a connection confirmation and to understand when to expect data from the lander
  //Wait a few seconds and transmit a reply confirmation message to the GS
}

void integration()
{
  //Wait until an integration command is received from the GS
  //This command will be sent by a user operating the GS and will signify the lander is integrated into the rocket and ready for launch
}

void launch()
{
	uint8_t payload_launch[] = uint8_t[6*2];
	while(bmp.readAltitude(seaLevelPressure) < 155) //Rounded up to 155 meters, around 510 feet to account for error
	{
	float altitude = bmp.readAltitude(seaLevelPressure);
	payload_launch[0]=(byte)('A'); //A for altitude reading
    payload_launch[5]=(byte)('.');
    writeFloat(altitude, payload_launch, 1, 4);
	time = Micros();
	float ax, ay, az;
    CurieIMU.readAccelerometerScaled(ax, ay, az);
	payload[6]=(byte)('C'); //C for acceleration reading
    payload[11]=(byte)('.');
    writeFloat(ax, payload_launch, 7, 10); //Identify which axis is actually facing in desired direction
	while(time % 1000000 > 4)
	{
		time = Micros();
	}
		Tx16Request tx2 = Tx16Request(0x1874, payload_launch, sizeof(payload_launch));
		xbee.send(tx2);
	}
  //In in one second intervals:
  //Measure altitude from the barometer and log it in the SD card
  //Calculate acceleration from barometric data and log it in the SD card
  //Transmit the altitude and acceleration to the GS
  //Do this until we reach 500 feet 
  //Send a confirmation to the GS that we have reached 500 feet
}

void deployment()
{
  //In in one second intervals:
  //Measure altitude from the barometer and log it in the SD card
  //Calculate acceleration from barometric data and log it in the SD card
  //Transmit the altitude and acceleration to the GS 
  //Do this until we are on the ground 
  //Send a confirmation to the GS that we are on the ground
}

void landing()
{
  //Open the side panels
  //Disable low power modes of components
  //Send a confirmation to the GS that we have uprighted
}

void groundOperation()
{
 //Maximize bandwith usage and point scoring. Ensure everything is timed correctly to avoid skipping transmissions because of data processing or servo operation delays
 //It is probably more effective to repeatedly send remotely-pointed images instead of bothering with the other telemetry.
 //Each remotely pointed image is worth 15 points each and I've read that they should be around 200kb each. 
 //Even if we just send images, we still need to send some sensor data at the begining to meet rule requirements

 //Send basic sensor data (light, temp, humidity, battery voltage, GPS) to GS several times
    for(int i=0; i<10; i++)
    {
        /*each reading has 4 bytes + letter (start) and period (end) = 6 bytes * 4 sensor readings */
        //payload = uint8_t[6*4]
        
 		time = Micros();
        /*finds temperature in Celcius*/
        temperature = bme.readTemperature();
        Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" C"); //for ground testing
        /*writes pressure to packet*/
        payload[0]=(byte)('T'); //T for temperature reading
        payload[5]=(byte)('.');
        writeFloat(temperature, payload, 1, 4);
        
        /*finds humidity in percent*/
        humidity = bme.readHumidity();
        Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %"); //for ground testing
        /*writes pressure to packet*/
        payload[6]=(byte)('H'); //H for humidity reading
        payload[11]=(byte)('.');
        writeFloat(temperature, payload, 7, 10);
        
        /*finds voltage in volts*/
        voltage = ina.getBusVoltage_V() + ina.getShuntVoltage_mV()/1000; //load voltage
        Serial.print("Voltage: "); Serial.print(voltage); Serial.println(" V"); //for ground testing
        /*writes voltage to packet*/
        payload[12]=(byte)('V'); //V for voltage reading
        payload[17]=(byte)('.');
        writeFloat(temperature, payload, 13, 16);
        
        /*finds light intensity in lux*/
        sensors_event_t event;
        tsl.getEvent(&event);
        if (event.light)
        {
            lightIntensity = event.light;
            Serial.print("Light: "); Serial.print(lightIntensity); Serial.println(" lux"); //for ground testing
            /*writes voltage to packet*/
            payload[18]=(byte)('L'); //L for light reading
            payload[23]=(byte)('.');
            writeFloat(temperature, payload, 19, 22);
        }
        
        //Send telemtry packet
        xbee.send(tx);
	
        time = Micros();
	while(time % 1000000 > 4)
	{
		time = Micros();
	}
    }
 
 //Then send images repeatedly:
 while(true)
 {
  //This will probably require reading the milliseconds clock to time everything:
  
  //For 0.25 seconds: Listen for transmissions from GS
  //If panorama command is received:
  takePanorama();
  //If camera rotation command is received
  rotateCamera();
  
  //**To most effectivly use our transmission bandwith we need to have at least 2 images stored at a time.
  //**A transmission can either have data from one image or data from the end of one image plus data from the begining of another.
  
  //If less than 2 images stored: Take picture
  
  //When the millisecond clock is at a whole second, transmit as much image data as possible
 }
}

void takePanorama()
{
  
}

void rotateCamera()
{
  
}

void writeFloat(float f, uint8_t* &payload, start, end) //writes float to data packet
{
    unsigned int asInt = *((int*)&f);
    for (int i = start; i <= end; i++)
    {
        payload[i] = (asInt >> 8 * i) & 0xFF;
    }
}
