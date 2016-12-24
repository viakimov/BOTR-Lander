void setup() 
{
 

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

