/*
COLA Arduino
(GPS) Getting the Position (using I2C Protocol)
*/
#ifndef GPS_I2C_h
#define GPS_I2C_h

#include <Wire.h> //I2C communication protocol
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGNSS;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
int armed = 0;

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  //pinMode(2, INPUT); //Digital sensor is on digital pin 2 ***IDK if I need to set a pin location to it***
  Serial.println("Starting GPS Testing:");
  Wire.begin();

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("GPS module not detected through I2C connection."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
}

void loop()
{
  long gps_location(long latitude, long longitude, long altitude, int armed) // Maybe use ***byte*** instead of long
  {
    //Query module only every second. Doing it more often will just cause I2C traffic.
    //The module only responds when a new position is available
    if (millis() - lastTime > 1000)
    {
      lastTime = millis(); //Update the timer
      
      long latitude = myGNSS.getLatitude();
      Serial.print(latitude);

      long longitude = myGNSS.getLongitude(); // {degrees*10^-7}
      Serial.print(longitude);

      long altitude = myGNSS.getAltitude(); // {mm}
      Serial.print(altitude);

      if (altitude >= 20000) //When COLA lander initially goes above 20 meters
      {
        armed = 1;
        //return armed;
      }
      if (armed == 1 && altitude == 0) //When COLA lands
      {
        exit_armed = 2;
        //return armed;
      }
    }
  }
}


#endif