/*
COLA Arduino
(GPS) Getting the Position (using I2C Protocol)
*/
#ifndef GPS_I2C_h
#define GPS_I2C_h

#include <Wire.h> //Needed for I2C to GNSS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGNSS;

long lastTime = 0; // Simple local timer. Limits amount if I2C traffic to u-blox module.
// int armed = 0; // Wait for the lander to go above 25 meters. (This is for the Rocket_Ignition.ino code)

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
}

void loop()
{
  int gps_location(long latitude, long longitude, long altitude, int armed)
  {
    //Query module only every second. Doing it more often will just cause I2C traffic.
    //The module only responds when a new position is available
    if (millis() - lastTime > 1000)
    {
      lastTime = millis(); //Update the timer
      // *** Currently have it printing but NEED to change it after the testing phase ***
      long latitude = myGNSS.getLatitude();
      // Serial.print(latitude);

      long longitude = myGNSS.getLongitude(); // {degrees*10^-7}

      long altitude = myGNSS.getAltitude(); // {mm}

      if (altitude >= 25000) //COLA lander initially went above 25 meters
      {
        armed = 1;
        return armed;
      }
    }
  }
}
