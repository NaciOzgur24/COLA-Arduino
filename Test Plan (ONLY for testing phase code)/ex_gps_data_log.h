/*
  Example code for GPS Altitude Data Log
*/

#ifndef _GPS_DATALOG_H
#define _GPS_DATALOG_H

#include <Wire.h>
#include <math.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGNSS;

long lastTime = 0;


void gps_setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Starting GPS Testing:");
  Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("GPS module not detected through I2C connection."));
    while (1)
      ;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
}

long gps_altitude()
{
  if (millis() - lastTime > 25)
  {
    lastTime = millis();

    long altitude = myGNSS.getAltitude();
    Serial.print(F(" Altitude: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));
    Serial.println();
    
    return altitude;
  }
}

#endif // _GPS_DATALOG_H
