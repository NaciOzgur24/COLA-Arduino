/*
  COLA's (Test Plan) GPS code
  Gets: Position and Velocity
  Using: (I2C Protocol)
  Pin (20, 21) (Rx, Tx)
*/

#ifndef _GPS_h
#define _GPS_h

#include <Wire.h>
#include <math.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //Sparkfun's GPS Library
SFE_UBLOX_GNSS myGNSS;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
long lastTime2 = 0;
long lastTime3 = 0;
long double latitude = 0; // IDK why long double doesn't work ***NEED to figure that out since it stores more data and is more accurate than double***
long double longitude = 0;
int armed = 0;
int exit_armed = 0;


void gps_setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; //Wait for user to open terminal
  Serial.println("Starting GPS Testing:");
  Wire.begin();
  //Wire.setClock(400000); //Optional. Increase I2C clock speed to 400kHz.

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("GPS module not detected through I2C connection."));
    while (1)
      ;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX);                 //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
}

double gps_latitude()
{
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 25)
  {
    lastTime = millis(); //Update the timer

    double latitude = myGNSS.getLatitude();
    Serial.print(F(" Latitude: "));
    double corrected_latitude = latitude / (10000000);
    Serial.print(corrected_latitude);
    Serial.print(F(" (Degrees)"));

    return corrected_latitude;
  }
}

double gps_longitude()
{
  if (millis() - lastTime2 > 25)
  {
    lastTime2 = millis();

    double longitude = myGNSS.getLongitude();
    Serial.print(F(" Longitude: "));
    double corrected_longitude = longitude / (10000000);
    Serial.print(corrected_longitude);
    Serial.print(F(" (Degrees)"));

    return corrected_longitude;
  }
}

long gps_altitude()
{
  if (millis() - lastTime3 > 25)
  {
    lastTime3 = millis();

    long altitude = myGNSS.getAltitude();
    Serial.print(F(" Altitude: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));
    Serial.println();

    return altitude;
  }
}

int Ignition_Condition()
{
  long altitude_condition = gps_altitude();
  if (armed == 0 && altitude_condition >= 20000) // Once the COLA goes above 20 meters
  {
    return armed = 1;
  }
  if (armed == 1 && altitude_condition == 0) //When COLA lands
  {
    return exit_armed = 1;
  }
}

#endif // _GPS_h
