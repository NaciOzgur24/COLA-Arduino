/*
COLA Arduino
(GPS) Getting the Position Part 2
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGNSS;

unsigned long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
unsigned long startTime = 0; //Used to calc the actual update rate.
unsigned long updateCount = 0; //Used to calc the actual update rate.

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("Starting GPS Testing:");

  Wire.begin();

  // Increase I2C clock speed to 400kHz to cope with the high navigation rate
  // Recommended to run bus at 100kHz
  Wire.setClock(400000); // 400,000 Baud
  
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("FML DIDNT CONNECT."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(5); //Set output to 5 times a second

  //uint8_t rate = myGNSS.getNavigationFrequency(); //Get the update rate of this module

  startTime = millis();
}

void loop()
{
  //Query module every 25 ms. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available. This is defined by the update freq.
  if (millis() - lastTime > 25)
  {
    lastTime = millis(); //Update the timer
    
    long latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees*10^-7)"));

    long altitude = myGNSS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    long altitudeMSL = myGNSS.getAltitudeMSL();
    Serial.print(F(" AltMSL: "));
    Serial.print(altitudeMSL);
    Serial.print(F(" (mm)"));
    
    updateCount++;
    
    //Calculate the actual update rate based on the sketch start time and the number of updates
    Serial.print(F(" Rate: "));
    Serial.print( updateCount / ((millis() - startTime) / 1000.0), 2);
    Serial.print(F("Hz"));
  }
}