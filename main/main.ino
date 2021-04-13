/*
COLA Arduino
main code
*/

#include "GPS_I2C.h"
#include "IMU.h"
#include "Rocket_Ignition.h"
#include "Data_Logging.h"
#include "COLAPID.cpp" // ***Prob need to Change***
#include "Servo_Control.h"

void setup()
{
}

void loop()
{
  while (flight_time <= 600)
  {
    //GPS Section
    long latitude = gps_location(long latitude);
    long longitude = gps_location(long longitude);
    long altitude = gps_location(long altitude);

    //IMU Section

    //Rocket Ignition Section

    //Data Logging Section

    //COLA PID Section
    /*
    double gRoll = colaPIDr(double gRoll);   // Roll gymbal angle
    double gPitch = colaPIDp(double gPitch); // Pitch gymbal angle
    */
   
    //Servo Control Section

    //Exit Condition
    int armed = gps_location(int armed);
    if (altitude == 0 && armed == 2)
    {
      //while (1);
      return 0;
    }
  }
}
