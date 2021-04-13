/*
COLA Arduino
main code
*/


#include "GPS_I2C.h"
#include "IMU.h"
#include "Rocket_Ignition.h"
#include "Data_Logging.h"
#include "COLAPID.cpp" // ***Prob need to Change***
#include "Gimbal2Servo.h"
#include "Servo_Control.h"

/*
  Serial Pins being used:
  IMU sensor: (0, 1) Rx, Tx
  Data Logging: (15, 14) Rx, Tx and (17, 16) Rx, Tx

  Digital Pins being used:
  GPS sensor: (2)
  Servo 1 (x-dir): (9)
  Servo 2 (y-dir): (10)
*/

void setup()
{
}

void loop()
{
  //GPS Section
  long latitude = gps_location(long latitude);
  long longitude = gps_location(long longitude);
  long altitude = gps_location(long altitude);

  //Rocket Ignition Section
  
  //IMU Section

  //PID Section
  double gRoll = colaPIDr(double gRoll);   // Roll gymbal angle
  double gPitch = colaPIDp(double gPitch); // Pitch gymbal angle

  //Gimbal/Servo Section

  //Data Logging Section

  //Exit Condition
  int armed = gps_location(int armed);
  if (altitude == 0 && armed == 2)
  {
    //while (1);
    return 0;
  }
}
