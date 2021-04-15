/*
  COLA main code
  Serial Pins being used:
  IMU sensor: (0, 1) Rx, Tx
  Data Logging: (15, 14) Rx, Tx and *** MAYBE -> (17, 16) Rx, Tx ***

  Digital Pins being used:
  GPS sensor: (2)
  Servo 1 (x-dir): (9)
  Servo 2 (y-dir): (10)
*/

#include "Servo_Control.h"
#include "GPS.h"
#include "IMU.h"
#include "Data_Logging.h"
#include "Rocket_Ignition.h"

void setup()
{
  gps_setup();
  //Servo_setup();
  //rocket_ignition_setup();
}

void loop()
{
  // GPS Code
  long latitude = gps_latitude();
  long longitude = gps_longitude();
  long altitude = gps_altitude();

  int altitude_condition = Ignition_Condition();

  // IMU Code

  // Servo Code
  //Servo_Control();

  // Data Logging

  /*
  // Exit Condition
  ignitor();
  int armed = Ignition_Condition();
  int exit_armed = Ignition_Condition();
  if (exit_armed == 1 && altitude == 0)
  {
    while (1);
  }
  */
}
