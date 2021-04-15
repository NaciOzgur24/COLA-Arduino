/*
  COLA main code
  Serial Pins being used:
  IMU sensor: (0, 1) Rx, Tx

  Digital Pins being used:
  GPS sensor: (SCL SDA)
  Servo_x: (9)
  Servo_y: (10)
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


  // Rocket Ignition
  //ignitor();


  /*
  // Data Logging
  long ignited_altitude = ignitor();
  */


  /*
  // Exit Condition
  int exit_armed = Ignition_Condition();
  if (exit_armed == 1 && altitude == 0)
  {
    while (1);
  }
  */
}
