/*
   Main code to for COLA
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
