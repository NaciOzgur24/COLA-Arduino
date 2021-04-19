/*
  Main code to for COLA's (Test Plan) 
*/

#include "GPS.h"
#include "IMU.h"
#include "Servo_Control.h"
#include "Rocket_Ignition.h"
#include "Data_Logging.h"
#include "COLAPID.h" //Might not need to include it here

void setup()
{
  gps_setup();
  //imu_setup(); ***Not done yet***
  //Servo_setup();
  //rocket_ignition_setup();
  //Data_Logger_setup();
}

void loop()
{
  // GPS Section
  double latitude = gps_latitude();
  double longitude = gps_longitude();
  long altitude = gps_altitude();
  long ground_speed = gps_GroundSpeed();

  // IMU Section
  //imu_loop();
  //byte quaternion = imu_quaternion();
  

  // PID
  //pid_loop();


  // Servo Section
  //Servo_Control();


  // Data Logging Section
  //Data_Logger_gps();
  //Data_Logger_Rocket_Ignition();


  // Rocket Ignition Section
  //ignitor();

  /*
  // Exit Condition Section
  int exit_armed = Ignition_Condition();
  if (exit_armed == 1 && altitude <= 0)
  {
    while (1); //return 0; EXITs the code since COLA has landed. ???Move the Gimbal to a certain angle to prevent damage on the sensors???
  }
  */
}
