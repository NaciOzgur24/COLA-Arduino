/*
    COLA IMU CALIBRATION & Euler Angles
*/

#ifndef _BNO055_H
#define _BNO055_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "BNO055_support.h"

struct bno055_t myBNO;
unsigned char accelCalibStatus = 0;   //Variable to hold the calibration status of the Accelerometer
unsigned char magCalibStatus = 0;   //Variable to hold the calibration status of the Magnetometer
unsigned char gyroCalibStatus = 0;    //Variable to hold the calibration status of the Gyroscope
unsigned char sysCalibStatus = 0;   //Variable to hold the calibration status of the System (BNO055's MCU)

struct bno055_euler myEulerData; //Structure to hold the Euler data

long lastTime_imu1 = 0;
long lastTime_imu2 = 0;
long lastTime_imu3 = 0;
long lastTime_imu4 = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
//Adafruit_BNO055 bno = Adafruit_BNO055(55,0x29);


void imu_setup() //This code is executed once
{
  //Initialize I2C communication
  Wire.begin(); //Wire.setClock(400000); Increase I2C clock speed to 400kHz. Default at like 115200hz

  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);
  Serial.begin(115200);
}

void imu_calibration_loop()
{
  if ((millis() - lastTime_imu1) >= 200) //To read calibration status at 5 Hz without using additional timers
  {
    lastTime_imu1 = millis();

    Serial.print("Time Stamp: ");     //To read out the Time Stamp
    Serial.println(lastTime_imu1);

    bno055_get_accelcalib_status(&accelCalibStatus);
    Serial.print("Is the Accelerometer working (1-Yes / 0-No): ");
    Serial.println(accelCalibStatus);
    Serial.println();
  }
}

float imu_Euler_heading()
{
  if ((millis() - lastTime_imu2) >= 100) //To stream at 10 Hz without using additional timers
  {
    lastTime_imu2 = millis();
    bno055_read_euler_hrp(&myEulerData); //Updates Euler data into the structure
    
    float myEulerData_h = float(myEulerData.h) / 16.00;
    
    Serial.println();
    return myEulerData_h;
  }
}

float imu_Euler_roll()
{
  if ((millis() - lastTime_imu3) >= 100)
  {
    lastTime_imu3 = millis();
    bno055_read_euler_hrp(&myEulerData);
    
    float myEulerData_r = float(myEulerData.r) / 16.00;
    
    Serial.println();
    return myEulerData_r;
  }
}

float imu_Euler_pitch()
{
  if ((millis() - lastTime_imu4) >= 100)
  {
    lastTime_imu4 = millis();
    bno055_read_euler_hrp(&myEulerData);
    
    float myEulerData_p = float(myEulerData.p) / 16.00;
    
    Serial.println();
    return myEulerData_p;
  }
}

#endif // _BNO055_H
