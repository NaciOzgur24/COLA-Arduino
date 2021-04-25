/*
    IMU sensor on the SDA1 and SCL1 pins
*/
#ifndef _BNO055_v2_H
#define _BNO055_v2_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "BNO055_support.h"

#include 
extern TwoWire Wire1;

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

void imu_setup()
{
  Wire1.begin(); //Wire.setClock(400000); Increase I2C clock speed to 400kHz. Default at like 115200hz

  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);
  Serial1.begin(115200);
}

double imu_Euler_Angles_loop()
{
  if ((millis() - lastTime_imu2) >= 100) //Stream at 10 Hz
  {
    lastTime_imu2 = millis();
    bno055_read_euler_hrp(&myEulerData);

    double myEulerData_r = (double(myEulerData.r) / 16.00);
    Serial1.print(F(" IMU Roll: "));
    Serial1.print(myEulerData_r);
    Serial1.print(F(" Degrees "));
    
    return myEulerData_r;
  }
}

#endif // _BNO055_H
